"""达妙电机 PC 上位机 CAN 驱动。

依赖：python-can
Linux 示例：
    sudo ip link set can0 up type can bitrate 1000000
    python damiao_can_pc.py

Windows 下将 can.Bus(interface=..., channel=...) 替换为 USB-CAN 厂商
提供的 python-can backend，协议封装部分无需修改。
"""

from __future__ import annotations

import struct
import time
import argparse
import math
from dataclasses import dataclass
from typing import Callable, Optional

import can


DM_MODE_MIT = 0x000
DM_MODE_POS_VEL = 0x100
DM_MODE_SPEED = 0x200
DM_MODE_POS_CUR = 0x300

# 寄存器值与 CAN ID 偏移是两种不同的编码。
DM_CTRL_MODES = {DM_MODE_MIT: 1, DM_MODE_POS_VEL: 2,
                 DM_MODE_SPEED: 3, DM_MODE_POS_CUR: 4}
DM_REGISTERS = {"CTRL_MODE": (0x0A, "<I"), "PMAX": (0x15, "<f"),
                "VMAX": (0x16, "<f"), "TMAX": (0x17, "<f")}

# 达妙反馈帧 D[0] 高 4 位的状态码。
DM_STATE_NAMES = {
    0x0: "disabled",
    0x1: "enabled",
    0x2: "not_calibrated",
    0x8: "over_voltage",
    0x9: "under_voltage",
    0xA: "over_current",
    0xB: "mos_over_temperature",
    0xC: "coil_over_temperature",
    0xD: "communication_lost",
    0xE: "overload",
}
DM_FAULT_STATES = frozenset((0x2, 0x8, 0x9, 0xA, 0xB, 0xC, 0xD, 0xE))


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _float_to_uint(value: float, low: float, high: float, bits: int) -> int:
    value = _clamp(value, low, high)
    return int((value - low) * ((1 << bits) - 1) / (high - low))


def _uint_to_float(value: int, low: float, high: float, bits: int) -> float:
    return value * (high - low) / ((1 << bits) - 1) + low


@dataclass
class Feedback:
    motor_id: int
    state: int
    position: float
    velocity: float
    torque: float
    mos_temperature: int
    coil_temperature: int
    position_raw: int
    velocity_raw: int
    torque_raw: int

    @property
    def state_name(self) -> str:
        return DM_STATE_NAMES.get(self.state, f"unknown_0x{self.state:X}")

    @property
    def is_fault(self) -> bool:
        return self.state in DM_FAULT_STATES


class DamiaoMotor:
    """一台达妙电机的 PC 端控制对象。"""

    def __init__(
        self,
        bus: can.BusABC,
        motor_id: int,
        *,
        mode: int = DM_MODE_MIT,
        pmax: float = 12.5,
        vmax: float = 30.0,
        tmax: float = 10.0,
        feedback_id: int = 0,
    ) -> None:
        if not 0 <= motor_id <= 0x0F:
            raise ValueError("达妙反馈帧的电机 ID 必须在 0~15")
        if mode not in (DM_MODE_MIT, DM_MODE_POS_VEL,
                        DM_MODE_SPEED, DM_MODE_POS_CUR):
            raise ValueError("无效的达妙控制模式")
        if not 0 <= feedback_id <= 0x7FF:
            raise ValueError("反馈 CAN ID 必须在 0~0x7FF")
        if not all(math.isfinite(v) and v > 0 for v in (pmax, vmax, tmax)):
            raise ValueError("量程必须为正的有限数值")
        self.bus = bus
        self.motor_id = motor_id
        self.mode = mode
        self.pmax = pmax
        self.vmax = vmax
        self.tmax = tmax
        self.feedback_id = feedback_id

    def _send(self, arbitration_id: int, data: bytes) -> None:
        if not 0 <= arbitration_id <= 0x7FF:
            raise ValueError("标准 CAN ID 必须在 0~0x7FF")
        self.bus.send(can.Message(arbitration_id=arbitration_id,
                                  data=data,
                                  is_extended_id=False))

    def enable(self) -> None:
        self._send(self.motor_id + self.mode, b"\xff" * 7 + b"\xfc")

    def disable(self) -> None:
        self._send(self.motor_id + self.mode, b"\xff" * 7 + b"\xfd")

    def clear_error(self) -> None:
        self._send(self.motor_id + self.mode, b"\xff" * 7 + b"\xfb")

    def save_zero(self) -> None:
        self._send(self.motor_id + self.mode, b"\xff" * 7 + b"\xfe")

    def mit(self, position: float, velocity: float,
            kp: float, kd: float, torque: float) -> None:
        p = _float_to_uint(position, -self.pmax, self.pmax, 16)
        v = _float_to_uint(velocity, -self.vmax, self.vmax, 12)
        kp_u = _float_to_uint(kp, 0.0, 500.0, 12)
        kd_u = _float_to_uint(kd, 0.0, 5.0, 12)
        t = _float_to_uint(torque, -self.tmax, self.tmax, 12)
        data = bytes((
            p >> 8, p & 0xFF,
            v >> 4, ((v & 0x0F) << 4) | (kp_u >> 8), kp_u & 0xFF,
            kd_u >> 4, ((kd_u & 0x0F) << 4) | (t >> 8), t & 0xFF,
        ))
        self._send(self.motor_id, data)

    def position_velocity(self, position: float, velocity: float) -> None:
        self._send(self.motor_id + DM_MODE_POS_VEL,
                   struct.pack("<ff", position, velocity))

    def speed(self, velocity: float) -> None:
        self._send(self.motor_id + DM_MODE_SPEED,
                   struct.pack("<f", velocity))

    def position_current(self, position: float,
                         velocity_limit: float,
                         current_pu: float) -> None:
        velocity_u = int(_clamp(velocity_limit, 0.0, 100.0) * 100.0)
        current_u = int(_clamp(current_pu, 0.0, 1.0) * 10000.0)
        self._send(self.motor_id + DM_MODE_POS_CUR,
                   struct.pack("<fHH", position, velocity_u, current_u))

    def read_register(self, rid: int) -> None:
        self._send(0x7FF, bytes((self.motor_id, 0x00, 0x33, rid, 0, 0, 0, 0)))

    def get_register(self, rid: int, fmt: str = "<I",
                     timeout: float = 0.5) -> float | int:
        """读取寄存器；此对象应独占总线接收，不能与其他 recv 线程并用。"""
        deadline = time.monotonic() + timeout
        self.read_register(rid)
        while time.monotonic() < deadline:
            msg = self.bus.recv(timeout=max(0.0, deadline - time.monotonic()))
            if msg is None:
                break
            data = bytes(msg.data)
            if (not msg.is_extended_id and not msg.is_error_frame
                    and not msg.is_remote_frame and not msg.is_fd
                    and msg.arbitration_id == self.feedback_id
                    and len(data) == 8
                    and data[:4] == bytes((self.motor_id, 0, 0x33, rid))):
                return struct.unpack(fmt, data[4:])[0]
        raise TimeoutError(
            f"寄存器 0x{rid:02X} 读取超时；预期反馈 ID=0x{self.feedback_id:03X}。"
            "请保留 candump 输出核对固件协议和反馈 ID"
        )

    def read_configuration(self) -> dict[str, float | int]:
        return {name: self.get_register(rid, fmt)
                for name, (rid, fmt) in DM_REGISTERS.items()}

    def configure_mit(self) -> None:
        """按当前模式失能，设置 CTRL_MODE=1 并读回；不使能、不保存 Flash。"""
        current = self.get_register(0x0A)
        offsets = {value: key for key, value in DM_CTRL_MODES.items()}
        if current not in offsets:
            raise ValueError(f"未知 CTRL_MODE={current}，停止配置")
        self._send(self.motor_id + offsets[current], b"\xff" * 7 + b"\xfd")
        time.sleep(0.05)
        self.write_register(0x0A, struct.pack("<I", 1))
        time.sleep(0.05)
        actual = self.get_register(0x0A)
        if actual != 1:
            raise RuntimeError(f"MIT 配置读回失败：CTRL_MODE={actual}，期望 1")
        self.mode = DM_MODE_MIT

    def prepare_position_control(self) -> dict[str, float | int]:
        """运动前确认内部模式，并使用电机实际量程编解码。"""
        config = self.read_configuration()
        if config["CTRL_MODE"] != DM_CTRL_MODES[self.mode]:
            raise RuntimeError(
                f"电机内部 CTRL_MODE={config['CTRL_MODE']} 与发送模式不符；"
                "MIT 需要 CTRL_MODE=1。先运行 --set-mit 配置并读回验证"
            )
        scales = [float(config[name]) for name in ("PMAX", "VMAX", "TMAX")]
        if not all(math.isfinite(value) and value > 0 for value in scales):
            raise ValueError(f"电机返回无效量程：{config}")
        self.pmax, self.vmax, self.tmax = scales
        return config

    def write_register(self, rid: int, value: bytes) -> None:
        if len(value) != 4:
            raise ValueError("寄存器数据必须为 4 字节小端数据")
        self._send(0x7FF, bytes((self.motor_id, 0x00, 0x55, rid)) + value)

    def save_parameters(self) -> None:
        self._send(0x7FF, bytes((self.motor_id, 0x00, 0xAA, 0x01)))

    def decode_feedback(self, data: bytes) -> Feedback:
        if len(data) != 8:
            raise ValueError("反馈帧必须为 8 字节")
        p_raw = (data[1] << 8) | data[2]
        v_raw = (data[3] << 4) | (data[4] >> 4)
        t_raw = ((data[4] & 0x0F) << 8) | data[5]
        return Feedback(
            motor_id=data[0] & 0x0F,
            state=data[0] >> 4,
            position=_uint_to_float(p_raw, -self.pmax, self.pmax, 16),
            velocity=_uint_to_float(v_raw, -self.vmax, self.vmax, 12),
            torque=_uint_to_float(t_raw, -self.tmax, self.tmax, 12),
            mos_temperature=data[6],
            coil_temperature=data[7],
            position_raw=p_raw,
            velocity_raw=v_raw,
            torque_raw=t_raw,
        )

    def receive(self, timeout: float = 0.0) -> Optional[Feedback]:
        """读取一帧反馈；timeout=0 表示立即返回。"""
        deadline = time.monotonic() + timeout
        while True:
            msg = self.bus.recv(timeout=max(0.0, deadline - time.monotonic()))
            if msg is None:
                return None
            if (not msg.is_extended_id and
                    not msg.is_error_frame and not msg.is_remote_frame and
                    not msg.is_fd and
                    msg.arbitration_id == self.feedback_id and
                    len(msg.data) == 8):
                if (msg.data[0] == self.motor_id and msg.data[1] == 0
                        and msg.data[2] in (0x33, 0x55)):
                    if time.monotonic() >= deadline:
                        return None
                    continue
                feedback = self.decode_feedback(bytes(msg.data))
                if feedback.motor_id == self.motor_id:
                    return feedback
            if time.monotonic() >= deadline:
                return None

    def move_to_position(
        self,
        target_position: float,
        *,
        velocity: float = 0.0,
        kp: float = 30.0,
        kd: float = 1.0,
        torque: float = 0.0,
        position_tolerance: float = 0.02,
        velocity_tolerance: float = 0.10,
        timeout: float = 10.0,
        command_period: float = 0.005,
        on_feedback: Optional[Callable[[Feedback], None]] = None,
        raise_on_fault: bool = True,
        control_mode: str = "mit",
    ) -> Feedback:
        """发送位置目标并实时接收反馈，直到到位或超时。

        参数单位：位置 rad、速度 rad/s、扭矩 N*m。MIT 的 velocity 为
        速度前馈（不是限速），定点定位用 0；kp > 0，kd >= 0。
        运动前调用 prepare_position_control() 核对内部模式和实际量程。
        ``on_feedback`` 每收到一帧匹配的反馈就调用一次，可用于打印或记录。
        到位返回最后一帧反馈；超时或检测到故障时抛出异常。函数退出时
        始终发送失能命令。
        """
        values = (target_position, velocity, kp, kd, torque, position_tolerance,
                  velocity_tolerance, timeout, command_period)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("运动参数必须为有限数值")
        if not -self.pmax <= target_position <= self.pmax:
            raise ValueError(f"target_position 必须在 {-self.pmax}~{self.pmax} rad")
        if control_mode == "pos_vel" and velocity <= 0.0:
            raise ValueError("pos_vel 的速度上限必须大于 0")
        if control_mode not in ("mit", "pos_vel"):
            raise ValueError("control_mode 必须是 mit 或 pos_vel")
        if control_mode == "mit" and (kp <= 0.0 or kd < 0.0):
            raise ValueError("位置控制要求 kp 大于 0，kd 不能为负数")
        if control_mode == "mit" and (kp > 500 or kd > 5
                or abs(velocity) > self.vmax or abs(torque) > self.tmax):
            raise ValueError("MIT 参数超出量程或增益范围（kp<=500，kd<=5）")
        if control_mode == "pos_vel" and (kp < 0.0 or kd < 0.0):
            raise ValueError("kp 和 kd 不能为负数")
        if position_tolerance <= 0.0 or velocity_tolerance < 0.0:
            raise ValueError("位置容差必须大于 0，速度容差不能为负数")
        if timeout <= 0.0 or command_period <= 0.0:
            raise ValueError("timeout 和 command_period 必须大于 0")
        expected_mode = DM_MODE_MIT if control_mode == "mit" else DM_MODE_POS_VEL
        if self.mode != expected_mode:
            raise ValueError(
                f"当前 mode=0x{self.mode:X} 与 control_mode={control_mode} 不匹配"
            )

        deadline = time.monotonic() + timeout
        last_feedback: Optional[Feedback] = None
        feedback_count = 0
        try:
            self.enable()
            while True:
                if time.monotonic() >= deadline:
                    if last_feedback is None:
                        raise TimeoutError(
                            f"电机未在 {timeout:g} s 内到达 {target_position:g} rad；"
                            "期间没有收到匹配的反馈帧，请检查反馈 CAN ID、"
                            "电机 ID、CAN 接线和波特率"
                        )
                    raise TimeoutError(
                        f"电机未在 {timeout:g} s 内到达 {target_position:g} rad；"
                        f"最后反馈: pos={last_feedback.position:.4f} rad, "
                        f"vel={last_feedback.velocity:.4f} rad/s, "
                        f"state=0x{last_feedback.state:X} "
                        f"({last_feedback.state_name}), "
                        f"共收到 {feedback_count} 帧"
                    )

                # 位置指令需要周期性重发，电机才会持续保持目标位置。
                if control_mode == "mit":
                    self.mit(target_position, velocity, kp, kd, torque)
                else:
                    self.position_velocity(target_position, velocity)
                cycle_deadline = min(
                    time.monotonic() + command_period, deadline
                )
                while time.monotonic() < cycle_deadline:
                    wait = cycle_deadline - time.monotonic()
                    feedback = self.receive(timeout=max(0.0, wait))
                    if feedback is None:
                        break
                    last_feedback = feedback
                    feedback_count += 1
                    if on_feedback is not None:
                        on_feedback(feedback)
                    if raise_on_fault and feedback.is_fault:
                        raise RuntimeError(
                            f"电机故障: state=0x{feedback.state:X} "
                            f"({feedback.state_name})"
                        )
                    if (
                        feedback.state == 1
                        and abs(feedback.position - target_position)
                        <= position_tolerance
                        and abs(feedback.velocity) <= velocity_tolerance
                    ):
                        return feedback
        finally:
            self.disable()


def run_example(channel: str = "can0", motor_id: int = 1,
                feedback_id: int = 0,
                control_mode: str = "mit",
                target_rad: float = 0.0,
                target_vel: float = 0.0,
                target_tor: float = 0.0,
                kp: float = 30.0,
                kd: float = 1.0,
                state: str = "enable",
                position_tolerance: float = 0.02,
                velocity_tolerance: float = 0.10,
                timeout: float = 10.0,
                command_period: float = 0.005,
                diagnose: bool = False,
                set_mit: bool = False) -> None:
    bus = can.Bus(interface="socketcan", channel=channel,
                  bitrate=1_000_000)
    mode = DM_MODE_MIT if control_mode == "mit" else DM_MODE_POS_VEL
    try:
        motor = DamiaoMotor(bus, motor_id, mode=mode, feedback_id=feedback_id)
        if set_mit:
            motor.configure_mit()
            print("CTRL_MODE=1 已读回确认；电机保持失能，未写入 Flash。")
            return
        if diagnose:
            for name, value in motor.read_configuration().items():
                print(f"{name}={value}", flush=True)
            return
        if state == "disable":
            motor.disable()
            return
        if state == "clear_error":
            motor.clear_error()
            time.sleep(0.1)
        elif state != "enable":
            raise ValueError("state 必须是 enable、disable 或 clear_error")

        config = motor.prepare_position_control()
        print(f"已读取电机配置：{config}；控制 CAN ID=0x{motor_id + mode:03X}",
              flush=True)

        def print_feedback(feedback: Feedback) -> None:
            print(f"pos={feedback.position:.4f} rad, "
                  f"vel={feedback.velocity:.4f} rad/s, "
                  f"tor={feedback.torque:.4f} N*m, "
                  f"state=0x{feedback.state:X} ({feedback.state_name}), "
                  f"mos={feedback.mos_temperature} C, "
                  f"coil={feedback.coil_temperature} C",
                  flush=True)

        final_feedback = motor.move_to_position(
            target_rad,
            velocity=target_vel,
            kp=kp,
            kd=kd,
            torque=target_tor,
            position_tolerance=position_tolerance,
            velocity_tolerance=velocity_tolerance,
            timeout=timeout,
            command_period=command_period,
            on_feedback=print_feedback,
            control_mode=control_mode,
        )
        print(f"目标位置已到达: {final_feedback.position:.4f} rad")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="达妙电机 MIT CAN 控制")
    parser.add_argument("--channel", default="can0", help="SocketCAN 接口")
    parser.add_argument("--motor-id", type=int, default=1)
    parser.add_argument("--feedback-id", type=lambda value: int(value, 0),
                        default=0,
                        help="反馈帧 CAN ID，默认 0；支持 0x 前缀")
    parser.add_argument("--control-mode", choices=("mit", "pos_vel"),
                        default="mit",
                        help="发送帧模式；运动前核对电机内部 CTRL_MODE")
    actions = parser.add_mutually_exclusive_group()
    actions.add_argument("--diagnose", action="store_true",
                         help="只读取内部模式和量程，不使能或运动")
    actions.add_argument("--set-mit", action="store_true",
                         help="失能后设置 CTRL_MODE=1 并读回，不运动或保存 Flash")
    parser.add_argument("--rad", dest="target_rad", type=float, default=0.0,
                        help="目标位置，单位 rad")
    parser.add_argument("--vel", dest="target_vel", type=float, default=0.0,
                        help="MIT 速度前馈（定点用 0）；pos_vel 速度上限，rad/s")
    parser.add_argument("--tor", dest="target_tor", type=float, default=0.0,
                        help="目标扭矩，单位 N·m")
    parser.add_argument("--kp", type=float, default=30.0,
                        help="MIT 位置刚度；位置控制不能为 0")
    parser.add_argument("--kd", type=float, default=1.0)
    parser.add_argument("--state", choices=("enable", "disable", "clear_error"),
                        default="enable",
                        help="启动状态命令；反馈 state 仍由电机返回")
    parser.add_argument("--position-tolerance", type=float, default=0.02,
                        help="到位位置容差，单位 rad")
    parser.add_argument("--velocity-tolerance", type=float, default=0.10,
                        help="到位速度容差，单位 rad/s")
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="到位超时时间，单位 s")
    parser.add_argument("--command-period", type=float, default=0.005,
                        help="MIT 指令发送周期，单位 s")
    args = parser.parse_args()
    run_example(**vars(args))
