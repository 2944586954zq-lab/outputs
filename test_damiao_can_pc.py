"""离线协议回归测试：不连接 SocketCAN，不驱动真实电机。"""

from collections import deque
import struct
import time
import unittest
from unittest.mock import patch

import can

from damiao_can_pc import DamiaoMotor, run_example


class SimulatedBus:
    def __init__(self, mode=1, respond=True):
        self.registers = {0x0A: struct.pack('<I', mode),
                          0x15: struct.pack('<f', 12.5),
                          0x16: struct.pack('<f', 8.0),
                          0x17: struct.pack('<f', 28.0)}
        self.queue = deque()
        self.sent = []
        self.respond = respond
        self.closed = False
        self.feedback = None
        self.accept_writes = True

    def send(self, msg):
        self.sent.append(msg)
        data = bytes(msg.data)
        if msg.arbitration_id == 0x7FF and self.respond:
            if data[2] == 0x55 and self.accept_writes:
                self.registers[data[3]] = data[4:]
            # 写确认也进入队列；读函数必须只匹配对应的读确认。
            self.queue.append(can.Message(arbitration_id=0, is_extended_id=False,
                data=data[:4] + self.registers[data[3]]))
        elif data[:7] != b'\xff' * 7 and self.feedback is not None:
            self.queue.append(self.feedback)

    def recv(self, timeout=0):
        if self.queue:
            return self.queue.popleft()
        time.sleep(timeout)
        return None

    def shutdown(self):
        self.closed = True


class DriverTests(unittest.TestCase):
    def test_register_read_ignores_wrong_frames(self):
        bus = SimulatedBus()
        for can_id, data in [(0, '11 A0 04 7F F7 FD 1C 18'),
                             (0, '02 00 33 0A 04 00 00 00'),
                             (0, '01 00 33 09 04 00 00 00'),
                             (2, '01 00 33 0A 04 00 00 00')]:
            bus.queue.append(can.Message(arbitration_id=can_id,
                is_extended_id=False, data=bytes.fromhex(data)))
        motor = DamiaoMotor(bus, 1)
        self.assertEqual(motor.get_register(0x0A), 1)
        self.assertEqual(bytes(bus.sent[0].data), bytes.fromhex('01 00 33 0A 00 00 00 00'))
        self.assertEqual(bus.sent[0].arbitration_id, 0x7FF)

    def test_prepare_uses_actual_scales(self):
        motor = DamiaoMotor(SimulatedBus(), 1)
        motor.prepare_position_control()
        self.assertEqual((motor.pmax, motor.vmax, motor.tmax), (12.5, 8, 28))

    def test_wrong_mode_stops_before_enable(self):
        bus = SimulatedBus(mode=2)
        with patch('damiao_can_pc.can.Bus', return_value=bus):
            with self.assertRaisesRegex(RuntimeError, 'CTRL_MODE=2'):
                run_example()
        self.assertTrue(all(m.arbitration_id == 0x7FF for m in bus.sent))
        self.assertTrue(bus.closed)

    def test_set_mit_disables_current_mode_and_verifies(self):
        bus = SimulatedBus(mode=2)
        motor = DamiaoMotor(bus, 1)
        motor.configure_mit()
        self.assertEqual(bus.sent[1].arbitration_id, 0x101)
        self.assertEqual(bytes(bus.sent[1].data), b'\xff' * 7 + b'\xfd')
        self.assertEqual(bytes(bus.sent[2].data), bytes.fromhex('01 00 55 0A 01 00 00 00'))
        self.assertEqual(struct.unpack('<I', bus.registers[0x0A])[0], 1)
        self.assertFalse(any(bytes(m.data) == b'\xff' * 7 + b'\xfc' for m in bus.sent))
        self.assertFalse(any(m.data[2] == 0xAA for m in bus.sent if m.arbitration_id == 0x7FF))

    def test_failed_mode_write_is_not_success(self):
        bus = SimulatedBus(mode=2)
        bus.accept_writes = False
        with self.assertRaisesRegex(RuntimeError, '读回失败'):
            DamiaoMotor(bus, 1).configure_mit()

    def test_diagnose_does_not_enable_or_move(self):
        bus = SimulatedBus()
        with patch('damiao_can_pc.can.Bus', return_value=bus), patch('builtins.print'):
            run_example(diagnose=True)
        self.assertEqual(len(bus.sent), 4)
        self.assertTrue(all(m.arbitration_id == 0x7FF and m.data[2] == 0x33
                            for m in bus.sent))

    def test_register_timeout(self):
        with self.assertRaisesRegex(TimeoutError, '寄存器 0x0A'):
            DamiaoMotor(SimulatedBus(respond=False), 1).get_register(0x0A, timeout=.002)

    def test_stationary_mit_target_encodes_zero_velocity(self):
        bus = SimulatedBus()
        bus.feedback = can.Message(arbitration_id=0, is_extended_id=False,
            data=bytes.fromhex('11 A0 04 7F F7 FD 1C 18'))
        motor = DamiaoMotor(bus, 1)
        motor.prepare_position_control()
        seen = []
        motor.move_to_position(3.1268, on_feedback=seen.append)
        command = bus.sent[-2]
        self.assertEqual(command.arbitration_id, 1)
        encoded_velocity = (command.data[2] << 4) | (command.data[3] >> 4)
        self.assertEqual(encoded_velocity, 2047)  # 12 位量化的零点
        self.assertEqual(len(seen), 1)
        self.assertAlmostEqual(seen[0].position, 3.1268, places=3)
        self.assertEqual(bytes(bus.sent[-1].data), b'\xff' * 7 + b'\xfd')

    def test_disabled_feedback_cannot_report_arrival(self):
        bus = SimulatedBus()
        bus.feedback = can.Message(arbitration_id=0, is_extended_id=False,
            data=bytes.fromhex('01 A0 04 7F F7 FD 1C 18'))
        with self.assertRaises(TimeoutError):
            DamiaoMotor(bus, 1).move_to_position(3.1268, timeout=.01)
        self.assertEqual(bus.sent[-1].data[-1], 0xFD)

    def test_receive_waits_past_unrelated_traffic(self):
        bus = SimulatedBus()
        bus.queue.append(can.Message(arbitration_id=2, data=b'12345678',
                                     is_extended_id=False))
        bus.queue.append(can.Message(arbitration_id=0, is_extended_id=False,
            data=bytes.fromhex('11 A0 04 7F F7 FD 1C 18')))
        self.assertIsNotNone(DamiaoMotor(bus, 1).receive(timeout=.01))


if __name__ == '__main__':
    unittest.main()
