# 达妙电机 CAN 驱动使用说明

本目录包含 STM32 HAL FDCAN 驱动（`.c/.h`）和基于 `python-can` 的 PC 驱动
（`damiao_can_pc.py`）。第 1～5 节介绍 STM32 接口，第 6～9 节介绍 PC 使用和
DM4340 实机调试记录。底层驱动覆盖：

- FIFO0 中断接收和 FIFO 排空；
- 达妙反馈帧解析；
- MIT、位置速度、速度、力位混控四种控制帧；
- 使能、失能、清除错误、设置零点；
- 寄存器读取、写入和参数保存命令。

## 1. CubeMX 配置

使用经典 CAN，标准帧，数据长度 8 字节，波特率必须与电机一致。文档示例是 1 Mbps。配置至少包含：

1. RX FIFO0 元素数量大于 0，元素大小为 8 字节；
2. TX FIFO/Queue 元素数量大于 0；
3. FDCAN 中断打开；
4. CANH、CANL 正确连接，整条总线两端各有 120 Ω 终端电阻。

`dm_can_init()` 会安装一个接收所有标准 ID 的过滤器，并启动 FDCAN。它不会替你设置时钟和波特率，这些仍由 CubeMX 或 BSP 完成。

## 2. 最小集成示例

```c
#include "damiao_can_driver.h"

extern FDCAN_HandleTypeDef hfdcan1;

static dm_can_bus_t  g_dm_bus;
static dm_motor_t    g_motor1;

static void on_motor_feedback(const dm_feedback_t *fb, void *user)
{
    dm_motor_t *motor = (dm_motor_t *)user;

    if (fb->id == motor->id) {
        motor->feedback = *fb;
    }
}

static void on_raw_frame(uint32_t can_id, const uint8_t *data,
                         uint8_t length, void *user)
{
    uint8_t rid;
    uint32_t value;
    (void)can_id;
    (void)user;

    if ((length == 8U) &&
        dm_motor_decode_reg_read_response(data, &rid, &value)) {
        /* 根据 rid 把 value 转成 float 或 uint32_t。 */
    }
}

void app_motor_init(void)
{
    /* 0 是达妙默认 master id；若已修改寄存器，则填修改后的值。 */
    dm_can_init(&g_dm_bus, &hfdcan1, 0U,
                on_motor_feedback, on_raw_frame, &g_motor1);

    /* DM4310 示例：PMAX=12.5，VMAX=30，TMAX=10。 */
    dm_motor_init(&g_motor1, &g_dm_bus, 1U,
                  12.5f, 30.0f, 10.0f, DM_MODE_MIT);
    dm_can_set_feedback_scale(&g_dm_bus, g_motor1.pmax,
                              g_motor1.vmax, g_motor1.tmax);

    dm_motor_enable(&g_motor1);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t rx_fifo0_it)
{
    (void)rx_fifo0_it;
    dm_can_rx_fifo0_callback(&g_dm_bus, hfdcan);
}

/* 例如每 1 ms 调用一次： */
void app_motor_1ms(void)
{
    dm_motor_mit(&g_motor1, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f);
}
```

## 3. 接收报文的关键流程

收到中断后，不要只读取一帧。`dm_can_rx_fifo0_callback()` 会循环读取 FIFO0，直到 FIFO 为空，避免高负载时堆积。它会：

1. 读取 `FDCAN_RxHeaderTypeDef` 和数据；
2. 把所有原始帧交给 `raw_frame_callback`；
3. 对 `feedback_can_id` 且长度为 8 的帧解析成位置、速度、扭矩和温度；
4. 通过 `feedback_callback` 回调给应用层。

达妙默认反馈 ID 是 0，反馈帧 `D[0]` 的低 4 位才是电机 ID，高 4 位是状态码。因此多电机系统应在回调中用 `fb->id` 区分电机。

## 4. 量程必须匹配

MIT 反馈的位置、速度和扭矩是定点线性映射。`pmax/vmax/tmax` 必须与电机内部的 PMAX/VMAX/TMAX 一致，否则数值会按比例缩放。`dm_can_bus_t` 默认按 12.5/30/10 解码；如果总线上的电机使用统一的其他量程，可调用 `dm_can_set_feedback_scale()` 修改。若多台电机量程不同，则在原始帧回调中按电机 ID 自行调用：

```c
dm_feedback_t fb;
dm_motor_decode_feedback(data, motor.pmax, motor.vmax,
                         motor.tmax, &fb);
```

## 5. 寄存器命令

```c
dm_motor_read_reg(&g_motor1, 0x0A);       // 读取控制模式
dm_motor_write_reg(&g_motor1, 0x0A, value); // value 为小端 4 字节
dm_motor_save_params(&g_motor1);          // 写入 Flash，失能状态下执行
```

保存参数会擦写电机内部 Flash，且一次最多约 30 ms，不能高频调用。

## 6. 使用电脑作为上位机

电脑需要通过 USB-CAN 适配器连接 CANH/CANL，不能把 USB 或普通串口直接接到电机。适配器设置为：

- 标准帧；
- 经典 CAN；
- 1 Mbps（必须与电机参数一致）；
- 总线两端有 120 Ω 终端电阻。

Python 版本驱动见 `damiao_can_pc.py`。Linux SocketCAN 示例：

```bash
cd /home/ubuntu/Documents/Codex/2026-10-09/new-chat
python3 -m pip install python-can
sudo ip link set can0 up type can bitrate 1000000
python3 outputs/damiao_can_pc.py --channel can0 --motor-id 1 --diagnose
```

以上绝对路径是本次工作区路径；复制到其他电脑时按实际路径修改。从 `~`
直接运行 `python3 outputs/damiao_can_pc.py` 会查找 `/home/ubuntu/outputs/`，
可能出现文件不存在错误。后续命令均从上述项目根目录运行。
Windows 使用 USB-CAN 厂商的 python-can backend 时，只需替换 `can.Bus(...)` 的 `interface` 和 `channel`，达妙协议封装不需要改动。

## 7. 下发位置指令并实时读取反馈

`damiao_can_pc.py` 的命令行示例会使用 MIT 模式周期性发送目标位置，直到
反馈位置和速度都进入容差范围。程序会在每帧反馈到达时打印实际位置、速度、
扭矩、状态和温度，并在到位、超时或故障时退出；退出前会发送失能命令。

```bash
python3 /home/ubuntu/Documents/Codex/2026-10-09/new-chat/outputs/damiao_can_pc.py\
  --channel can0 \
  --motor-id 1 \
  --rad 1.5708 \
  --vel 1.0 \
  --tor 0.0 \
  --kp 30.0 \
  --kd 1.0 \
  --position-tolerance 0.02 \
  --velocity-tolerance 0.10 \
  --timeout 10
```

其中 `--rad` 是目标位置（rad），`--vel` 是速度上限（rad/s），`--tor` 是
MIT 扭矩前馈（N·m）。位置控制时 `--kp` 不能为 0，具体增益应根据负载调整。
如果电机的反馈 CAN ID 已被改过，使用 `--feedback-id 0xXX` 指定它；默认反馈 ID 为 0。
反馈中的 `state=0x1 (enabled)` 表示使能，故障状态会停止本次运动。

也可以在自己的 Python 程序中使用同一个闭环接口：

```python
import can
from damiao_can_pc import DamiaoMotor, Feedback

bus = can.Bus(interface="socketcan", channel="can0", bitrate=1_000_000)
motor = DamiaoMotor(bus, motor_id=1, pmax=12.5, vmax=30.0, tmax=10.0)

def on_feedback(fb: Feedback) -> None:
    print(f"position={fb.position:.4f} rad, state={fb.state_name}")

try:
    final = motor.move_to_position(
        1.5708,
        velocity=1.0,
        kp=30.0,
        kd=1.0,
        on_feedback=on_feedback,
    )
    print("到位:", final.position)
finally:
    bus.shutdown()
```






Python 循环和回调是尽力按周期运行，不提供硬实时保证；自定义反馈回调应快速返回。

## 9. DM4340 实机调试记录与操作步骤（2026-10-10）

本次使用 DM4340、`can0`、电机 ID `1`、反馈 CAN ID `0x000`。下面的读回值
和运动结果来自本次实机终端输出，不代表所有 DM4340 的出厂参数。

### 9.1 读取内部模式和实际量程

```bash
cd /home/ubuntu/Documents/Codex/2026-10-09/new-chat
python3 outputs/damiao_can_pc.py \
  --channel can0 --motor-id 1 --diagnose
```

实际读回：

```text
CTRL_MODE=2
PMAX=6.28000020980835
VMAX=10.0
TMAX=30.0
```

`CTRL_MODE=2` 表示电机内部仍是位置速度模式；仅发送 MIT 格式不能代替内部
模式配置。PMAX 的小数尾差来自 float32 表示，实际量程约为 `6.28/10/30`。
旧程序使用 `12.5/30/10`，导致目标编码和反馈解码都不匹配。例如旧显示
`3.1268 rad` 按正确 PMAX 换算约为 `1.5709 rad`，已接近旧目标 `1.5708 rad`。

### 9.2 切换 MIT 并读回确认

```bash
python3 outputs/damiao_can_pc.py \
  --channel can0 --motor-id 1 --set-mit
```

实际输出：

```text
CTRL_MODE=1 已读回确认；电机保持失能，未写入 Flash。
```

此时已确认电机内部切换为 MIT。该操作先按原模式发送失能命令，再设置
`CTRL_MODE=1` 并读回验证，不发送运动指令。未保存 Flash，断电重启后需再次
运行 `--diagnose`；若模式不为 1，再运行 `--set-mit`。

在另一终端用 `candump can0` 可以核对报文。对于本次电机 ID `1`：

| 报文 | CAN ID / 数据特征 |
| --- | --- |
| MIT 使能 | `001 [8] FF FF FF FF FF FF FF FC` |
| MIT 控制 | `001 [8]`，数据按实际量程和目标编码 |
| 位置速度控制 | `101 [8]`，位置与速度两个小端 float32 |
| 已使能反馈 | `000 [8] 11 ...` |
| MIT 失能 | `001 [8] FF FF FF FF FF FF FF FD` |
| 已失能反馈 | `000 [8] 01 ...` |

历史抓包中的 `001 [8] 90 15 84 30 F5 33 37 FF` 使用旧量程和 `vel=1`，
不能作为当前量程下的目标命令复用。实际数据应由驱动根据读回量程生成。

### 9.3 小位移 MIT 位置测试：已观察到转动，尚未满足到位容差

量程修正后，起始位置约为 `1.57 rad`。目标改为绝对位置 `1.67 rad`，
预期位移约 `0.10 rad`（`5.7°`），不是相对转动 `1.67 rad`。

```bash
python3 outputs/damiao_can_pc.py \
  --channel can0 \
  --motor-id 1 \
  --control-mode mit \
  --rad 1.67 \
  --vel 0 \
  --tor 0 \
  --kp 10 \
  --kd 1 \
  --timeout 10
```

实际末尾反馈：

```text
pos=1.6382 rad, vel=-0.0024 rad/s, tor=0.3004 N*m, state=0x1 (enabled), mos=29 C, coil=26 C
TimeoutError: 电机未在 10 s 内到达 1.67 rad；最后反馈: pos=1.6382 rad, vel=-0.0024 rad/s, state=0x1 (enabled), 共收到 1965 帧
```

位置已从约 `1.57 rad` 变化到 `1.6382 rad`，证明此时电机已响应 MIT 位置
控制。超时原因是到位条件仍未满足：

| 判断项 | 本次值 | 默认阈值 | 是否满足 |
| --- | --- | --- | --- |
| 绝对位置误差 | `1.67 - 1.6382 = 0.0318 rad`，约 `1.82°` | ≤ `0.02 rad`，约 `1.15°` | 否 |
| 绝对速度 | `0.0024 rad/s` | ≤ `0.10 rad/s` | 是 |
| 电机状态 | `0x1` | 已使能 | 是 |

MIT 控制力矩可近似理解为 `kp × 位置误差 + kd × 速度误差 + 扭矩前馈`。
本次位置项约为 `10 × 0.0318 = 0.318 N·m`，与约 `0.30 N·m` 的反馈接近
（存在编码量化等差异）。这符合摩擦或负载下存在稳态位置误差的表现；仅靠
这些数据不能确定具体负载原因。延长超时通常不能消除已稳定不动的误差。

### 9.4 下一步增益调整：kp=20，实机结果待验证

保持目标、速度前馈和阻尼不变，将 `kp` 从 `10` 调到 `20`：

```bash
python3 outputs/damiao_can_pc.py \
  --channel can0 \
  --motor-id 1 \
  --control-mode mit \
  --rad 1.67 \
  --vel 0 \
  --tor 0 \
  --kp 20 \
  --kd 1 \
  --timeout 10
```

提高位置刚度通常能减小上述误差，但不能保证到位；截至本记录，尚未收到
`kp=20` 的实机反馈。应观察位置误差和振荡情况，出现振荡时不要继续提高
`kp`。若应用需要其他精度，可通过 `--position-tolerance` 设置，但放宽容差
只改变到位判断，不会提高实际定位精度。

### 9.5 本次驱动修改结果及退出行为

- 新增 `move_to_position()`：周期下发位置指令，每帧反馈调用 `on_feedback`，
  根据已使能状态、位置误差和速度误差判断到位。
- 增加实时位置、速度、扭矩、状态、MOS 温度和线圈温度输出；超时信息区分
  无匹配反馈与未到位，并包含最后位置、速度、状态和反馈帧数。
- 新增 `--diagnose` 和 `--set-mit`；寄存器读请求补齐为 8 字节，读回核对
  电机 ID、操作码和 RID，切换模式后必须读回确认。
- 命令行运动前检查内部模式并读取实际 PMAX/VMAX/TMAX，避免继续使用旧示例
  量程；模式不符或读取失败时不开始运动。
- MIT 速度前馈默认改为 `0`，允许定点控制使用 `--vel 0`；与位置速度模式
  的速度上限含义区分。增加有限数值、增益和量程检查。
- 到位、运动超时、默认故障处理或 Ctrl+C 中断运动时，驱动会尝试发送失能；
  命令行退出时关闭 CAN 连接。退出后不再主动保持目标位置，也不继续反馈输出。

已完成 10 项离线协议测试，覆盖寄存器响应匹配、实际量程读取、模式不匹配
拦截、MIT 写入及失败读回、只读诊断、读取超时、零速度 MIT 编码、失能状态
不可判定到位，以及接收时跳过无关报文。实机已确认模式切换和位置发生变化，
尚未确认 `kp=20` 下到位。本节新增诊断和自动量程读取属于 Python 驱动；
STM32 示例仍需由调用方配置模式和量程。