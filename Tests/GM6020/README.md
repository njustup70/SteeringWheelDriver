# GM6020 模块测试与调参

本目录包含 GM6020 驱动的主机端测试和 STM32F405 板级测试应用。

## 测试与调参的区别

- 测试：验证驱动逻辑、CAN 协议、LED、超时和安全行为，结果为通过或失败。
- 调参：在功能测试通过后，用固定的阶跃输入反复比较 PID 响应，选择超调、稳定时间和稳态误差合适的参数。
- 主机端测试先排除纯逻辑错误；板级测试负责 CAN、中断、真实电机和机械负载。

## 文件

- `firmware/gm6020_test.c/.h`：板级测试应用。
- `host/test_gm6020.c`：主机端测试。
- `host/stubs/main.h`：主机端 CAN/HAL 类型桩。
- `host/run.ps1`：编译并运行主机端测试。

## 主机端测试

在根目录用 Windows PowerShell 运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\Tests\GM6020\host\run.ps1
```

测试内容包括：

- 最短角误差 `wrap180`；
- GM6020 角度、转速、电流和温度解码；
- 软件零位和方向；
- 首次反馈目标角重置；
- 20 ms/21 ms 超时边界；
- 超时零电压和 PID 复位；
- 速度环从负积分状态恢复，避免静止时积分被冻结；
- 位置/速度抗饱和限幅跟随当前运行时限幅；
- ID 1/ID 5 的 `0x1FF`/`0x2FF` 电压槽位；
- CAN1 接收全部和 CAN2 主控精确过滤器。

主机端测试不能验证真实 CAN、NVIC、TIM6、LED 和电机响应。

## 硬件接线

| 信号 | 连接 |
|---|---|
| GM6020 CAN_H/CAN_L | 分控板 CAN1_H/CAN1_L |
| GM6020 GND | 分控板 GND |
| GM6020 24 V | 24 V 电源 |
| ST-Link SWDIO/SWCLK/GND | 分控板对应 SWD 引脚 |
| USB-TTL RX | PA9（USART1_TX） |
| USB-TTL GND | 分控板 GND |

注意：

- CAN 总线两端各接 120 Ω 终端电阻。
- 首次测试必须固定电机和舵轮机构，输出轴空载或低负载。
- 首次台架使用 `voltage_limit_raw=5000` 的安全电压限幅。
- 共地后再上电。

## 固件初始化

`main.c` 的调用顺序：

1. `GM6020Test_Init()` 初始化 GM6020、LED 和 UART；
2. `BSP_CAN_StartAcceptAll(&hcan1)` 配置 CAN1 接收全部、启动 CAN、开启 RX FIFO0 通知；
3. `HAL_TIM_Base_Start_IT(&htim6)` 最后启动 1 kHz 控制；
4. 主循环调用 `GM6020Test_Loop()`。

测试阶段 CAN2 保持停止。

## 测试模式

在 Keil Watch 中修改 `g_test_mode`：

|   值 | 模式                | 说明                                    |
| --: | ----------------- | ------------------------------------- |
|   0 | `DISABLED`        | 零输出                                   |
|   1 | `ZERO_CAL`        | 禁止输出，观察原始角                            |
|   2 | `POSITION_MANUAL` | 使用 `g_test_target_deg`                |
|   3 | `POSITION_AUTO`   | 正弦波或三角波动态位置测试                         |
|   4 | `SPEED`           | 使用 `g_test_target_rpm` 调速度环           |
|   5 | `VOLTAGE`         | 绕过位置/速度环，直接使用 `g_test_target_voltage` |

切换到需要使能的模式后，将 `g_test_enable` 设为 1。切换到 `DISABLED` 或 `ZERO_CAL` 会强制 `g_test_enable=0`。

Watch 变量：

- `g_test_target_deg`：手动模式目标角，或动态模式的原始波形角；
- `g_test_target_rpm`：速度模式目标 RPM；
- `g_test_target_voltage`：直接电压模式的目标 raw 电压；
- `g_test_zero_raw`：在线修改零位；
- `g_test_waveform`：动态波形，0=正弦波，1=三角波；
- `g_test_wave_amplitude_deg`：动态波形幅值，默认 30°；
- `g_test_wave_period_ms`：动态波形周期，默认 4000 ms；
- `g_test_use_ramp`：动态测试路径，1=经过 `SetpointRamp`，0=绕过斜坡，默认 1；
- `g_gm6020_motor.position_pid.kp/ki/kd`；
- `g_gm6020_motor.speed_pid.kp/ki/kd`；
- `g_gm6020_motor.voltage_limit_raw`；
- `g_gm6020_motor.max_speed_rpm`。
- `g_test_setpoint_ramp.output_angle_deg`：经过斜坡后的实际目标角；
- `g_test_setpoint_ramp.velocity_deg_s`：斜坡参考角速度；
- `g_test_setpoint_ramp.max_speed_deg_s`、`max_accel_deg_s2`：斜坡限制。
- `g_gm6020_motor.last_tx_status`：最近一次 CAN 发送返回值；
- `g_gm6020_motor.tx_ok_count`、`tx_busy_count`、`tx_error_count`：发送结果计数；
- `g_gm6020_motor.last_tx_std_id`、`last_tx_voltage`、`last_tx_data[0..1]`：最近一帧控制内容；
- `g_can1_error_code`、`g_can1_esr`、`g_can1_tsr`、`g_can1_rf0r`、`g_can1_state`、`g_can1_tx_free`：CAN1 硬件状态。

修改零位会先失能并清 PID，避免角度坐标变化时突然输出。

当前测试固件将机械零位配置为 `GM6020_TEST_ZERO_RAW=4096`。在该机械位置，GM6020 原始角 `4096` 会换算为 `0°`；重新上电后仍使用该零位。

## 位置目标斜坡

手动位置模式始终经过 `SetpointRamp`。自动位置模式根据 `g_test_use_ramp` 决定是否经过斜坡，两种路径最终都进入 GM6020 位置环。斜坡默认限制为：

```text
最大角速度：790 °/s
最大角加速度：4000 °/s²
```

大角度阶跃会形成加速、匀速、减速的梯形轨迹；小角度会形成三角形轨迹。速度模式和直接电压模式不经过斜坡。

调试时观察：

- `g_test_target_deg`：主控/测试设定的目标；
- `g_test_setpoint_ramp.output_angle_deg`：实际送入位置环的平滑目标；
- `g_gm6020_motor.angle_deg`：实际反馈角。

如果大角度仍然冲击明显，先降低 `max_accel_deg_s2`，再考虑降低 `max_speed_deg_s`。

## 动态响应测试

`POSITION_AUTO` 不再执行固定阶跃序列，而是在使能时记录的当前位置附近生成连续周期轨迹：

- `g_test_waveform=0`：正弦波；
- `g_test_waveform=1`：三角波；
- `g_test_wave_amplitude_deg`：相对使能位置的幅值；
- `g_test_wave_period_ms`：完整周期，建议从 4000 ms 开始。
- `g_test_use_ramp=1`：原始波形先经过 `SetpointRamp`，测试实际生产链路；
- `g_test_use_ramp=0`：原始波形直接进入位置环，测试理想参考下的闭环动态。

切换 `g_test_use_ramp` 时会以当前实际角作为新的波形中心和斜坡输出，并重新开始一个波形周期，避免切换过程产生目标跳变。默认使用实际生产链路；需要分析控制器本体时再绕过斜坡。

建议先使用 `30°` 幅值、`4000 ms` 周期，然后依次增大幅值或缩短周期。观察第 1 通道的最终目标角、第 26 通道的原始波形角、实际角、目标 RPM、实际 RPM、目标电压和反馈电流，检查斜坡整形、相位滞后、幅值衰减、超调及电压饱和。

## 直接电压测试

确认电机型号是 GM6020、ID 为 1 后，可以把模式设为 `VOLTAGE`：

```c
g_test_mode = 5;                 // VOLTAGE
g_test_enable = 1;
g_test_target_voltage = 1000;    // raw voltage
```

确认方向和小电压响应后，再逐步增加，例如 `2000`、`3000`，但不要超过 `g_gm6020_motor.voltage_limit_raw`。该模式不经过位置环、速度环或 `voltage_sign`，发送的就是限幅后的 raw 电压，适合排查电机和功率链路。电压给定范围为 `-25000..25000`。

## LED

LED 均为低电平点亮：

- PB7：GM6020 在线；超时后熄灭；
- PB6：驱动已使能；
- PB4：1 Hz 测试心跳；
- PB5：自动位置模式运行指示。

## UART 遥测

USART1：115200、8N1，50 Hz，CSV/FireWater 换行输出 26 个整数通道：

```text
目标角×100,实际角×100,目标RPM,实际RPM,目标电压,实际电流,在线,使能,模式,原始角,
TX状态,TX成功数,TX忙数,TX错误数,CAN错误码,ESR,TSR,RF0R,CAN状态,TX邮箱空闲数,
温度,最近TX ID,最近TX电压,最近TX data0,最近TX data1,原始波形角×100
```

角度缩放到 0.01°。例如 `9000` 表示 90.00°。第 1 通道是最终送入位置环的目标角；启用斜坡时它与第 26 通道的原始波形角之差表示斜坡的整形和滞后。第 1 通道会归一化到 `0..360°`，第 26 通道保留波形原始数值，因此跨越 0°/360° 边界时需要先展开再比较。

`TX状态` 的定义与 HAL 一致：0=`HAL_OK`、1=`HAL_ERROR`、2=`HAL_BUSY`、3=`HAL_TIMEOUT`。

如果 `TX状态` 长时间不是 0，或者 `TX忙数/TX错误数` 持续增长，应优先检查 CAN1 发送路径、总线 ACK、CAN_H/CAN_L 和电机型号/控制帧。

## 测试步骤

1. 不连接 GM6020，烧录并上电，确认四个 LED 自检后按心跳工作，PB7 熄灭。
2. 连接 GM6020，确认 PB7 点亮，串口显示在线为 1。
3. 进入 `ZERO_CAL`，将机构转到机械零位，记录 `g_gm6020_motor.raw_angle`，将 `g_test_zero_raw` 设为该值，并确认实际角约为 0°。
4. 保持零输出，人工正负转动舵轮，确认角度增加/减少方向符合定义。
5. 使用小电压或速度模式确认 `angle_sign`、`voltage_sign`；方向错误时修改符号，不要通过改接线掩盖问题。
6. 进入 `POSITION_MANUAL`，设定 `g_test_target_deg=10`，使能后确认电机沿最短方向运动。
7. 依次测试 +30°、−30°、+90°、−90° 和 359°↔1°。
8. 进入 `POSITION_AUTO`，选择正弦波或三角波，观察动态跟踪并记录相位滞后、幅值衰减、超调和最大电流。
9. 断开 CAN1，确认 PB7 在约 21–25 ms 内熄灭、目标电流为 0、电机失去主动力矩。
10. 恢复 CAN1，确认 PB7 点亮；目标角先重置为当前角，重新发送目标后才继续运动。

## 调参步骤

先完成零位、方向、超时和 LED 测试，再调增益。

### 速度环

1. 进入 `SPEED`，使能，`g_test_target_rpm=0`。
2. 保持 `Ki=0, Kd=0`，逐步修改 `speed_pid.kp`。
3. 用 `0 → +50 → 0 → −50 → 0 RPM` 阶跃观察响应。
4. 不出现持续振荡后再小量增加 `speed_pid.ki`，消除稳态误差。
5. 首次调试保持 `Kd=0`，因为速度差分对编码器噪声敏感。

### 位置环

1. 进入 `POSITION_MANUAL`，使能。
2. 保持 `Ki=0, Kd=0`，从 `position_pid.kp=6` 开始测试 ±10°、±30°、±90°。
3. 增大 `kp` 直到响应足够快但无持续振荡；超调过大时回调。
4. 稳态误差明显时小量增加 `position_pid.ki`。
5. 同时观察 `voltage_limit_raw` 和 `max_speed_rpm`，不要只追求响应速度。

### 积分策略

- 位置环保留 10° 积分分离，避免大角度接近目标时累积过多积分。
- 速度环默认关闭积分分离。大角度减速后可能残留负积分，若此时因速度误差较大而冻结积分，输出电压会停在突破静摩擦所需的电压以下，表现为电机有力矩但不继续运动。
- 速度环依靠条件积分抗饱和限制积分；`position_pid.output_max` 和 `speed_pid.output_max` 会在每个控制周期同步到当前的 `max_speed_rpm` 和 `voltage_limit_raw`。
- 再次调参时不要重新打开速度环积分分离。若静止时电压持续上升过多，应降低速度 `Ki` 或积分上限。

## 首版验收标准

- 方向正确，无持续振荡或飞车；
- ±30° 阶跃超调小于 15%，2 秒内进入 ±1°；
- 目标电压不超过当前限幅，实际电流不异常；
- 359°↔1° 走最短路径；
- CAN 断开后 PB7 熄灭、目标电流为 0；
- 恢复后目标角重置为当前角，不跳转。

## 记录模板

| 日期       | 位置 Kp/Ki/Kd | 速度 Kp/Ki/Kd | 电压限幅  | 阶跃  | 超调  | 稳定时间 | 稳态误差          | 最大反馈电流 | 备注          |
| -------- | ----------- | ----------- | ----- | --- | --- | ---- | ------------- | ------ | ----------- |
| 26.10.24 | 10/0/0      | 60/30/0     | 20000 | 100 | 0   | 400  | 0             | 2710   | 空载          |
| 26.10.24 | 6/0/0       | 70/50/0     | 20000 | 100 | 0   | 900  | $\approx$ 0.5 | 5100   | 上机，大角度起步冲击大 |
|          |             |             |       |     |     |      |               |        |             |
|          |             |             |       |     |     |      |               |        |             |

每次只改一组参数，保留上一轮记录，才能判断调整是否真正改善响应。
