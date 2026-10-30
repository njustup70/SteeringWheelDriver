# 驱动模块文档

本目录描述 `Lib/` 中各生产模块的职责、接口、算法和使用约束。
整机接线、主控协议、LED 和串口通道请先阅读项目根目录的 `README.md`。

| 模块 | 职责 |
|---|---|
| [BSP_CAN](BSP_CAN.md) | CAN 过滤器、启动和 RX 通知 |
| [PID](PID.md) | 位置式 PID、积分分离和抗饱和 |
| [GM6020](GM6020.md) | 舵向反馈、角度换算和串级位置控制 |
| [VESC](VESC.md) | 轮向 ERPM 指令和状态反馈 |
| [MasterCan](MasterCan.md) | 主控命令解码和链路状态 |
| [AngleOpt](AngleOpt.md) | 最近物理等效舵向选择 |
| [SetpointRamp](SetpointRamp.md) | 位置目标速度和加速度限制 |

模块的主机测试分别位于 `Tests/<Module>/host/`。`Tests/*/firmware`
中的测试壳只在模块调试阶段使用，不参与当前正式固件构建。
