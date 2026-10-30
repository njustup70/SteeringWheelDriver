# 主控 CAN 模块测试

本目录包含 `MasterCan` 的主机端测试和 STM32F405 板级接收测试。

## 协议

- 控制帧：标准帧 `0x100 + wheel_id`，DLC 8；
  - `Data[0..3]`：大端 `int32 ERPM`；
  - `Data[4..7]`：大端角度码，范围 `0..16383`。
- 心跳帧：标准帧 `0x110 + wheel_id`，DLC 8。
- 本次板级测试使用轮号 1：
  - 控制帧 `0x101`；
  - 心跳帧 `0x111`。

主控每 80 ms 发送心跳；ERPM 或角度目标变化时立即发送控制帧，不周期重发控制帧。

## 命令与链路语义

- 收到控制帧后，ERPM、角度码和目标角持续有效，不会因控制帧本身变旧而失效。
- `control_seen` 表示本次有效链路周期内已经收到过控制目标。
- `link_valid` 由控制帧或心跳帧维持，当前有效期为 500 ms。
- 最终应用应使用 `control_seen && link_valid` 判断目标是否可执行。
- 链路超时后清除 `control_seen`，但保留最后一次解码值供串口观察。
- 链路恢复时，单独的心跳帧只能恢复 `link_valid`；必须收到新的控制帧才能恢复 `control_seen`。

因此，主控保持目标不变时只需持续发送心跳，不需要周期重发控制帧。

## 板级测试接线

| 信号 | 连接 |
|---|---|
| 主控 CAN_H/CAN_L | 分控板 CAN2_H/CAN2_L |
| 主控 GND | 分控板 GND |
| USB-TTL RX | PA9（USART1_TX） |
| USB-TTL GND | 分控板 GND |

CAN2 使用 PB12/PB13，波特率为 `1 Mbps`。总线两端各接 `120 Ω` 终端电阻。

本次测试不连接 GM6020 或 VESC，也不启动 CAN1。

## 固件启动

`main.c` 的测试顺序：

1. `MasterCanTest_Init(&hcan2, &huart1)`；
2. `BSP_CAN_StartMasterFilter(&hcan2, 1)`；
3. `HAL_TIM_Base_Start_IT(&htim6)`；
4. 主循环调用 `MasterCanTest_Loop()`。

CAN2 过滤器只接收 `0x101` 和 `0x111`。TIM6 以 1 kHz 更新时间戳并执行链路超时判断。

## LED

LED 均为低电平点亮：

- PB4：测试心跳；
- PB5：`link_valid`，主控链路正常；
- PB6：`control_seen`，当前链路周期内已经收到目标；
- PB7：收到有效控制帧后短暂点亮 100 ms。

## UART 遥测

USART1：`115200、8N1`，50 Hz，FireWater 格式输出 20 个通道：

```text
当前时间ms,轮号,控制ID,心跳ID,目标ERPM,角度码,目标角×100,
control_seen,link_valid,控制帧年龄ms,心跳帧年龄ms,
控制帧计数,心跳帧计数,无效帧计数,
CAN错误码,ESR,TSR,RF0R,CAN状态,RX FIFO填充量
```

串口只输出解码状态，不输出原始 CAN 字节。

建议在 Vofa 中绘制：

- `目标ERPM`；
- `角度码`；
- `目标角×100`；
- `link_valid`；
- `control_seen`；
- 控制帧年龄和心跳帧年龄。

## 主机端测试

运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\Tests\MasterCan\host\run.ps1
```

覆盖内容包括：

- 轮号 1 和 4 的 ID；
- 大端 ERPM 和角度码解码；
- 角度码边界处理；
- 标准帧、数据帧和 DLC 8 校验；
- 控制帧建立持久目标；
- 心跳维持 `link_valid`；
- 500 ms 链路边界；
- 链路超时清除 `control_seen` 并保留最后目标；
- 心跳恢复但不恢复旧目标；
- 新控制帧恢复目标；
- 毫秒时间戳回绕。

## 板级测试步骤

1. 不连接主控，烧录并上电，确认 LED 自检后 PB4 心跳，PB5/PB6 熄灭。
2. 串口应显示 `link_valid=0`、`control_seen=0`。
3. 连接主控 CAN2，确认 PB5 点亮，心跳帧计数约每秒增加 12–13 次。
4. 在主控修改一次 ERPM 或角度目标，确认只增加一个控制帧计数。
5. 检查串口 ERPM、角度码和目标角与主控下发值一致。
6. 保持目标不变超过 1 秒，确认 `control_seen` 和 `link_valid` 仍为 1，解码目标保持不变。
7. 断开主控 CAN2，确认链路最多保持约 500 ms，随后 `link_valid` 和 `control_seen` 变为 0，最后目标值仍保留。
8. 重新连接主控，确认心跳恢复后 `link_valid=1`，但 `control_seen` 仍为 0。
9. 再次修改主控目标，确认收到新控制帧后 `control_seen=1`，目标更新。
