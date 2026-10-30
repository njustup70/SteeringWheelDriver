# BSP_CAN

## 目标

封装 STM32F4 bxCAN 最常见的启动流程，统一保证：

```text
配置过滤器 -> HAL_CAN_Start -> 开启 RX FIFO0 通知
```

本模块不保存接收报文，不调用具体驱动解码，也不负责发送。

## 接口

```c
HAL_StatusTypeDef BSP_CAN_StartAcceptAll(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef BSP_CAN_StartMasterFilter(
    CAN_HandleTypeDef *hcan,
    uint8_t wheel_id);
```

### `BSP_CAN_StartAcceptAll`

配置 ID 和掩码全为 0 的过滤器，使 FIFO0 接收总线上的全部帧。
当前用于 CAN1：

- GM6020 使用多个标准反馈 ID；
- VESC 使用扩展状态 ID；
- 上层 RX 回调再按驱动和 ID 分发。

### `BSP_CAN_StartMasterFilter`

配置两个精确匹配的标准 ID：

```text
0x100 + wheel_id   控制帧
0x110 + wheel_id   心跳帧
```

CAN1 使用 bank 0/1，CAN2 使用 bank 14/15。`wheel_id` 当前限制为 `1..4`。

## 过滤器位数

32 位 bxCAN 过滤器中的标准 ID 位于高寄存器 bit[31:21]，
所以配置时必须左移 5 位。掩码 `0x7FF << 5` 表示 11 位标准 ID 必须全部相等。

## 错误处理

所有公开函数在过滤器配置、CAN 启动或中断通知任一步失败时返回错误。
正式应用在 `main.c` 中对返回值调用 `Error_Handler()`。

## 使用顺序

```c
MX_CAN1_Init();
MX_CAN2_Init();

BSP_CAN_StartAcceptAll(&hcan1);
BSP_CAN_StartMasterFilter(&hcan2, 1U);

HAL_TIM_Base_Start_IT(&htim6);
```

TIM6 必须最后启动，避免控制周期开始运行时 CAN 尚未就绪。

## 测试

过滤器 bank、ID 和配置顺序由 GM6020 主机测试覆盖：

```powershell
powershell -ExecutionPolicy Bypass -File .\Tests\GM6020\host\run.ps1
```
