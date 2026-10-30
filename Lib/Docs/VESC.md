# VESC

## 目标

通过扩展 CAN 帧向 VESC 发送 ERPM 指令，并解码 VESC 状态反馈。
VESC 内部已经完成速度闭环，分控板只负责给出目标。

## 协议

### 控制帧

```text
ExtId = (CAN_PACKET_SET_RPM << 8) | controller_id
      = (3 << 8) | controller_id
```

当前 `controller_id=67`，因此控制 ID 为 `0x343`。

- 扩展数据帧；
- DLC 8；
- `Data[0..3]`：大端 `int32 ERPM`；
- `Data[4..7]`：0。

### 状态帧

```text
ExtId = (CAN_PACKET_STATUS << 8) | controller_id
      = (9 << 8) | controller_id
```

当前状态 ID 为 `0x943`。

| 字节 | 内容 | 换算 |
|---|---|---|
| `Data[0..3]` | 实际 ERPM | 大端 `int32` |
| `Data[4..5]` | 电流 | 大端 `int16 / 10 A` |
| `Data[6..7]` | 占空比 | 大端 `int16 / 1000` |

## 运行状态

`VESC_ControlStep()` 应每 1 ms 调用：

- 状态帧超过 100 ms 未更新：标记离线；
- 离线或失能：发送 0 ERPM；
- 在线且使能：发送当前 `target_erpm`。

`VESC_Enable()` 从 0 目标开始；`VESC_Disable()` 立即发送 0 ERPM。

## 发送诊断

驱动保存最近发送 ID、ERPM、8 字节负载及成功/忙/错误计数。
成功计数表示 HAL 已接受发送邮箱，不代表 VESC 已经执行或应答。

## 单位

控制单位是电气转速 ERPM，不是机械 RPM。机械转速换算需要电机极对数，
本驱动不承担该换算。

## 测试

```powershell
powershell -ExecutionPolicy Bypass -File .\Tests\VESC\host\run.ps1
```

板级配置和测试说明见 `Tests/VESC/README.md`。
