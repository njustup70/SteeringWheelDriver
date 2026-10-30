# MasterCan

## 目标

解码主控控制帧和心跳帧，并向应用提供：

- 最后收到的 ERPM 和角度目标；
- 是否已经收到控制目标；
- 主控链路是否仍有效；
- 控制帧和心跳帧时间戳。

## 协议

### 控制帧

- 标准数据帧，ID `0x100 + wheel_id`；
- DLC 8；
- `Data[0..3]`：大端 `int32 ERPM`；
- `Data[4..7]`：大端角度码，合法范围 `0..16383`。

角度换算：

```text
target_angle_deg = angle_code * 360 / 16384
```

越界角度码会被夹紧到最后合法值。

### 心跳帧

- 标准数据帧，ID `0x110 + wheel_id`；
- DLC 8；
- 负载不参与控制。

## 持久命令语义

主控只在目标变化时发送控制帧，因此目标不能因为“控制帧变旧”而失效。

```text
control_seen = 当前有效链路周期内是否收到过控制目标
link_valid   = 控制帧或心跳是否仍在 500 ms 内
```

应用最终使用：

```c
if (master.link_valid && master.control_seen) {
    /* 可以执行 drive_erpm 和 target_angle_deg */
}
```

## 超时与恢复

- 控制帧同时刷新控制时间戳和心跳时间戳；
- 心跳处理前先按旧时间戳执行一次更新，以检测是否已经断链；
- 链路失效时清除 `control_seen`，但保留最后 ERPM 和目标角用于诊断；
- 心跳恢复只恢复 `link_valid`，旧目标不会自动复活；
- 必须收到新的控制帧，才能重新置位 `control_seen`。

所有时间差使用 `uint32_t` 无符号减法，因此支持毫秒时间戳自然回绕。

## 边界

- 只接受标准数据帧；
- 只接受 DLC 8；
- 错误轮号、扩展帧和远程帧会被忽略；
- 模块不配置 CAN 过滤器，也不启动 CAN。

## 测试

```powershell
powershell -ExecutionPolicy Bypass -File .\Tests\MasterCan\host\run.ps1
```

测试覆盖轮号、大小端、角度边界、链路空闲、故障恢复和时间回绕。
