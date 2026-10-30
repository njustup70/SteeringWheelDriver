# PID

## 目标

提供可复用的位置式 PID，支持：

- 物理控制周期 `dt`；
- 输出绝对值限幅；
- 积分项输出限幅；
- 可选积分分离；
- 条件积分抗饱和；
- 角度业务与 PID 算法解耦。

## 数学形式

```text
P = Kp * error
I = Σ(Ki * error * dt)
D = Kd * (error - last_error) / dt
output = clamp(P + I + D, output_max)
```

`integral` 字段保存的是已经乘过 `Ki*dt` 的积分输出，不是原始误差和。

## 两个计算入口

### `PidCalPositionError`

调用者直接提供误差。适合角度控制，因为调用者可以先执行 `wrap180`：

```c
error = Wrap180(target_angle - current_angle);
output = PidCalPositionError(&pid, error);
```

### `PidCalPosition`

便利接口，内部计算 `target - feedback`。适合 RPM、电流等线性量，
不应直接用于跨越 0°/360° 的角度。

## 积分分离

当 `integral_separation != 0` 且 `|error| > threshold` 时，本周期不累加积分。

GM6020 当前策略：

- 位置环启用积分分离，减少大角度运动时积分堆积；
- 速度环关闭积分分离，使静止时也能积累电压克服静摩擦。

## 条件积分抗饱和

若输出已正向饱和且误差仍为正，或输出已负向饱和且误差仍为负，
本周期积分增量会被回滚。误差反向时允许积分恢复，以便退出饱和。

## 配置要求

- 所有同一次控制中的周期必须一致；
- 修改 `output_max` 后应同步给 PID，否则抗饱和判断可能与真实限幅不一致；
- `PidReset()` 只清运行状态，保留整定参数；
- 从故障恢复或重新使能时必须调用 `PidReset()`。

## 测试

```powershell
powershell -ExecutionPolicy Bypass -File .\Tests\GM6020\host\run.ps1
```

测试覆盖限幅、积分恢复、超时和与 GM6020 串级控制配合的行为。
