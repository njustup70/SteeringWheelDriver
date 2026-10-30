# SetpointRamp

## 目标

把阶跃目标角转换为满足速度上限和加速度上限的连续移动目标，
用于减小 GM6020 位置环的启动冲击。

本模块输出参考角，不直接控制电机，也不替代 PID。

## 状态

```text
desired_angle_deg  最终目标角
output_angle_deg   当前送给位置环的移动目标角
velocity_deg_s     当前参考角速度
max_speed_deg_s    速度上限
max_accel_deg_s2   加速度上限
```

## 单周期算法

1. 计算当前输出到最终目标的最短角误差；
2. 计算制动速度包络 `sqrt(2*A*|error|)`；
3. 用最大速度限制包络；
4. 让当前参考速度按 `A*dt` 接近目标速度；
5. 按 `velocity*dt` 推进输出角。

因此输出同时满足速度和加速度限制，并能在目标附近主动减速。

## 轨迹形状

- 小角度、达不到速度上限：三角形轨迹；
- 大角度：加速、匀速、减速的梯形轨迹；
- 到达角度和速度阈值后直接吸附到最终目标，避免浮点残差。

## 默认参数

```text
最大角速度：790 °/s
最大角加速度：4000 °/s²
```

这些值来自实机调参。改变机械负载或传动结构后必须重新验证。

## API 生命周期

- `Init`：初始化参数和起点；
- `Reset`：把目标和输出同步到当前实际角，适合首次使能或故障恢复；
- `SetDesired`：接收新目标，不重置当前速度；
- `Update`：按固定 `dt` 推进一次；
- `GetOutput/GetVelocity`：供控制与遥测读取。

正式应用调用 `Update(..., 0.001f)`，必须与 TIM6 1 kHz 一致。

## 测试

```powershell
powershell -ExecutionPolicy Bypass -File .\Tests\SetpointRamp\host\run.ps1
```

测试覆盖边界角、速度上限、加速度上限、跨 0° 和重置行为。
