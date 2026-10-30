# SetpointRamp 位置目标斜坡

`SetpointRamp` 在位置环之前生成带速度、加速度限制的平滑目标角。

默认参数：

```text
最大角速度：120 °/s
最大角加速度：600 °/s²
```

它只改变目标参考，不改变 PID 公式。小角度形成三角形速度轨迹，大角度形成梯形轨迹。

测试：

```powershell
powershell -ExecutionPolicy Bypass -File .\Tests\SetpointRamp\host\run.ps1
```

测试覆盖 5°、30°、100° 阶跃，速度/加速度限制，359°↔1° 最短路径以及运行中目标改变。
