# AngleOpt 单轮方向选择

`AngleOpt_Select()` 根据当前舵向角、主控目标角和目标 ERPM，在两个物理等价候选之间选择：

```text
A: target_angle,  target_erpm
B: target_angle + 180°, -target_erpm
```

选择当前角到候选角距离更短的一个，结果角度归一化到 `0..360°`。

规则：

- `target_erpm == 0`：保持目标角，不选择反向候选；
- 两个候选距离相同：保持 A，即优先不反转 ERPM；
- 初版不加入换向阈值、滞回或负载代价。

运行主机端测试：

```powershell
powershell -ExecutionPolicy Bypass -File .\Tests\AngleOpt\host\run.ps1
```

该模块不访问 CAN、GM6020、VESC 或中断状态。
