# CaP SDK 入口

当前使用 [ASPIRE CaP 对齐清单](aspire-cap-parity.md) 与 [控制面板](operator-panel.md)。公共类型、24 个工具和 Python 程序执行器复用 ASPIRE，设备通过 `robol2l_deploy` Runtime 接入。

旧 `franka_cap.CapSDK` / `def run(api)` 是早期只读实验入口，不是当前面板或 CaP 的动作通路。`franka_cap` 中仍复用 FR3 运动学、配置、设备标定和相机工具；没有第二套真机控制器。

当前程序直接调用公开函数，使用类型属性并设置顶层 `result`：

```python
state = get_robot_state()
plan = plan_motion(state.joint_positions["primary"])
print(plan.ok)
result = plan
```

这段仅生成计划。执行通过 `execute_trajectory` 或公开便捷运动接口，全部由唯一 Runtime 派发。人工与 CaP 使用相同入口；连接或服务重载不会自动启用运动。
