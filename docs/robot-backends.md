# 实验前选择 panda_py 或 franky

当前实机状态见 [2026-11-17 控制通路验证](harness-verification-2026-11-17.md)：两后端已实际运动和开合夹爪；后续抓取下降触发 cartesian_reflex，当前停止派发。下文较早的只读验证记录保留为历史。

两个后端共用 `FR3Adapter → StationRuntime → ASPIRE CapApi / ProgramExecutor`、相机、感知、规划和执行记录。仅在启动进程时选择一个库；没有热切换、混合控制或自动回退。切换时先退出上一场实验，再启动另一入口。

启动只读工作台（选一条）：

```bash
scripts/start-hardware.sh panda_py
scripts/start-hardware.sh franky
```

地址均为 `http://127.0.0.1:8767/hardware`，页面显示后端名称。两者分别保存到 `.franka-player/workbench-panda_py` 和 `.franka-player/workbench-franky`。启动命令本身不授权运动。

已有的 CaP 真机入口也支持同一个参数。在另行授权的实验中，例如一次 2 cm 离台试验：

```bash
.venv/bin/python -m robol2l_deploy.cap.live \
  --robot-backend franky \
  --task probe --probe-distance-m 0.02 \
  --authorize-physical-task --authorization '填写本次具体动作授权' \
  --codex-model gpt-6-astra \
  --codex-bin .franka-player/tools/codex/node_modules/.bin/codex
```

改成 `--robot-backend panda_py` 就使用原后端。该命令实际运行模型生成的运动程序，不是预览命令；本次适配工作没有执行它。两边均使用既有的授权、异常停止和执行收据，不自动恢复或 Home。

| 部分 | panda_py | franky |
|---|---|---|
| Python adapter | `robol2l_deploy/adapters/panda.py` | `robol2l_deploy/adapters/franky.py` |
| 时间曲线 | 已有 `panda_py.JointTrajectory` | Ruckig 0.17.3 预计算轨迹 |
| 运动入口 | 已有 `execute_joint_trajectory` | `franky.Robot.move` |
| 关节控制 | 已有 panda_py 关节阻抗控制 | libfranka 机器人内部 joint impedance |
| 夹爪 | libfranka Gripper | franky Gripper（封装 libfranka） |

franky 的薄 C++ 桥 `native/franky_station.cpp` 把冻结的 Ruckig 时间曲线交给 franky 的 Motion 接口；预览和执行采样同一对象，实时循环中没有 Python 回调。曲线结束后，直接委托上游 `franky::JointMotion` 在同一终点保持并发送 `MotionFinished`，保留上游用于滤波收敛的收尾过程。桥接层不自行按时间或误差阈值宣布运动完成，也不实现力矩控制器或修改负载、增益。第一版在路径节点处停稳，没有做路径拼接优化。

2026-11-19 删除项目偏差门槛时曾同时丢失 franky 的正常收尾，这是实现缺陷，已修复。删除的起点偏差门槛、0.003 rad 终点验收及最多 3 秒终点等待未恢复；实测偏差继续作为反馈，真实控制异常和终态不确定单独记录。IK 改用目标位姿等式约束和基于 URDF 关节范围的冗余姿态选择，不再把目标精度与贴近 seed 的代价加权混合。详见 [本次修复与软件验证](franky-ik-fix-2026-11-19.md)。

上游 `Robot` 构造器默认会写入碰撞阈值。本项目补丁增加 `configure_collision_behavior=False`，以保留现场已有设置；补丁同时注册上述轨迹桥。没有调用错误恢复、夹爪 homing 或 payload 配置接口。

安装为项目本地构建，不替换系统 libfranka，也不改变 panda_py 安装：

```bash
scripts/build-franky.sh
.venv/bin/python -m tests.check_franky_binding
.venv/bin/python -m tests.check_fr3_motion_regression
.venv/bin/python -m unittest discover -s tests -v
```

依赖版本和来源见 `config/native-dependencies.json`；构建产物与 SHA256 在 `.franka-player/native/python/franky-build.json`。上游源码与补丁分别保留在 `vendor/franky`、`patches/franky-station.patch`。安装使用本项目 `.venv`，需要本地 Eigen、libfranka 0.19、CMake、编译器与 Python 开发头文件。

2026-11-19 两后端已重新编译并安装，数字策略变更的源码/补丁/二进制哈希见 `.franka-player/native/numeric-policy-build.json`。软件检查包含 21 项针对性回归、franky 原生轨迹偏差回放及 panda_py 原生 PD/完成条件回放；此次没有执行实机运动，不构成新版控制参数的物理验证。

2026-11-17 状态：用户决定暂将 panda_py 作为可试验方案；历史失败收据未改写。franky 已编译，132 项软件回归、franky 原生轨迹正/负反馈回放以及既有 panda 原生控制器和绑定检查通过。本轮没有读取现场硬件、执行机械臂或夹爪动作，尚未验证 franky 的物理连接和运动。验证输出保存于 `.franka-player/backend-validation/2026-11-17`。

2026-11-17 后续实机只读验证：用户授权并在 Desk 开启 FCI 后，panda_py 和 franky 均已读取实际 FR3 状态、RealSense 640×480 与 Gemini 1280×800 RGB-D，完成冻结像素定位、当前末端向上 20 mm 的 IK 和轨迹预览。两次验证均无运动派发，尚不构成运动跟踪或抓放成功验收。记录见 `.franka-player/backend-validation/2026-11-17/live-readonly.json`。本次修复了启动器把 TIME_WAIT 误判为占用端口、franky.Errors 无 `__bool__` 导致空错误被误判，以及 ApiError 的 traceback 传播问题。

上游参考：[franky 源码](https://github.com/TimSchneider42/franky/tree/f88f0e9b8f598d7c537b4e727e4411e67c95e017)、[franky 文档](https://timschneider42.github.io/franky/)、[Ruckig 0.17.3](https://github.com/pantor/ruckig/tree/v0.17.3)。
