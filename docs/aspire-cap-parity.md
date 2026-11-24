# ASPIRE CaP 对齐（2026-11-19）

对照本地 ASPIRE `fa00f0ef2bc1d91d594b9aa502c22133c9f538f7` 的 `cap_harness/api.py`、`registry.py`、`runtime.py` 与 `yam_real/adapter.py`。这份清单描述当前入口；旧实验记录不改写。

当前链路：**ASPIRE CapApi / ProgramExecutor → StationEnvironment / StationPlanner → 唯一 StationRuntime → 启动时选定的 panda_py 或 franky → FR3**。相机与感知 provider 由同一 Runtime 组织。没有增加第二个机械臂控制器，也没有把 YAM 的驱动伪装成 FR3 驱动。

## 公共接口

24 个 shared public 工具均已注册，函数签名保持 ASPIRE；真实运动是否成功仍以设备回执为准。对应模块为 `robol2l_deploy/cap/api.py`、`runtime.py`、`motion.py` 和 `adapters/`。

| 类别 | 接口 | 当前实现 |
| --- | --- | --- |
| 任务与观测 | `get_task_context`, `get_observation`, `get_robot_state` | 任务语言、冻结 RGB-D、实测机器人状态 |
| 分割 | `segment_text`, `segment_points` | 复用 SAM3 provider；未启动/缺权重时返回实际错误 |
| 几何 | `mask_to_point_cloud`, `crop_point_cloud`, `estimate_geometry`, `localize_object` | 复用 ASPIRE 几何方法；接受原始 bool mask；localize 与上游一样使用首个分割结果 |
| 抓取 | `generate_grasps`, `select_grasp` | Contact-GraspNet 与实际 Hand TCP 坐标转换，候选不是任务成功判定 |
| 规划 | `solve_ik`, `plan_motion`, `plan_synchronized_motion` | FR3 IK、原生时间参数化；支持 Pose、关节目标、显式起点与采样间隔 |
| 执行 | `execute_trajectory`, `step` | 原生控制器执行；支持自建/修改/重复调用的 Trajectory，以及 primary 单臂 SynchronizedTrajectory |
| 运动便捷接口 | `move_to_joints`, `move_to_pose`, `move_synchronized`, `go_home` | 同一个规划/执行入口；不通过仿真 servo 循环，不加终点精度成功门槛 |
| 夹爪 | `set_gripper`, `set_grippers`, `open_gripper`, `close_gripper` | 归一化开口、按臂字典、原生开合与接触抓取 |

`monitor_snapshot` 是 ASPIRE 的主机监控方法，不在这 24 个生成程序工具里；已委托 Runtime 状态缓存。不同 embodiment 的移动底盘、躯干、仿真元数据扩展不适用于固定基座单臂 FR3，没有虚构这些硬件。PyRoki/cuRobo/Mink 未配置，显式选择会如上游一样返回 `unsupported`；默认使用已安装的 station provider。SAM3、Contact-GraspNet 的配置存在不等于推理服务健康。

## 本轮移除的额外规则

- 不再要求轨迹必须预先登记、只能执行一次或必须与原计划完全一致。登记只用于复用原生曲线；新建、修改或配置变化后的轨迹按实际输入重新准备。
- 不再因观测 epoch、旧结果引用或配置版本不同拒绝 mask、点云及人工工具引用。保留来源记录；调用方负责选择正确观测，冻结画面不会暗中更新深度。
- 不再额外要求分割恰好一个目标；遵循 ASPIRE `localize_object` 的候选选择行为。
- 取消本部署的接口禁用表，接通 Home、step、关节目标、组合运动、组合夹爪和 timing 参数。
- 不截断程序 stdout，也不截断普通反馈的字符串、列表和字典。大图像/点云数组仍作为单独证据保存，Python 程序得到完整数组，模型收到 RGB 图像。
- 物理任务已经满足时，AI 可直接报告完成，不强制先执行一版程序。

此前移除的自动场景、模型碰撞拦截、目标登记、软件持物一致性、起点/终点误差阈值、50 mm 位移限制和固定执行次数不恢复。

## 硬件语义与仍然保留的规则

- `plan_motion` 返回实际原生时间曲线的采样；默认不建模、不检查场景碰撞。`collision_aware=False` 是“未检查”，不是“检查通过”。
- `start_joints` 显式提供时作为规划起点；省略时使用当前后端的起点约定（franky 的 `q_d`、panda_py 的实测 `q`）。没有额外起点偏差拒绝；原生控制器仍可拒绝不连续指令。
- 自建轨迹样本位于 `0, dt_s, …, (N−1)dt_s`，使用端点速度为零的三次样条。单样本保持一个 `dt_s`。两库执行同一系数曲线，既不把路径节点误当等时间点，也不擅自重定时。
- `step` 的目标运动时间为站点控制周期（默认 `1/30 s`），之后读取观测。它不是 Python 中的 1 kHz 实时流接口；大目标可能被真实控制器的速度/加速度约束拒绝。完整关节移动使用 `move_to_joints`。
- Franka Hand 的指令是阻塞式的。轨迹带夹爪列时，在夹爪值变化的路径点执行开合并增加停留时间；反馈明确标记 `blocking_at_change_waypoints` 和 `gripper_dwell_added`。未提供夹爪列时不重发夹爪命令。**不宣称机械臂与夹爪是逐帧同步控制。**
- `go_home` 只在显式调用时运行一次，使用 `config/station.json` 的 `home_joint_positions`；未配置时记录本会话首次实测关节位置到 Runtime 的 `home.json`。服务重载保留此参考，不重新定义，更不会自动 Home。
- 便捷运动仍接受 `tolerance` / `max_steps` 以保持签名；在本站原生整段运动中不拿它们施加仿真次数/精度门槛，实测偏差保留在反馈。
- 保留 ASPIRE 的有效类型、有限数、维度、坐标、归一化开口、只允许 `import math`、禁止私有属性等程序规则。上游要求不可信程序在隔离进程中运行；本项目的进程隔离落实这一点，不向程序暴露密钥或任意 shell。
- 保留用户 Pause、真实设备错误/失联处理、唯一设备 owner、运行中不换库、重启不续发动作；这些不是碰撞推测或精度阈值。正常原生回执之后允许下一次调用。
- 实际关节行程、夹爪行程与控制库参数域属于设备/库定义。比如已安装 panda_py 的轨迹生成器原生最小速度系数为 `0.001`；没有把库内限制描述成项目自行验收阈值。

## 人工操作与验证

打开工作台 `http://127.0.0.1:8771/hardware`，在“高级工具”的完整 API 列表选择接口。新增接口都有参数模板；`$ref` 引用先前结果，`$type` 可构造公开的 RobotAction、ArmCommand、Trajectory 等类型，不执行任意 Python。CaP 仍使用注入函数和顶层 `result` 的程序格式，无需 `run(api)`。

本轮验证覆盖全部 24 个工具的 mock 通路、ASPIRE 签名对照、实际 FR3 adapter + 假驱动、原生两库时间曲线、程序隔离/长输出、错误反馈及继续会话。软件 fixture 和原生回放均不是物理证据；本轮没有执行真机动作，也没有重新声称模型推理或抓放成功。详见 `runs/aspire-api-parity-2026-11-19/validation.json`。
