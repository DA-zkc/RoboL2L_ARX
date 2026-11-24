# 双臂 ARX 项目限制清单（2026-11-24）

用户要求：适配器只适配 CaP 与明确的项目需求，禁止未经用户明确指令或 SDK 明确依据自行增加硬件限制；包括过去认为批准过的限制，也全部列出重新审阅。此文保留删除前的审计快照，不是当前生效清单，也不是对这些限制的重新批准。

范围：当前双 X5-2023 站点配置、ARX 单臂共用层/双臂适配器、夹爪、示教/标定、常驻 owner、Runtime 和实验派发链。数值门槛、条件门禁、超时、默认降速及模型提示约束均列出。SDK 闭源内部所有保护不在可枚举范围；这里只区分项目设置与已核实的 SDK 契约。普通数组形状/有限值/刚体变换/类型检查归为接口契约，仍在表中列出。

最新状态：用户随后要求不再逐项审阅，直接删除 SDK／CaP 未规定的数值门槛。已完成的删除、保留项和离线验证见 [最新实现说明](arx-sdk-cap-only.md)。以下“保留／未删除／本次”的措辞均是早先审计当时的状态。

## 早先仅删除 A01 时的证据

实验 `ee7877f7f8e64a1d86a573d1b23ade6a/v04,v05` 指令只发给右臂；左 J1 速度分别为 0.08351648351648322、0.1098901098901095 rad/s，两份回执左臂关节位置增量均为零。`idle_arm_moved` 是项目把单次速度读数与 0.08 比较后的判断，不能称为 SDK 原生故障，也不能仅凭这些回执证明左臂发生了实际位移。

已按用户要求删除**未派发运动侧**的速度拒绝：派发前、执行中及等待结束路径均不再以 0.08 rad/s 拦截；左右对称，包含同步计划中目标未改变、实际不派发的侧。速度继续记录。未新增替代阈值。位置变化检查仍存在，列为 A03；运动侧速度、采图速度、示教速度是其他独立检查，本次未删除。

27 项离线软件回归通过，含两次历史速度、左右对称、持续超阈值反馈、同步 no-op、不误改其他臂指令及独立位置变化检查。没有重载服务、停止实验、修改在线授权或派发真机动作。**磁盘修改尚未在线生效**。这里的改动在控制台适配器，后续部署需与其他尚未部署的 owner/手动摆位修改分别核对，不能笼统称全部已加载。

证据：`.franka-player/diagnostics/20261124-arx-limit-audit/`；旧实验诊断：`.franka-player/diagnostics/20261124-waiting-operator-audit/`。当前 ARX 文件多为尚未提交文件，不能用 Git blame 虚构某次提交/作者来源。

## 依据标记

- **P**：项目代码设置；本次检查的用户记录及本机 SDK 资料中未找到该具体门槛的明确依据。这是“未找到”，不是声称完整历史绝无授权。
- **U**：找到用户明确的功能/数值要求；下文分别注明。功能授权不自动等于用户指定了每个数值。
- **S**：本机 SDK 文档或安装实现可核实；明确区分文档和实现常量。
- **C**：现有 CaP/ASPIRE 接口或既有项目流程；不是 ARX SDK 规定。

查阅了最初 ARX 接入聊天 `01a1f92e-282c-7c33-ad17-03c5627f37e6` 的全部分页、相关手动摆位/等待人工处理聊天及项目交接。初次单臂 adapter 请求仅为“adapter”，其后的初始实现包含这些局部阈值；不能把泛化的适配请求当作用户指定了 0.08 或 0.1。

明确授权记录：该聊天 turn `01a1fd07-e7ed-7ef3-93ee-2673ebcb37db` 的用户 `PLEASE IMPLEMENT THIS PLAN` 明确要求调用方 tolerance/max_steps、逐路点完成后推进、失败停止后续段、可配置碰撞检查、夹爪端点映射、设备绑定、保持当前关节时 0.15° 边界容差，并明确写了“遵守现有 0.5 秒最短运动时长和 10 毫秒时间粒度”。这不等于授权其他未写出的速度门槛。

## A. 运动、规划、到位与故障

| 编号 | 当前条件/数值 | 行为/后果 | 来源与依据 |
|---|---|---|---|
| A01 | 未运动侧速度 > **0.08 rad/s** | 原来拒绝派发或报 `idle_arm_moved`，执行中归为非恢复控制器失败；**本次已从磁盘删除，在线未加载** | [arx_dual.py:check_execution/_execute_segment](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx_dual.py:566)；P |
| A02 | 独立关节规划上限 **0.1 rad/s**；实际取 `min(SDK v_max × speed_factor, 0.1)` | 延长规划时间，给定段时长不足则拒绝；两臂都适用，不只是左臂 | `config/arx-x5-2023-dual.json:motion`、[arx.py:_minimum_duration](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx.py:263)；P，SDK README 没有此上限 |
| A03 | 未运动侧相对计划位置偏差 > **1°** | 派发前拒绝；执行中 `idle_arm_moved` → 非恢复失败、后续闭锁 | [arx_dual.py:check_execution/_execute_segment](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx_dual.py:566)；P；此次保留并列出 |
| A04 | 运动侧起点速度 > **0.08 rad/s** | `arx_arm_must_be_stationary`，拒绝派发 | [arx.py:check_execution](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx.py:312)，双臂复用；P |
| A05 | 运动侧结束速度须 ≤ **0.08 rad/s** | 没达到就继续等，超过额外 3 秒归为未静止控制器失败 | [arx_dual.py:_execute_segment](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx_dual.py:765)、[arx.py:execute](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx.py:349)；P |
| A06 | 实测起点与计划起点偏差 > **1°** | `arx_trajectory_start_state_changed`；重算最短时长大于原计划时也拒绝 | [arx.py:check_execution](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx.py:312)；P（SDK 有限位/规划不代表要求这项 1° 起点门槛） |
| A07 | 原始 `execute_trajectory` 每路点位置公差 **1°** | 不满足不进入下一路点；双臂当前返回可恢复 `goal_tolerance_not_met`，不再当电机故障 | [arx_dual.py:_execute_segment/verify_goal](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx_dual.py:765)；逐点与失败停止 U，固定 1° 数值 P |
| A08 | 高层关节/位姿移动用调用方 **tolerance**；姿态误差 **10°**；调用方 **max_steps** | 超路点预算零派发；未达精度返回失败；高层双臂最终目标没有额外叠加 1° | [runtime.py:execute_trajectory](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/runtime.py:409)、[arx_dual.py:verify_goal](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx_dual.py:693)、`cap_harness/api.py`；U/C，10° 来自 ASPIRE 约定，不是 SDK |
| A09 | 默认 **speed_factor=0.05**；允许 **0<s≤1**；配置最大值最低 **0.0001**、`max_joint_speed_rad_s` 配置最低 **0.001** | 默认降速；倍数/配置超域拒绝。ARX 原生接口接收 duration，不能把自定义 speed_factor 域直接称为 SDK 原生参数范围 | [arx.py:__init__](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx.py:37)、[drivers.py:resolve_speed_factor](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/drivers.py:8)、站点配置；C/P，未找到用户指定 0.05 |
| A10 | 最短运动 **0.5 s**、最长 **120 s**、时长 **0.01 s** 整数倍 | 规划向上量化；过短/过长/粒度不符拒绝 | [arx.py](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx.py)、[arx_dual.py:prepare_trajectory](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx_dual.py:507)；0.5/0.01 U；0.01 另有 SDK 实现 `TRAJ_QUANTUM`=0.01；120 P |
| A11 | 五次静止到静止模型峰值系数 **1.875** | 按该系数算最短时长/超速拒绝，不直接向 SDK 交由它决定时长 | [arx.py:_minimum_duration](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx.py:263)；P（数学系数不等于 SDK 执行契约） |
| A12 | 指令时长后额外 **3 s** 完成窗口；每 **0.05 s** 轮询 | 到期仍不满足局部条件→失败；单臂共用反馈函数也如此 | [arx.py:_command_with_feedback](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx.py:331)、[arx_dual.py:_execute_segment](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx_dual.py:765)；P |
| A13 | SDK 六轴关节角限位（度）：J1 −150…180、J2 0…210、J3 0…180、J4/J5 −90…90、J6 −120…120 | 越限目标拒绝；非有限/非六轴数据拒绝 | [arx.py:_joints](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx.py:185)；S，本机 `DOC_JOINT_LIMITS_DEG`；实际执行采用 `kin.q_min/q_max` |
| A14 | 仅保持当前角度且离边界 ≤ **0.15°** 时收回合法边界 | 不改原测量，记录修正；主动越限或更大偏差仍拒绝 | [arx_dual.py:_target](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx_dual.py:439) 等；U，非 SDK 原始门槛 |
| A15 | 碰撞模型完整时按最多 **1°** 关节变化采样插值路径 | 检出臂间碰撞拒绝；显式要求检查且模型缺失也拒绝；模型齐全时普通路径也会执行检查 | [arx_collision.py:PairCollision.check](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx_collision.py:60)、[arx_dual.py](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx_dual.py)；可配置避碰功能 U，采样1° P；当前缺失项应看在线 capabilities，不能声称避碰已实机验收 |
| A16 | SDK fault/offline/overlimit、结束后不是 JOINT 等 | 原生故障状态导致拒绝；项目对其包装/锁定。局部超时、未运动侧位移也被包装为 `controller_failed`，应与 SDK 故障区分 | [arx.py:get_robot_state](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx.py:197)、[arx_dual.py:_execute_segment](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx_dual.py:765)、[runtime.py:_dispatch](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/runtime.py:363)；原生状态 S，映射/持久闭锁策略 U/C/P |
| A17 | 异常后仅健康 JOINT 控制器可请求实测位置保持；不清原生故障，不强制切回模式 | 失败后取消尚未完成目标，用 SDK duration=0 重规划；夹爪保留目标，`hold_verified=false` | [arx_support.py](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx_support.py)；支撑修复 U，SDK duration=0 语义 S；不是硬件保持证明 |

## B. 夹爪

| 编号 | 当前条件/数值 | 行为/后果 | 来源与依据 |
|---|---|---|---|
| B01 | SDK 原始目标 **0…5 rad**；CaP **0…1** 映射实测端点；无标定侧不开放动作 | 越域拒绝，缺端点在运动前拒绝 | SDK README §0；[arx.py](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx.py)、[arx_dual.py:check_hands](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx_dual.py:614)；S/U/C |
| B02 | 左闭0.02/开4.869344625、右闭0.01/开4.7793 rad；空载闭位实测 ≤ **0.1 rad** | 端点映射与空载闭位记录条件 | commissioning配置；U，用户现场确认，完整重复性仍未验收 |
| B03 | 端点差至少 **0.1 rad** | 小于则拒绝加载/保存标定 | [arx.py:__init__](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx.py:37)、[arx_gripper_calibration.py:_save_closed](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/arx_gripper_calibration.py:160)；P |
| B04 | 普通夹爪指令固定 **2 s**；误差 ≤ **0.1 rad** 且速度 ≤ **0.1 rad/s**；再等最多 **3 s** | 非零目标不到位超时失败；原始单臂路径也有此门槛 | [arx_dual.py:move_hands](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx_dual.py:622)、[arx.py:set_gripper](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx.py:390)；P |
| B05 | 零值闭合停在目标之外：速度≤0.1、位置跨度≤**0.02 rad**、持续≥**0.5 s**、采样间隔≤**0.15 s**，仅在指令2s之后判断 | 可报告 `settled_short_of_closed`；窗口不满足会继续等待/超时；不证明持物 | [arx_gripper_feedback.py:ClosingFeedback](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx_gripper_feedback.py:6)；持物闭合语义修复 U，具体稳定数字 P |
| B06 | 闭合过程中位置 > 初始位置+**0.1 rad**；稳定判据的位置域 **0…开位+0.1 rad** | 判为反向张开/反馈越范围，动作失败 | [arx_dual.py:move_hands](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx_dual.py:622)、[arx_gripper_feedback.py](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx_gripper_feedback.py)；P |
| B07 | 标定点动误差 ≤ **0.035 rad**（闭位目标≤0.1时改用空载闭位域），速度≤0.1；未选侧夹爪变化≤**0.05 rad** | 标定步未核验或另一夹爪变化则失败 | [arx_dual.py:calibration_increment](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx_dual.py:588)；闭位0.1 U，其余 P |
| B08 | 标定前/确认故障时夹爪速度≤0.1；两次编码器差≤**0.025 rad**；另复用双臂静止0.05/.003 | 拒绝点动、记录或确认；采样等待0.15s，确认流程另等0.3s | [arx_gripper_calibration.py](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/arx_gripper_calibration.py)；P |
| B09 | 重复标定须先离开≥**0.1 rad**、返回差≤0.1；闭位须实际成功试过同一目标，两次目标差≤**1e−6 rad** | 重复记录不通过则不保存；已确认端点不会因未完成重复试验自动撤销 | [arx_gripper_calibration.py:record/_record_closed](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/arx_gripper_calibration.py:105)；重复验证功能有交接依据，精确数字 P；当前配置注明repeatability_verified=false |

## C. 采图、示教、锁位与外参标定

| 编号 | 当前条件/数值 | 行为/后果 | 来源与依据 |
|---|---|---|---|
| C01 | 腕相机定位采样前后关节变化≤**0.003 rad**、速度≤**0.08 rad/s** | 超出拒绝观察/定位，即便未执行运动也可能触发；此次没有删除 | [arx_dual.py:get_observation](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx_dual.py:269)；P；独立单臂旧路径变化0.01 rad、速度同0.08 |
| C02 | 图像须落在前后状态时间内，窗口≤**0.45 s**；启动等帧20s/后续5s（单臂18s/5s） | stale/timing mismatch 或取图超时；不直接代表电机故障 | [arx_dual.py:get_observation](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx_dual.py:269)、[arx.py:get_observation](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx.py:216)；P |
| C03 | 示教进入速度≤**0.05 rad/s**；停放复验双臂速度≤0.05、位置变化≤**0.003 rad** | 拒绝进入拖动/复验；夹爪标定也复用此检查 | [arx_dual.py:check_teach_start/check_teach_parked](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx_dual.py:343)；P |
| C04 | 锁位误差≤**0.01 rad**、速度≤**0.025 rad/s**、连续≥**0.5 s**、间隔≤**0.15 s**、总等候**4 s**；采样50ms、SDK cycle必须递增、JOINT模式 | 不满足会重置稳定计时，超时变教学故障；恢复入口另核对0.01rad和约0.5s静止窗口 | [arx_dual.py:teach_hold](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx_dual.py:370)、[arx_teach.py:acknowledge_parked](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/arx_teach.py:35)；锁位与核验功能 U，具体数字 P；运动最短0.5获批不等于锁位稳定窗口0.5获批 |
| C05 | 一次只示教一侧、占用 Runtime；旧按住模式心跳 **0.65 s**；新模式延迟 **10 s**后持续拖动直到明确锁位 | 旧模式松键/心跳丢失锁位；新模式不要求按住且无30s总时限；新模式需owner版本支持 | [arx_teach.py](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/arx_teach.py)；旧按住模式用户曾明确选择，10秒与取消30秒 U，0.65 P；新模式尚未部署 |
| C06 | 示教故障仅允许匹配侧/类型复验，须运动关闭、双臂支撑确认；复验失败不自动再次派发 | 其他故障或不满足条件不能通过此入口清除 | [arx_teach.py:acknowledge_parked](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/arx_teach.py:35)；恢复功能 U，细节项目实现；此前一次确认不等于持续现场确认 |
| C07 | 外参采样变化≤**0.003 rad**、速度≤**0.025 rad/s**；双图时间差≤**0.18 s**、总窗口≤**0.45 s**、后边界余量**0.02 s** | 拒绝保存标定样本 | [arx_calibration.py:sample](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/arx_calibration.py)；标定功能 U，具体数字 P |
| C08 | 标定最少**6个非共线角点**、重投影RMS≤**2 px**、每臂≥**12姿态**、旋转多样性第二主方向≥**18°**；与已有姿态同时差<**5mm/3°**视为重复 | 拒绝采样或求解；每4组留1组验算 | [arx_calibration.py:detect/solve/sample](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/arx_calibration.py:45)；具体采样门槛 P，不是 ARX SDK要求 |
| C09 | 标定板行列**3…40**、格长≤**500 mm**、标签小于格子、字典容量足够 | 拒绝板参数；形状/字典自洽检查与人为范围并存 | [arx_calibration.py](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/arx_calibration.py)；人为上限P；本站板型号/参数U |

## D. 配置、Runtime、实验与 Agent 约定（也列入复核）

| 编号 | 当前条件/数值 | 行为/后果 | 来源与依据 |
|---|---|---|---|
| D01 | TCP平移范数≤**0.5 m**，几何需来源、开度不变、verified或锁定官方模型匹配 | 超出/不匹配拒绝配置 | [arx_grasp_geometry.py](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx_grasp_geometry.py)；官方TCP接入 U，0.5m P；模型匹配是项目校验 |
| D02 | 只接受left/can1、right/can3；相机序列号、Home身份、外参、模型/网格/负载SHA及变换绑定一致；负载质量/惯量物理自洽 | 不符拒绝载入/规划/取图/维护替换 | `arx*.py`配置加载；本站绑定与负载模型 U，校验实现为项目契约；不是SDK限速 |
| D03 | 六轴顺序、有限数、正确坐标系/机器人身份；同步入口显式命名两侧；轨迹形状/整条所有路点与夹爪列预检；调用方采样周期不能超出 | 错误在首段前拒绝；双臂`step`复用0.5s运动周期，不是流式伺服；旧单臂后端仅点到点且不开放step/close_gripper | [arx.py](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx.py)、[arx_dual.py](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx_dual.py)、[motion.py](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/motion.py)、[runtime.py](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/runtime.py)；U/C/能力边界，不能把未实现称SDK禁止 |
| D04 | 默认motion_enabled=false；连接/运动授权分开；任务匹配的PhysicalSession；paused/faulted/closed/busy禁派发；一次Runtime任务/SDK owner | 派发门禁、任务串行，不新增控制器抢占 | [physical.py](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/physical.py)、[runtime.py](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/runtime.py)、[arx_owner.py](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/arx_owner.py)；U/C；没有首个2cm探测或固定派发次数硬门禁（旧字段仍存在） |
| D05 | owner租约 **3 s**、心跳 **0.5 s**、RPC **5 s**、消息 **128 KiB**；关闭等待5s、启动等待10s | 租约过期禁旧派发/请求保持；未知回复不重发；活着但无响应时拒建第二控制器；超长消息拒绝 | [arx_owner.py](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/arx_owner.py)；常驻与未知结果不重发 U，具体时间/大小是项目值；3秒写在获准迁移方案中但不是SDK要求 |
| D06 | owner白名单RPC、同一身份单租约；硬件模型更换/关闭需hardware_supported=true；关句柄结果未知保留锁；普通重载保留owner | 拒绝危险生命周期操作/非法方法；不自动断开SDK | [arx_owner.py](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/arx_owner.py)、`launcher.py`、[arx.py:close](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx.py:82)；用户常驻修复 U；依然列出供复核 |
| D07 | 控制器失败/未知结果持久闭锁；失败/暂停终止后续段；stop_on_termination必须true；故障恢复有明确确认路径，ARX不支持通用recover | 限制后续派发；重启不自动消掉未知结果；项目局部判断误归类会扩大影响 | [runtime.py](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/runtime.py)、`execution_feedback.py`、[arx_teach.py](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/arx_teach.py)；停止后续/未知锁定 U/C；不是每个controller_failed都来自SDK |
| D08 | 实验最多**8版**；连续**3次**静态/响应格式拒绝转等待；assist只接受规定外部原因与证据；复盘source不直接执行 | 拒绝第9版；错误可能进入waiting_operator | `cap/experiment.py`；C，非硬件限制；需用户重新审阅，不能拿来当SDK规定 |
| D09 | 首次冻结快照直接运行；重试复位需既有人工授权；自动小运动复位：累计关节行程和初始偏差<**0.02 rad**、TCP偏差<**5 mm**、夹爪读数变化<**0.01**且无夹爪指令、无失败/未知回执；回位另核对关节差<**0.02 rad** | 不满足自动复位条件→人工处理；回位核验失败也等待 | `cap/reset.py`、`cap/experiment.py:_restore_initial`；流程 U/C，具体阈值仍列出；不是 SDK约束 |
| D10 | 策略级get_robot_state最多约**20Hz**（间隔50ms） | 调用延迟，不改变底层SDK控制周期 | `cap/program.py`；用户授权修复状态轮询问题，具体20Hz为项目选择 |
| D11 | 可选max_calls/max_steps/timeout；默认None；生成程序受静态语法/公共工具/隔离执行限制 | 调用方设置才应用预算；不开放任意原生SDK/文件/网络入口 | `cap/program.py`、`cap/worker.py`、公共工具注册；C/U；不是固定硬件动作次数门槛 |
| D12 | 模型提示默认/推荐s=**0.05**，接触/持物时再降速；未确认接收侧持物前不放给出侧；未知结果停止、不要盲目重夹 | 对Agent的文字约束，不是独立硬件门禁；原审计误将通用 `motion_speed_guidance` 当成 ARX 实际提示；ARX 当时另有“station constrains joint speed”覆盖字段，现已修正为原生 SDK 时长语义 | `cap/api.py`、`cap/experiment.py`；C/项目提示；接口描述存在不准确处，本次如实列出 |
| D13 | 已标定RGB-D/指定相机可用、标定身份一致才提供相应定位能力；全RGB-D来源/非手工区域是旧scene验证条件 | 能力拒绝/数据来源检查；当前ARX运动不会自动触发感知或要求已识别目标 | [arx.py:get_observation](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/adapters/arx.py:216)、[physical.py:validate_scene](/home/future/Documents/ChatGPT/Robotl2l/RoboL2L/robol2l_deploy/physical.py:27)、`cap/api.py`；C/U；SAM3分数门槛是另项已要求的感知修复，不是电机许可条件 |

## SDK核对和历史已删除项

本机 `ARX5_beta/readme-CN.md` §0 明确 X5-2023夹爪0…5；§1/2明确 `duration=0` 表示最快到达；§3只是提供速度反馈，未给项目所用0.08、0.05、0.025的静止判据。只读导入控制器类（没有构造控制器）确认 `TRAJ_QUANTUM=0.01`，并保存了SDK关节范围常量。不能把我们额外的0.1rad/s上限归给SDK，也不能因为数学上能估算运动就宣称是SDK要求。

历史但不应当作当前有效的项目限制：旧精度失败立即protect/damp分支已修；旧服务退出自动damp已由常驻/维护路径替换；示教30秒总时限在磁盘新代码已取消、在线旧版状态需另核；首版实验强制回位已取消；强制2cm探测/固定派发数、固定程序大小/计划年龄等在当前通用路径没有硬门禁。`tests/test_numeric_policy_removal.py` 已有“去项目数值政策”回归，但它只检查通用speed_factor、没有覆盖ARX额外0.1rad/s，这正是现有覆盖缺口。

此轮除了A01所述删除和相应回归，未成批修改这些数值或接管Agent决策。清单列出不等于保留批准；等待用户逐项审阅后按明确范围实施。
