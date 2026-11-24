# 手动摆位、“解锁”与示教闭锁恢复交接

2026-11-24，用户要求保存本次现场经历，另开会话解决机械臂解锁与手动复原问题。本文件记录现场过程和已验证回执，不代表手动摆位流程已经修好。

同日后续：已完成独立 10 秒手动摆位入口、owner 复验/失联处理及锁位诊断的软件修复，见[实现与验证](arx-manual-positioning.md)。尚未部署或真机验收；用户要求当前实验继续，不得为加载本修复中断实验。以下现场经过保留为历史证据。

## 用户实际需要

用户需要把机械臂摆到可靠支撑的位置，以便维护重启。当时只有一个人在现场，无法方便地同时托住左臂、操作鼠标并持续按住示教按钮。用户曾要求“先托住，10 秒后让左臂能被手拖动”；后来找到另一人协助，才完成本次恢复。需求是**机械臂手动引导／摆位与退出后的保持**，不要混淆为电动张开夹爪、清除派发闭锁、解除全部电机力矩或修改夹爪标定。

## 本次如何恢复

1. 用户说控制面板“解锁机械臂”点不开。当时旧 Runtime `f54f973dc0574e3383c0c5be61b8bf39` 仍直接持有 SDK，左臂尚未可靠支撑，因此没有重启 8771。独立启动器 8770 与 SAM3 可分别更新，双臂控制器保持运行。
2. 代码检查：实验页 `exp-unlock` 走通用 `Runtime.unlock()`，后者要求适配器实现 `recover()`；当前双 ARX 适配器无此方法。该通用故障恢复入口不是进入重力补偿的入口。用户那次点击的原始 HTTP 回执未保存，不要虚构其具体报错。
3. 可用的手动路径在 `/hardware/calibration` 的“按住拖动”。现实现把示教入口和标定批次绑在一起，必须有批次 ID、勾选扶臂并持续按住。为启用现有入口，本会话创建了空批次 `abc2845b2b524bc495dc308ade0c43d0`，没有采样、求解或修改外参。不能要求用户因此重做标定。
4. 随后的左臂示教退出出现 `hold_feedback_not_verified: last_error_J2=0.00458 rad, last_speed_J2=0.11868 rad/s, longest_stable=0.458 s`，`phase=error`、`faulted=true`、`motion_enabled=false`，页面显示 `station_faulted_or_paused`，提供 `can_recheck_hold=true`。这是退出示教后的锁位反馈未通过，不是证明电机已解除保持，也不等于已确认用户手抖；控制器/反馈/负载、时序等原因未隔离。
5. 用户要求延时 10 秒进入左臂拖动。本会话准备了仅调用既有 TeachKey API 的限时脚本，但**预检查发现 faulted=true，当场退出，未调用 prepare/heartbeat，倒计时未生效，也没有进入拖动**。没有留下未来会突然触发的后台动作；不要重新运行临时脚本来绕过故障。
6. 用户确认自己托稳左臂、右臂已有支撑后，调用一次 `/api/hardware/calibration/recheck-hold`，参数为当前 `id`、`side="left"`、`confirmed=true`。返回 **409 `left: park_arm_and_wait_for_stationary_feedback`**，静止检查未通过，未解除闭锁。本会话未放宽阈值或反复重发。
7. 用户随后明确“现在有人了，你先把这个状态解除”，附左臂示教异常截图。让现场继续托稳后，再走同一个现有复验入口，此次 **HTTP 200**：`phase=held`、`error=null`、`faulted=false`、`motion_enabled=false`。回执最大位置误差 **0.0030518044 rad**，最大速度 **0.021978022 rad/s**，稳定窗口 **0.505554428 s**。此动作是当前位置锁位复验及解除对应闭锁，**没有自动进入拖动**。
8. 用户随后反馈“ok了重启服务”，确认摆位完成，才通过新版启动器的 `maintenance/reload`（`hardware_supported=true`）完成旧 Runtime→常驻 owner 迁移。本会话没有指挥自动回 Home 或抓取搬运，也没有逐帧记录人工搬动轨迹；人工最终摆放完成依据用户确认，不要编造具体手动轨迹。

## 现有实现与需要排查的问题

- `robol2l_deploy/web/experiment.js`、`web/index.html`：通用“解锁机械臂”的文案、能力可用性与 ARX 行为不一致。
- `robol2l_deploy/runtime.py::unlock()`：通用 recover 接口；不能把它直接改成无差别撤销电机保持。
- `robol2l_deploy/web/calibration.js`、`calibration.html`、`web.py`：手动入口依赖标定批次，单人操作困难；prepare/heartbeat/release 协议和 UI 状态需一起审查。
- `robol2l_deploy/arx_teach.py::TeachKey`：0.65 秒心跳期限，单侧独占租约，重力补偿最多 30 秒；松键/失联/超时走 `teach_hold()`；失败则保持故障记录，支持明确确认后的 `recheck-hold`。
- `robol2l_deploy/adapters/arx_dual.py`：`teach_drag()` 调用 SDK `gravity_compensation()`；`teach_hold()` 按实测关节位置请求 JOINT 保持，并验证速度、位置与连续稳定时间。当前锁位检查要求误差 ≤0.01 rad、速度 ≤0.025 rad/s、连续约0.5秒，采样间隔 >0.15秒重置稳定计时，最多等待4秒。复验前另检查双臂速度 ≤0.05 rad/s、采样位移 ≤0.003 rad。阈值依据与反馈质量需要诊断，不能把放宽阈值当作已解决。

后续目标是提供语义明确、可用的 ARX 手动摆位和保持恢复流程，考虑单人操作，同时查明本次锁位反馈失败原因。应区分手动引导、保持位置、恢复故障、清除闭锁、启用程序运动。不要自动选取某种“解锁”等于断力方案。不要以 damp/protect/断开控制器代替手动引导；本机已证明 PROTECT 只有阻尼而无位置刚度/重力前馈，曾发生下沉。

## 证据与当前接续状态

目录：`.franka-player/diagnostics/20261124-perception-load/`。

- `manual-position-preparation.json`：空批次准备，没有进入示教或改外参。
- `left-teach-preflight-state.json`、`left-teach-preflight-calibration-status.json`：延时请求被派发前拦截时的状态。
- `left-teach-recheck.json`：单人托臂阶段的 409 静止检查失败。
- `left-teach-assisted-recheck-1795525539548953001.json`：有人协助后的复验成功及前后状态、实测回执。
- `before-maintenance.json`、`maintenance-reload.json`、`final-load-verification.json`：用户后续授权重启及加载状态。
- 现场事件流：`.franka-player/panel/sessions/7ef324daa5da4e47aa705542efc0d19d/hardware/events.jsonl`，可核对 `teach_dispatch`、`teach_held`、`teach_recheck_dispatch`、`teach_recheck_held` 和故障事件。此归档目录沿用，不等于在线 session ID。

末次核验在线 Runtime 为 `1f27ffc087fc4f8993f46995259a6b77`，owner instance `1480137d16b54f6db1d2567743135291`，`hardware_lifecycle.mode=resident_arx_owner_v1`，无故障、空闲、运动关闭。新会话先只读核查实时状态；不要把本次有人扶臂/支撑确认当作以后仍然成立的现场事实。

重载控制台现应保留硬件 owner，硬件退出或模型更换仍按 `docs/arx-resident-runtime.md` 的维护确认处理。新会话用户要求修复此流程不自动授权立即开启重力补偿、移动双臂或断开控制器。保留已有修改、标定端点与现场回执，先完成代码诊断和软件验证，再按当时明确的现场操作授权验证。
