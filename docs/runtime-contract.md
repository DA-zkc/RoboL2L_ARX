# Runtime 与实验协议

## CaP 主流程

live 实验固定为 `pipeline=cap`、`execution_policy=read_perceive_plan`。本项目独立实现 `FrankaRuntime`，使用同一 `franka_cap` SDK 和站点配置，不依赖旧项目源码。

| 方法 | 契约 |
|---|---|
| validate(config) | 核对站点且 motion_enabled=false；无动作 |
| describe() | 返回主流程只读/感知/规划接口契约 |
| observe() | 重新读取双 RGB-D 与机器人状态，附供模型使用的图像 |
| execute_program(source, should_pause) | 全树校验后受限解释执行，保存源码/结果；错误作为反馈 |
| validation_evidence() | 从本次 SDK 调用记录计算验收项，不能由模型返回值自证通过 |
| close() | 释放资源，无隐式动作 |

模型响应是 JSON：`program` 包含 reason 和 source；`finish` / `assist` 只包含 reason。source 必须是有界 `def run(api)`。禁止 import、shell、任意文件/网络调用以及运动方法。每轮重新观察；保留当前图像、任务/验收标准、冻结 AC/C、SDK 契约和最近 16 条去除大数组后的反馈。完整证据持久化，旧图像 base64 不重复入上下文。

`finish` 仅在机器人读取、双 RGB-D、非空颜色候选、表面定位及 checked 离线计划全部有记录时变成 `validated`。`physical_task_success=false`，没有机械臂/夹爪运动。颜色候选不能证明语义，局部规划不是完整抓放轨迹。

## 状态与恢复

ready → observing → reasoning → acting → verifying → observing。程序失败或响应格式错误回馈模型；assist / 轮次预算耗尽进入 waiting_operator。模型传输异常进入 error。每次模型请求计入 max_actions（CaP 模型轮次上限）；actions 记录程序尝试轮次。

暂停在观测、模型响应和程序 API 调用边界检查，不能取消正在进行的阻塞调用。恢复重新观察并重新生成程序，不重放已中断程序。waiting_operator 恢复必须附人工说明。新尝试归档旧证据；进程重启将活动尝试标为 interrupted，绝不自动恢复。

演示仍保留原确定性 fixture 工具循环，不调用模型或设备，completed 不代表物理成功。

## HTTP

GET `/api/state`、`/api/export`。
POST `/api/experiments`（JSON）、`/api/experiments/import`（原始 ZIP body）。
POST `/api/select`、`/api/config`、`/api/start`（confirmed=true）、`/api/pause`、`/api/resume`、`/api/stop`。
写请求需 `X-Console-Request: 1`，只允许 loopback Host 与同源 Origin。不存在可用的直接运动 HTTP API。CAP 工具区与主流程共享硬件锁。

模型密钥只来自服务器环境变量，ZIP/面板/记录只保存环境变量名。站点配置、标定与运动限制不从 ZIP 自动安装。
