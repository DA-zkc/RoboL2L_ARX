# RoboL2L · Franka Single Arm Player

独立真机实验分支 `Franka-single-arm-player`。以 ASPIRE 的 **observe → reason → act → verify → recover** 为核心，只导入 AC / C 经验，不运行训练、AC/V、TG 或仿真 seeds 环路。原训练仓库保留在 main 与 Git 历史。

## 独立 Franka CAP SDK（本机）

已接通本项目自己的 FR3/夹爪状态、D435 RGB-D、Gemini RGB-D 观测和离线 IK/路径规划。通过受限 `def run(api)` 程序入口调用，与面板共用 SDK；运行不依赖 ASPIRE 或旧 RoboL2L 源码。**本轮只验收读取、感知和离线规划，运动接口明确禁用。** 详见 [SDK 使用与坐标契约](docs/cap-sdk.md)。

```bash
bash scripts/cap.sh describe
bash scripts/cap.sh run examples/cap/observe.py
bash scripts/cap.sh run examples/cap/plan_5mm_up.py
bash scripts/start-panel.sh  # http://127.0.0.1:8768
```

## CaP 主流程与 ZIP

live 模式：实时观测 → 模型生成代码 → 受限执行 → 反馈修正 → 离线验收。API、模型与密钥环境变量由服务器配置，ZIP 不携带密钥。当前任务 ZIP 位于 `real_expriment/`（保留用户指定拼写），AC/C 留空。`validated` 仅表示读取、感知和局部规划能力通过，不表示绿色方块已实际放进盘子。

见 [ZIP 协议](docs/experiment-bundles.md) 和 [主流程协议](docs/runtime-contract.md)。

## 立即查看演示面板

Python 3.10+，仅标准库，无需安装依赖：

```bash
cd /root/codebases/pengbo/RoboL2L
python3 -m franka_player --port 8765
```

服务仅绑定服务器的 `127.0.0.1`。在自己的电脑建立 SSH 隧道后打开 http://127.0.0.1:8765 ：

```bash
ssh -N -L 8765:127.0.0.1:8765 -p 2222 zizhuo_wang:litchi_data_015@114.67.229.27
```

这只是网页访问；本机不运行模型、仿真或机器人程序。

1. 点击“加载实验”，优先上传 `real_expriment/green-cube-to-plate.zip`；也可手动填写或载入 `examples/*.json`。
2. 导入 AC、C 的 Markdown/TXT 经验；原文与 SHA-256 冻结到实验。
3. 首次选择 **演示**，点击开始，查看流程高亮、工具证据与事件。
4. 演示结局选择“请求人工帮助”可检查：等待处理 → 输入回复 → 重新观察 → 继续。
5. 暂停在当前调用结束后生效。此时可以改 API / 模型 / 推理深度，再继续。
6. 先结束当前暂停/等待中的尝试，才能切换下一个或载入其他实验。
7. 导出包含全部实验、各次尝试、经验与配置历史的 JSON。导出包是记录归档，不是实验定义；单任务定义格式见 examples。

## 当前实际能力与留空部分

- 已实现：中英切换、实验创建/载入/选择、经验导入、持久化、流程图、暂停/继续、人工消息、动作预算、记录导出。
- 演示 fixture 只验证软件流程，不调用模型、不连接硬件、不运行仿真；演示 completed 不代表物理成功。
- 已实现 OpenAI-compatible Chat Completions 模型传输；API/model/station 默认空。仅在真实运行且适配器通过校验后使用。不同服务的 reasoning_effort/JSON/image 支持需自行核实；实际服务验收记录见 `runs/setup/`。
- **已提供独立 CAP 读取/规划适配器，主流程已接通 CaP 读取/感知/离线规划闭环，真实运动仍禁用。** 新增代码不发送机器人或夹爪运动；物理执行后端尚待实现和验收。
- 双相机、标定核验、IK 和采样碰撞检查已由 `franka_cap` 提供；完整场景模型、物理控制与执行验收仍待补齐。
- 主流程使用受限 `run(api)`，仅允许读取、感知和离线规划；运动方法在程序检查阶段拒绝。不自动 Home、不自动物理复位、不执行 Agent 生成的任意源码。
- 会话内上下文随工具证据保留；人工回复后重新观察。进程重启将未完成尝试标为 interrupted，绝不自动续发运动。

## 与 ASPIRE 的对应与区别

ASPIRE 当前 System 2 使用持久 CAP/MCP 会话、模型选择工具、工具返回证据、模型诊断恢复。本仓库复刻这一控制结构，保留 Runtime 边界；**尚未直接连接 ASPIRE MCP**。
本面板的暂停、人工帮助确认、实验队列和配置历史是本项目增加的操作层，不冒充 ASPIRE 的既有摆场状态机。
当前采用 OpenAI-compatible JSON 封装的 CaP 代码生成循环，未复刻 Codex CLI 的全部工具、上下文压缩和技能自动写入能力。
AC/C 仅作为显式导入的上下文，不启动 Coach 或任何训练。

来源：
- https://github.com/NVlabs/ASPIRE/blob/main/docs/system2.md
- https://github.com/NVlabs/ASPIRE/blob/main/AGENTS.md
- https://github.com/NVlabs/ASPIRE/blob/main/src/cap_harness/agent/mcp.py
- https://github.com/NVlabs/ASPIRE/blob/main/src/cap_harness/agent/session.py

## 后续接口

见 [接口契约](docs/runtime-contract.md)。服务器安装自有适配器后，传入：

```bash
python3 -m franka_player --runtime-factory your_station.runtime:create_runtime
```

模型密钥只读取服务器环境变量（默认 `ROBOT_MODEL_API_KEY`），不允许在面板上传密钥。
设置支持视觉输入/JSON 输出的模型；observation.images 中的内联图像会传给模型。
经验与证据可能含研究数据，导出前按自己的共享范围检查。

## 验证

```bash
python3 -m unittest discover -s tests -v
```

仅软件测试：fixture、状态机、并发暂停、经验快照、HTTP 访问限制。不会连接机器人、模型或仿真。

## 文件

- `franka_player/core.py`：持久实验状态与顺序闭环。
- `franka_player/runtime.py`：工具契约、模型传输、未配置/演示后端。
- `franka_player/server.py`：仅 loopback 的 HTTP API。
- `franka_player/web/`：无需构建的中英面板。
- `.franka-player/`：运行时记录，默认忽略，不提交。

## Modular hardware workbench / 模块化硬件工作台

第一阶段的独立硬件接口、持续 RGB-D 采集与工作台见 [hardware-workbench](docs/hardware-workbench.md)。使用 `--hardware-mock` 可离线验证；默认不连接真机、不启用运动。新模块与旧 CaP 入口独立选择，后续 Player/CaP 将通过同一个 Station Runtime 接入。

## Basic CaP / 基础 CaP

RoboL2L 格式的代码生成、执行反馈和修复闭环已接入统一 Station Runtime。先运行 `python -m robol2l_deploy.cap.cli mock`；默认使用明确标注的测试模型。真实模型连接、Web 入口和抓放任务准备见 [basic-cap](docs/basic-cap.md)。生成程序的真机动作仍禁用。
