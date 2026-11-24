# 自动感知与 FR3 只读部署

当前主路径复用 ASPIRE 的 `CapApi`、`ToolRegistry`、`ProgramExecutor` 和 SAM3 / Contact-GraspNet provider。`robol2l_deploy` 只实现站点适配、同帧绑定、设备派发与证据记录。来源 commit、原始文件哈希和本地修改见 `cap_harness/UPSTREAM.json`；可审阅补丁见 `patches/aspire_deployment.patch`。

## 模型收到什么，与本地 ASPIRE 的关系

当前每次模型决策输入包含：任务语言、公开 API、机器人状态与相机元数据、前序程序/print/逐调用反馈/错误，以及各相机的实际 RGB JPEG 图像。API 模式使用多模态 `image_url` 内容块；Codex 模式把同一图像作为 `--image` 附件。深度不是用 JSON 展开给语言模型的百万像素列表，完整 RGB-D、内参和位姿在程序 `get_observation()` 的类型对象中，可由程序调用几何工具处理。一个程序内的 API 返回值先交给 Python，程序结束后执行反馈和新图像才进入下一次模型决策。

核对本地 ASPIRE `fa00f0ef2bc1d91d594b9aa502c22133c9f538f7` 的 `src/cap_harness/agent/mcp.py`：`observe` 返回状态/时间戳/相机元数据的 JSON，以及独立的 JPEG `ImageContent`；动作完成后附新的相机图像。`agent/session.py` 的执行摘要包含 `ok`、error、final_errors、diagnostics 和终态；完整程序反馈包含 stdout 与 calls。该 checkout 的 System-2 MCP 入口还提供具名程序目录，不等于每一步都由语言模型重新生成程序；我们借用的是 API/执行器与图像、反馈的分层方式。

两层职责明确：模型看图、选择策略和工具；SAM3/点云/抓取模型执行 agent 显式请求的计算。工具计算不意味着宿主替 agent 选择目标或动作。当前站点 planner 明确声明不需要观测场景，避免 ASPIRE 默认规划组合隐式采集 RGB-D；执行校验委托唯一 Runtime，避免公共层叠加起点/夹爪偏差门槛。

## 数据与接口

`get_observation` 冻结校正后的 RGB-D、内参、相机在 base 下的位姿与机器人状态。后续 `segment_text`、`mask_to_point_cloud`、`estimate_geometry`、`localize_object` 和 `generate_grasps` 共享该观测。接口保留接受 raw mask 和旧结果引用的能力；调用方负责掩码与图像、深度、内外参对应。选择掩码后到投影完成前，不应刷新观测或移动相机／物体。

CaP 每轮收到真实 RGB 图像作为多模态输入；程序可取得同帧 RGB-D 数组、内参和位姿。是否分割、定位、生成抓取及如何规划由 agent 决定。SAM3 接收完整图像和短文本描述，或同图的一个正点提示。文本候选在服务和客户端均经过非零阈值拒绝，再排序；点提示分数具有不同语义。`localize_object` 默认选择通过文本阈值的最高分候选，也接受调用方已选择的 `segmentation`，此时不再文本分割。成功只表示所选掩码的几何计算成功，不证明类别、唯一性或实物定位精度。深度投影仅排除非有限或非正深度，不隐式取最大簇。详见 [SAM3 输入、拒绝与掩码复用](sam3-input-and-confidence.md)。

Contact-GraspNet 在真实相机光学坐标系推理，结果转换回 base。服务返回网络预测开度；缺失开度会失败，不再固定写入 80 mm。`config/perception.json` 的变换依据网络 `build_6d_grasp` 和 FR3 Hand URDF，将模型 X 夹持轴转换为 Hand TCP 的 Y 夹持轴，并处理 103.4 mm 掌心到 TCP 偏移。返回的 `GraspCandidate.pose` 已是 TCP 目标，不应再次应用该变换。此变换仍需物理标定验收。

2026-11-19 已取消动作入口的自动场景建模、模型碰撞拦截、目标登记与持物一致性门槛。`providers/scene.py` 和 `preview.py` 仅保留作离线调试，不参与当前面板/CaP 派发。感知结果不再改变运动许可或计划绑定。

FR3 的 IK 生成路径，所选 `panda_py` 或 `franky` 生成原生时间曲线；规划不需要相机/感知服务，不执行模型碰撞检查。预览从该对象采样，执行入口保留同一个已登记对象，不重新插值。默认禁止运动和自动恢复；是否启用动作取决于当前会话。原 native 控制 helper 不作为失败后的替代控制器。

## 环境与来源

主环境 `.venv` 安装项目的 `cap` extras。模型独立安装在 `.franka-player/venv-perception`，使用 Python 3.12、torch 2.9.1+cu128 和 torchvision 0.24.1+cu128。仓库、commit、权重 SHA-256 见 `config/perception-lock.json`。实际权重路径保存在 Git 忽略的 `.franka-player/perception/weights.json`，不包含 token。

Franka 站使用 RATs 已有的 `rats-cache/models/sam3-modelscope/sam3.pt`。双臂 ARX 站的 SAM3 权重来自 [ModelScope 的 facebook/sam3](https://modelscope.cn/models/facebook/sam3)，版本为 `f8ee5d16d5c30d0e87b1d22cbd358ce97761df56`。两站采用同一个权重哈希。ARX 站的 SAM3 专用环境为 `.franka-player/venv-sam3`，使用 Python 3.10.19 和 PyTorch 2.7.0+cu126。三视角验证步骤见 [ARX 标定与 CaP 定位](arx-calibration.md#用-sam3-对比三个视角)。

Franka 的 Contact-GraspNet 使用固定源码版本附带的正式检查点，严格加载全部模型参数并进入 eval/inference 模式。缺权重、哈希不符、参数不匹配均停止启动。

重建时先按 lock 文件 clone / checkout `vendor/sam3` 与 `vendor/contact_graspnet_pytorch`，保留上游许可证，然后运行：

```bash
python3.12 -m venv .franka-player/venv-perception
.franka-player/venv-perception/bin/pip install torch==2.9.1 torchvision==0.24.1 --index-url https://download.pytorch.org/whl/cu128
.franka-player/venv-perception/bin/pip install -r config/perception-requirements.txt
.franka-player/venv-perception/bin/pip install -e vendor/sam3
```

`panda_py` 使用 lock 中的 panda-py commit，在本机 libfranka 0.19.0 上编译。先应用 `patches/panda_py_deployment.patch`，再使用 `.venv/bin/pip install ./vendor/panda-py`。需要系统 libfranka/Eigen 开发文件和 C++ 编译器。补丁只暴露已有 C++ JointTrajectory 控制器的同对象执行、关闭隐式恢复，以及缺少 Pinocchio 开发头时关闭可选 RobotModel binding。它不是新的运动控制算法；尚未进行物理执行验收。

分别启动服务，保持各自前台进程或交给本机服务管理器：

```bash
.venv/bin/python scripts/perception-service.py sam3
.venv/bin/python scripts/perception-service.py contact_graspnet
```

启动器核验权重和源码版本后才加载服务。`http://127.0.0.1:8114/healthz` 与 `http://127.0.0.1:8115/healthz` 只有模型加载完成才报告 `weights_loaded: true`。监听端口或返回 OpenAPI 文档本身不代表就绪。

## 自主只读验证

```bash
.venv/bin/python -m robol2l_deploy.cap.cli mock \
  --codex-model gpt-6-astra \
  --codex-bin .franka-player/tools/codex/node_modules/.bin/codex --validate

.venv/bin/python -m robol2l_deploy.cap.cli visual \
  --codex-model gpt-6-astra \
  --codex-bin .franka-player/tools/codex/node_modules/.bin/codex \
  --observation /absolute/path/to/observation-ID
```

若已有唯一硬件工作台占用相机/FCI，可通过它取得一份新观测，然后离线推理、规划：

```bash
.venv/bin/python -m robol2l_deploy.cap.cli visual \
  --codex-model gpt-6-astra \
  --codex-bin .franka-player/tools/codex/node_modules/.bin/codex \
  --live-url http://127.0.0.1:8765 \
  --station-directory /absolute/path/to/existing/hardware
```

这是一次当前物理采集之后的冻结回放，不是持续更新的物理控制会话。它只向现有 owner 提交 `get_observation`，不会创建第二个机器人或相机连接。新的记录包含原始机器人状态及 device binding；旧记录必须与归档 raw RGB-D 逐像素匹配，并核对原始标定哈希、内参、相机变换及机器人时间戳/关节值；缺失或不匹配则拒绝，不补造历史版本或 commanded state。

更新后的工作台另有“自动感知与只读规划”按钮，只允许真实模型和物理观测。Codex 模式由服务器环境变量 `ROBOL2L_CODEX_MODEL` / `ROBOL2L_CODEX_BIN` 配置；API 模式沿用现有非敏感配置和密钥环境变量。旧工作台进程不会热替换其设备 owner。

## 证据与范围

`session/v*/source.py` 保存模型原始程序，`result.json` 保存 stdout、源码位置、参数和实际错误；`runtime/perception/*` 保存分割覆盖图、mask、点云与抓取候选；`events.jsonl` 保存场景和调用；`plan-*/plan.json` 保存实际时间采样与版本绑定。

自主验收不使用人工 ROI、颜色脚本、预制抓放源码、历史坐标或隐藏 mock 评分。评分只在测试端，模型只看到公开执行反馈。任何失败都不回退到这些调试工具。公开 `result` 和程序成功不等同于任务成功。

视觉准备不再要求固定的定位/抓取/场景工具链。模型根据图像和工具结果报告准备情况，记录为模型判断；程序成功、模型报告完成、独立物理验证分别表示，不将模型判断自动标记为物理成功。

ASPIRE 原执行器在 NumPy 2 的数组归约方法上存在受限 builtins 兼容问题。已加入小范围补丁：归约仅使用预加载的 `numpy._core._methods`，程序源码仍只允许导入 math，文件/网络/进程限制不变。修改和哈希记录在共享补丁清单中。
