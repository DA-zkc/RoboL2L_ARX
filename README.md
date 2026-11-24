# RoboL2L_ARX

ARX X5 (2023) 双臂真机部署栈：CaP 程序生成与执行、唯一 Runtime、ARX SDK 适配器、常驻硬件 owner、三路 RGB-D、SAM3 客户端与推理服务，以及浏览器控制台。

这是工作站源码的独立快照，包含截至 2026-11-24 的本地修改。源码发布、在线加载和真机验收是不同状态；本仓库不宣称所有功能已完成实物验收。已有实验最多执行 **8 版（v00–v07）**。实验者明确“复位并写下一版”的决定会独立传给 Agent，既有疑问与原始证据保留。

## 包含的内容

| 路径 | 内容 |
| --- | --- |
| `cap_harness/` | 固定上游版本的 CaP API、类型、执行器、SAM3 / Contact-GraspNet HTTP 客户端和服务 |
| `robol2l_deploy/cap/` | 模型调用、生成/复盘、隔离程序执行、八次预算、人工继续决定、反馈摘要和录像 |
| `robol2l_deploy/adapters/` | ARX 单臂/双臂适配、SDK 原生规划/查询、夹爪、负载/TCP、Orbbec RGB-D、Mock |
| `robol2l_deploy/arx_owner.py` | 独立于控制台生命周期的常驻硬件 owner |
| `robol2l_deploy/providers/` | 感知调用、观察绑定和本地推理服务启动 |
| `robol2l_deploy/web/`、`franka_player/` | 启动器、控制台、标定、手动摆位和实验页面 |
| `config/`、`robol2l_deploy/models/` | 工作站配置、已记录的夹爪端点/外参、模型来源及锁定版本 |
| `scripts/`、`tools/`、`tests/`、`docs/` | 启动/安装辅助、诊断工具、软件回归和技术说明 |

`franka_player`、`franka_cap` 和少量 Franka/native 兼容代码保留原包名，因为控制台与共用接口仍依赖这些模块；它们不代表 ARX 需要安装 Franka 驱动。仿真训练目录、原仓库 Git 历史、现场实验档案/录像、虚拟环境、密钥和模型权重不随本仓库发布。

## 安装

主控制环境使用 **Ubuntu 24.04 / Linux x86_64 / Python 3.12**，以匹配锁定 ARX SDK 的二进制 ABI。

```bash
git clone git@github.com:DA-zkc/RoboL2L_ARX.git
cd RoboL2L_ARX
python3.12 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e '.[cap,arx]'
```

厂商 SDK 不重复分发。依据 [`config/arx-sdk-lock.json`](config/arx-sdk-lock.json) 安装到同级 `../ARX5_beta`，或设置 `ARX5_SDK_DIR`：

```bash
git clone https://github.com/ARXroboticsX/ARX5_beta.git ../ARX5_beta
git -C ../ARX5_beta checkout c0f612fe7c9565b0e8c85871d9cd897334b15d7b
export ARX5_SDK_DIR="$(realpath ../ARX5_beta)"
export PYTHONPATH="$ARX5_SDK_DIR${PYTHONPATH:+:$PYTHONPATH}"
```

依赖厂商库的系统安装请按该版本 SDK 文档执行。不要为了运行本项目盲目执行厂商全局免密 sudo 脚本。

### 配置工作站

当前 `config/arx-x5-2023-dual*.json` 是原工作站的参考配置，包含 **left/can1、right/can3**、相机序列号、双臂外参、Home、夹爪端点及负载模型。它们不是任意两台机器通用的标定值；新工作站必须核对设备绑定和标定。不要直接把原工作站 Home 用于另一套装置。

- [标定与复验](docs/arx-calibration.md)
- [夹爪与 TCP 几何](docs/arx-grasp-geometry.md)
- [手动摆位](docs/arx-manual-positioning.md)
- [Runtime 生命周期](docs/arx-resident-runtime.md)
- [SDK 原生约束与项目边界](docs/arx-sdk-cap-only.md)

`scripts/start_x5_can.sh` 保留原工作站 USB-CAN 绑定，运行前核对脚本中的设备标识。`scripts/setup-arx-cameras.sh` 配置 Orbbec USB 权限，需要 sudo。上述脚本不作为机械臂动作接口。

### 感知服务

应用层服务和客户端已包含。SAM3 模型源码、GPU 环境和权重作为独立依赖安装，来源、commit、权重 SHA-256、端口见 [`config/perception-lock.json`](config/perception-lock.json)。完整步骤见 [感知部署说明](docs/automatic-perception.md#环境与来源)。

将模型源码安装到 lock 指定的 `vendor/` 子目录，权重清单放在不入库的 `.franka-player/perception/weights.json`，格式为：

```json
{
  "sam3": {
    "path": "/absolute/path/to/sam3.pt",
    "sha256": "与 config/perception-lock.json 中的 weights_sha256 一致"
  }
}
```

SAM3 环境可使用 `.franka-player/venv-sam3`；否则服务使用 `.franka-player/venv-perception`。该环境需能导入本仓库及对应的 vendor 模型包。

```bash
.venv/bin/python scripts/perception-service.py sam3
```

服务会先核验源码 commit 和权重哈希。`http://127.0.0.1:8114/healthz` 中 `weights_loaded=true` 才表示模型加载完成。ARX 启动器不自动启动 Contact-GraspNet；其代码保留，但默认抓取坐标转换包含 Franka 模型假设，不能直接当作 ARX 抓取标定。

模型 API 密钥只通过服务器环境变量提供，默认变量名为 `ROBOT_MODEL_API_KEY`；在实验页设置实际 provider/model 等非敏感配置。不提交 `.env`、访问令牌或本地模型设置。

## 启动

```bash
bash scripts/start-studio.sh
```

打开 `http://127.0.0.1:8770/panel`，选择 ARX 双臂。实验前布置机械臂、夹爪和物体；首次开始冻结初始快照。后续回位由实验者授权，通过同一个 Runtime 执行。

普通控制台重载保留常驻硬件 owner；硬件 owner 更新、关闭或更换模型属于另外的维护操作。不要运行第二个控制器接管正在使用的 CAN/相机，也不要把 SDK 参考轨迹结束当成物理到位验收。

## 软件检查

无需连接真机即可验证实验编排、公共 API 和感知处理。示例：

```bash
PYTHONPATH=.:tests .venv/bin/python -m unittest \
  test_operator_continuation test_experiment_pipeline \
  test_experiment_auto_reset test_experiment_model_recovery \
  test_sam3_confidence test_depth_projection
```

部分 ARX 原生语义测试需要 SDK：在 `PYTHONPATH` 追加 `$ARX5_SDK_DIR`。模型服务测试可能需要额外依赖；依赖现场私有档案或旧上游路径的历史测试不保证在新克隆中运行。软件 Mock 输出不是真机成功证据。

## 来源与发布说明

- 源码快照与原始文件哈希：[`config/source-snapshot.json`](config/source-snapshot.json)
- 外部 SDK 锁：[`config/arx-sdk-lock.json`](config/arx-sdk-lock.json)
- 许可与来源：[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md)
- 部分历史技术文档引用原工作站的 `.franka-player/` 档案和绝对路径；这些是来源索引，现场数据未公开。
