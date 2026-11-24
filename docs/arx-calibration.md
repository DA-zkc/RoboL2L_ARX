# 双臂 X5 桌面 ChArUco 标定

从双臂工作台打开“手眼标定”。标定板平放并固定在桌面上，全局 335 和当前腕相机应能同时看到同一块板。左右腕相机需要刚性固定到对应的末端。记录实测格子和码的边长（毫米），横纵均填格数，字典、行列方向和旧版布局以制板文件为准。板的尺寸误差会直接影响尺度。不要在同一批次中移动板、底座或相机安装座。

## 本站标定板 CC300-20-15

2026-11-22 按现场图像核对为横向 14 格、纵向 9 格，标准布局。OpenCV 的 `DICT_5X5_100` 可解码已观测标签（包括 ID 62）；该字典足以覆盖本板。格子边长和标签外黑框边长按型号暂记为 20 mm 和 15 mm，现场应以实测或随板图纸复核。表单预填上述值，可在创建批次前修改。

## 摆位与采样

1. 填写板参数并新建批次，选择左臂。换侧后要重新确认支撑。
2. 托住所选臂，由同伴按住示教按钮，或把脚踏键设为空格、聚焦示教按钮后踩住。显示“重力补偿中”后再拖动。腕部载荷影响补偿，先在有支撑的位置检查。
3. 停稳后先释放使能键，手继续托住。控制器锁定释放时读到的六个关节目标。SDK 接受目标、连续 0.5 秒实测误差在 0.01 rad 内且速度在 0.025 rad/s 内后，页面显示本次锁位成功。保持托护并轻轻卸力，确认实际承重。按住示教路径的失焦或按键心跳中断会请求锁位，已取消 30 秒上限。日常单人摆位请用控制面板或实机实验页的“10 秒后手动摆位”，不需要创建标定批次；该路径不依赖浏览器心跳，完成后点击“锁定当前位置”。断电、进程退出、CAN 故障时软件无法保证承重；先支撑好再断开工作台。
4. 点“记录当前姿态”。每次保存校正后的两张 RGB 图、相机序列号、内参、角点、曝光时间对应的设备系统时间、关节采样前后时间和角度，以及 FK 得到的末端位姿。若出现时间不匹配、手臂移动、角点不足或重投影误差过大，重新采样。每侧至少 12 个不同的姿态，建议 16–20 个，改变位置并绕两个以上方向倾斜。按同样操作采右臂。
5. 点“求解并验算”，展开矩阵或下载 JSON。同一 Runtime 数据目录下可刷新页面、重载后继续当前批次。新建批次会保留先前的文件。

## 结果

外参矩阵使用列向量，`T_A_from_B` 把 B 中的点变换至 A，平移单位为米。双臂公共参考系取左基座。输出包括右基座到左基座、全局 RGB 光学系到左基座、左右腕 RGB 光学系到各自 FK 末端，以及标定板在两个基座中的位姿。腕相机在公共坐标系中的位姿随关节变化，使用时应乘对应的实时 FK。

每侧每第四个样本留作验算。报告显示留出样本的板原点平移和朝向差异（RMS、最大值）、各相机单帧角点重投影 RMS 的最大值，以及腕相机与同次全局观测之间的偏差。全局重复性可帮助发现板或支架位移。请用独立参照物检验是否达到任务所需的绝对精度。求解结果仅导出，不自动改写生产控制或碰撞配置。

原始资料位于当前 Runtime 的 `hardware/arx-calibration/<批次 ID>/`。每个样本对应两个 PNG 和一个 JSON；`extrinsics.json` 保存最后一次求解的输入样本 ID、设备绑定、矩阵和指标。

## 在 CaP 中使用本站外参

夹爪抓取中心与相机外参是不同配置，见[抓取坐标说明](arx-grasp-geometry.md)。启用有效的抓取几何后，公开机器人末端位姿使用抓取中心；本站采用官方模型尺寸，实物 TCP 验收另行记录。本页手眼标定及腕相机外参仍以原始 `link6` 法兰为参考。

双臂工作站配置的 `extrinsics` 字段指向本工作站实测的 RGB 相机外参报告。当前使用批次 `635933667e2549cf8b1e7aed0f81bf97`。调用 `get_observation()` 后，三路相机的 `camera_pose` 和双臂末端位姿都以 `left_base` 表示。相机各自的 `frame` 仍标识彩色光学坐标系。腕部投影使用曝光前后的实测关节角插值计算 FK，采样时应先让对应手臂停稳。

对同一份冻结观测，从指定视角调用 `mask_to_point_cloud(mask, camera_name="right_wrist", target_frame="left_base")`。布尔遮罩的尺寸应与该相机 RGB 图像一致。使用文本目标时，`localize_object(query, camera_name="global")` 通过配置的分割服务生成遮罩，再用实测深度定位。检查其 `ok` 后读取 `geometry.pose`。三路结果可以直接比较，返回的位置单位为米。

在项目根目录运行以下命令可复验标定留出样本和同一工作站的归档 RGB-D。它读取本地 FK 模型和文件，可在机械臂断电时运行。

```bash
PYTHONPATH=.:../ARX5_beta .venv/bin/python tools/verify_arx_localization.py .franka-player/panel/sessions/d2146e67e87e40bfb3c7c4688d8544a2/hardware/arx-calibration/635933667e2549cf8b1e7aed0f81bf97
```

## 用 SAM3 对比三个视角

在双臂控制台选择 ARX 后，启动器会检查本地 SAM3 服务。可查看 [SAM3 健康状态](http://127.0.0.1:8114/healthz)。模型进程从本地文件加载权重。启动时对照 `config/perception-lock.json` 校验整个文件的 SHA-256 和模型源码的 Git 版本。本机使用独立的 `.franka-player/venv-sam3` 环境，权重路径和下载版本记录在 `.franka-player/perception/weights.json`。

在同一段 CaP 程序内分别调用 `localize_object("black and white checkerboard calibration board", camera_name=name)`，其中 `name` 取 `global`、`left_wrist` 或 `right_wrist`。成功时，`geometry.pose.position` 是选中遮罩内有效深度点云的三维包围盒中心，以米表示，坐标系见 `geometry.pose.frame`。相机只看到目标的一部分时，各视角的包围盒会覆盖目标的不同区域。

要在同一组归档 RGB-D 上重跑比较，在仓库根目录执行以下命令。第二个路径指定一个尚未创建的结果目录。`--check-board` 会额外比较三个遮罩内同编号 ChArUco 角点，适用于本站标定板。

```bash
PYTHONPATH=.:../ARX5_beta .venv/bin/python tools/verify_arx_sam3_localization.py \
  .franka-player/panel/sessions/d2146e67e87e40bfb3c7c4688d8544a2/hardware/observation-8f788272733d44a0965986c7fb0e2c4f \
  .franka-player/sam3-replay-01 --check-board
```

`comparison.json` 记录各视角坐标、两两差值、相机采集时间跨度，以及使用的标定批次。`selected-masks.jpg` 显示产生这些坐标的分割轮廓，`policy/` 保留策略源码与逐次调用回执。
