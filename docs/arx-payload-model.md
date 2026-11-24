# X5-2023 官方夹爪与 Gemini 305 负载模型

2026-11-24：修复双臂适配器始终使用 SDK 默认负载模型的问题。左右臂分别使用包含腕部相机本体的 URDF，规划 FK/IK 与 Runtime 内部 SDK 控制器加载同一路径。只改变 link6 的质量、重心和惯量，不改变关节几何、限位、控制增益、夹爪端点或抓取 TCP。

这是有来源的相机本体近似模型，尚未做实物负载辨识或抖动验收。2026-11-24 用户随后要求重载，已通过现有启动器加载到会话 `f54f973dc0574e3383c0c5be61b8bf39`，在线核查左右模型及双臂、三路 RGB-D 读取成功，运动授权关闭。软件测试和服务加载不等于实机抖动已消除。

## 来源与取值

- [ARX SDK 官方说明](https://github.com/ARXroboticsX/ARX5_beta/blob/main/readme.md) 第 4、5 节说明默认值已考虑夹爪，加装相机需要调整末端质量。保留当前 SDK link6 的原始质量 0.589997204780952 kg、原始重心与惯量，避免重复加入官方夹爪。
- 已核对 [ARX_Model 的 X5-2023 模型](https://github.com/ARXroboticsX/ARX_Model/tree/a45e8dbb4b1844ae7aef62d16182e5f1724da6d3/X5/X5-2023)。CAD 模型将 link6、link7、link8 分开，三者质量合计约 0.583836 kg，与 SDK 合并模型不完全相同；因此没有用 CAD 的全臂 URDF 替换 SDK，也没有把两种模型的夹爪质量叠加。
- [Orbbec 官方产品页](https://www.orbbec.com/gemini-305/)与 [Gemini 305 Datasheet V1.0](https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Orbbec_Gemini%20305%20Datasheet%20V1.0_20260105.pdf)：本体净重 **68 g**，尺寸 **42 × 42 × 23 mm**。规格书第 14、28、29、44 页分别用于核对质量/尺寸、RGB 光学坐标、深度零平面及机械图。
- [官方 CAD 下载页](https://doc.orbbec.com/documentation/Gemini%20305%20Documentation/Download%20CAD%20Files%20of%20Gemini%20305)提供外形图和 STEP；本次未找到可直接采用的厂家整机重心和惯量数据。不能把外形 CAD 的几何中心称为实测重心。

相机重心采用均匀外壳包围盒的几何中心近似：RGB 光学坐标中 `[9, 0, -7.55] mm`。X 来自 18 mm 双目基线的一半；Y 取居中近似；Z 为深度零平面距前面玻璃 3.95 mm 减半机身厚度 11.5 mm。惯量使用均匀长方体公式。**这两个量都是建模近似，不是厂家认证质量属性。**

由现有 `T_left_flange_from_left_wrist_optical`、`T_right_flange_from_right_wrist_optical` 将相机重心与惯量转到各自 link6，再按加权重心和平行轴定理合并。使用原始法兰坐标，不使用抓取 TCP；全局相机不装在臂上，不计入负载。

| 项目 | 左 | 右 |
| --- | --- | --- |
| 相机重心，link6，mm | `[62.764, -0.730, 108.532]` | `[60.365, -0.441, 109.918]` |
| 合并 link6 质量，kg | 0.657997204780952 | 0.657997204780952 |
| 合并重心，link6，mm | `[52.466, -0.059, 11.209]` | `[52.218, -0.030, 11.352]` |

## 配置和后续支架数据

入口是 `config/arx-x5-2023-dual.json` 的 `payload`，指向 `arx-x5-2023-dual-payload.json`。相机质量、外形尺寸、名义重心、左右安装变换、附加部件全部可配置，没有写死在驱动代码中。

用户表示支架“大概率是原装的”，但未确认型号。两侧 `bracket` 只是待确认状态记录，质量为 null；**不表示支架实际质量为零**。目前不计入支架、线缆力和抓持物体，`physically_verified=false`。

确认支架数据后，在对应侧的 `additional_components` 加入：

- `mass_kg`：该部件质量。
- `com_link6_m`：原始 link6 坐标中的重心，三维向量，单位米。
- `inertia_com_link6_kg_m2`：关于部件自身重心、轴方向与 link6 一致的 3×3 惯量矩阵。
- `source`：厂家文件或实测来源。

`bracket` 中的备注/质量字段不会自动参与动力学，实际质量属性必须放入 `additional_components`，避免重复计重。部件数据应采用完整质量属性，不能只编造一个惯量值来通过校验。

修改后离线运行：

```bash
PYTHONPATH=. .venv/bin/python tools/build_arx_payload.py
```

生成器更新左右 URDF 和 SHA-256，不触碰服务或硬件。外参改变后应核对新的安装变换并更新负载配置；它不会自动覆盖标定结果。Runtime 加载时检查 CAN、相机序列号、安装变换、基准 URDF、生成结果和哈希是否一致，不一致就拒绝加载。连接 SDK 前再次核对模型哈希。新负载进入设备绑定，旧运动计划不会沿用；相机几何标定绑定不因纯惯性参数变化而失效。

## 离线验证

`tests/test_arx_payload.py` 验证质量不重复计入、惯量旋转/平行轴合并、额外支架配置、过期模型拒绝加载，以及设备绑定与标定绑定的区别。

装有 SDK 时，还使用纯 Kinematics 在三组姿态、左右两模型上验证：FK 和关节限位不变；新增重力力矩等于相机势能对关节位置的数值导数（绝对容差 1e-7 Nm）。另在 SDK SingleArm 包装层拦截 ArmController 构造，确认 URDF 覆盖确实传入底层、仍使用原默认 200 Hz；该检查没有创建控制器，也没有连接 CAN。

```bash
PYTHONPATH=.:tests:../ARX5_beta .venv/bin/python -m unittest test_arx_payload
```

来源 PDF 存档位于 `.franka-player/diagnostics/20261124-payload-model/`。后续应通过现有 Runtime 在现场比较运动反馈，再判断相机负载模型是否是 1–10 Hz 抖动的主要原因。该现象也可能涉及跟踪增益、摩擦、轨迹或实时调度；当前没有证据把它们排除。
