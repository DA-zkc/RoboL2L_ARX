# ARX 双臂夹爪抓取坐标

2026-11-24：按用户确认的 ARX 官方单轨夹爪，已为本站两侧配置官方 X5-2023 模型推导的抓取中心。代码和配置已完成，未重载在线 Runtime、未执行真机动作。模型几何已核对，实物 TCP 验收尚未进行；已确认的夹爪开闭角度保持不变。

## 官方来源与推导

采用 [ARXroboticsX/ARX_Model 的 X5/X5-2023](https://github.com/ARXroboticsX/ARX_Model/tree/a45e8dbb4b1844ae7aef62d16182e5f1724da6d3/X5/X5-2023)，锁定提交 `a45e8dbb4b1844ae7aef62d16182e5f1724da6d3`。未采用 X5-2023-newbase、OpenX5 或 X5-2025。原始 URDF、两指 STL 和 package.xml 保存在 `robol2l_deploy/models/arx-x5-2023-gripper/`，加载时核对 SHA-256；运行时不访问网络。

URDF 中 joint7/joint8 均直接接在 link6 上，为沿 ±Y 的相反直线运动，每侧范围 0～44 mm。两指安装原点约为 `[86.57, ±24.90, -0.244] mm`，这不是指尖接触点。网格中朝内的最远端平面是两个梯形接触面；`arx_gripper_model.official_gripper_record()` 读取这些面的三角形，计算面积加权中心，再取两侧中心的中点，定义本适配器的虚拟 TCP。厂家 URDF 本身没有命名这个 TCP。

推导出的 `T_link6_from_grasp_tcp` 为：

```text
[ 0  0  1   0.149335045004 ]
[ 1  0  0  -0.000002000000 ]
[ 0  1  0  -0.000713876255 ]
[ 0  0  0   1              ]
```

平移单位米。TCP 的 +Z 对应 link6 的 +X（夹指前伸方向），TCP 的 +X 对应 link6 的 +Y（两指开合轴）。最前端指尖在 link6 的 X≈157.57 mm，比 TCP 再向前约 8.235 mm；接触面沿 X 覆盖约 142.57～157.57 mm。TCP 是接触区域中心，不能把它直接贴到桌面后就认为指尖有净空。完整面边界、运动轴和行程均提供给 Agent 的 `capabilities.grasp_geometry.*.geometry.contact_geometry`。

对称开合时两个中心做相反的等量位移，因此中点不随开度变化。URDF 的夹指变量单位是米，控制 SDK 的夹爪变量是电机弧度；模型没有提供二者的传动函数。88 mm 是模型两侧行程之和，不能宣称已标定编码器端点下的实测张口为 88 mm，也不据此改写当前 0～1 到编码器的映射。

官方完整机械臂模型与本机 SDK 的 joint1～6 旋转和轴方向一致；joint3/4 的 Y 原点各差 ±0.03 mm，joint5 的 Y 原点差 0.5 mm，joint6 定义一致。本次仅使用 link6 后的夹爪尺寸，不替换 SDK 的机械臂运动学，也不重算已标定的相机外参。这些 CAD 版本差异不代表实测精度。

## 三个不同的标定

- 开闭端点：归一化开度 0/1 对应各侧编码器角度，记录在 commissioning 文件中。本站已完成。
- 相机外参：腕相机相对 SDK `link6` 的位姿，记录在 extrinsics 文件中。
- 抓取几何：SDK `link6` 相对夹指接触中心的变换，记录在 `config/arx-x5-2023-dual-grasp-geometry.json` 中。本站已接入官方模型，实物验收待完成。

`gripper_calibrated=true` 只指第一项。Agent 和操作员应分别查看 `capabilities.grasp_geometry`、`grasp_geometry_ready` 和 `end_effector_references`。

## 配置约定

站点配置的 `grasp_geometry` 指向上述文件。顶层 `schema=1`、`station=arx-x5-2023-dual`、`ports` 必须与设备配置一致，`sides` 必须包含 `left`、`right`。每侧为 `null` 时保留旧的法兰位姿语义，并明确报告缺少几何。非空对象必须具备以下字段：

| 字段 | 含义 |
| --- | --- |
| `T_link6_from_grasp_tcp` | 4×4 刚体变换，把抓取坐标中的点变换到 SDK `link6`。平移单位为米，右下角为 1，最后一行为 `[0,0,0,1]`。旋转的各列是抓取坐标轴在 `link6` 中的方向。 |
| `source` | 非空的尺寸／实测依据，注明对应的夹指与安装方式，以及验证记录位置。 |
| `verified` | 物理几何验收标志。本站为 `false`，官方模型来源不能冒充实物验收。 |
| `verification` / `model_verified` | 本站为 `official_model` / `true`，要求锁定源文件校验通过且全部配置数值与模型重算结果一致。其他实测配置仍须 `verified=true`。 |
| `opening_invariant` | 当前仅支持固定抓取坐标，须已验证该参考点与坐标轴不随开度变化，才能填 `true`。 |

抓取坐标原点定义在两指预定接触区域的中心，+Z 为朝向物体的接近方向，+X 为夹指闭合轴，+Y 补成右手系。长度和方向都须相对 SDK 的 `link6` 原点确定，不能仅测外壳边缘到指尖的长度。本站两侧使用用户确认的相同官方夹爪，故采用相同模型局部变换，右基座变换仍单独处理。

若实际机构的接触中心随开度变化，本次固定变换实现不适用，应先建立随开度变化的几何模型。开度 0～1 仍表示编码器角度的归一化值，不表示米制夹持宽度；本次不推导角度到两指间距的映射。

无来源的参数用 `null`，不要填单位阵或失败实验估算的 138.7 mm。矩阵非刚体、非有限值、设备端口不符、既非实测验证也非锁定模型推导、或偏移大于 0.5 m 时拒绝加载。`grasp_geometry_ready` 表示坐标转换可用，`physically_verified` 单独表示实物验收情况。

## 运行语义

配置了有效几何的那一侧，以下接口统一使用 `grasp_tcp`：

- `get_robot_state().end_effector_poses`，以及 observation 中的机器人位姿。
- `solve_ik`、`plan_motion`、`move_to_pose` 和同步运动中的位姿目标。
- 执行回执 `measured_pose` 和位姿到位检查。

转换关系为：

```text
T_base_from_tcp = T_base_from_link6 @ T_link6_from_tcp
T_base_from_link6_target = T_base_from_tcp_target @ inverse(T_link6_from_tcp)
```

右臂仍按原约定接受 `right_base`，有双臂外参时也接受 `left_base`。偏移按末端姿态旋转，不能直接在基座 Z 坐标上加减一个长度。Agent 不应再手工补一次指尖偏移。

回执同时保存 `measured_flange_pose` 和 `pose_reference`；规划诊断也记录每侧的 `end_effector_references`。`grasp_tcp` 是站点的虚拟工具坐标，不是厂商 URDF 里的额外连杆。SDK 关节控制、Home 关节目标、夹爪编码器端点保持原义。相机手眼标定与曝光时 FK 始终使用原始法兰，避免工具偏移被重复计入相机外参。

几何按会话启动时加载，不热更新。有效几何的内容纳入设备绑定；改变它后不能沿用旧实验的位姿快照继续复位。完成配置后应在空闲时开启新会话和新实验。未知几何时现有法兰规划仍可用，但不具备已标定的抓取中心语义。

## 验证范围

软件测试覆盖两侧不等长偏移、非单位旋转、右基座变换、FK/IK 往返、单侧和同步运动、到位回执、相机外参不变、设备绑定变化，以及未配置时保持旧语义：

```bash
PYTHONPATH=.:../ARX5_beta .venv/bin/python -m unittest discover -s tests -p 'test_arx_grasp_geometry.py' -v
```

测试还覆盖锁定模型的接触面推导、全开度范围中点不变、源文件／配置篡改拒绝、实际 SDK 的离线 FK/IK 往返，以及已确认编码器端点不变。所有检查均为软件测试，实际 SDK 测试显式提供关节种子且确认驱动未打开。

仍须在多个姿态下用独立实物参照验收抓取中心与接近方向；位置回执由关节 FK 计算，不是指尖绝对位置的外部测量。抓取是否成功仍须持物证据。TCP 几何改动不改变 SAM3，也未因此增加完整的夹爪场景避碰能力。

## 闭合时被物体阻挡的回执（2026-11-24）

11 月 23 日实验 `a4219f4194f8487bbb1ca65cc808d00d/v05` 中，左爪目标 0.02 rad，反馈 2.554551 rad，软件按空载到位条件报错；后续图像和 v07 抬升记录表明它实际夹住并保持了方块。修复针对适配器的完成条件，不改变已工作的 SDK 位置目标、夹持保持方式、控制增益或开闭标定。

双臂后端中的 `close_gripper()`、`set_gripper(0)`、`set_grippers()` 的零值，以及轨迹中的零值夹爪命令，均通过 `move_hands()` 使用同一规则：

- 目标仍是该侧已标定闭位；抵达目标的误差和速度条件保持原样。
- 未到闭位时，等待 2 s 指令时长结束，再检查至少 0.5 s 的稳定窗口：位置范围不超过 0.02 rad，速度绝对值不超过 0.1 rad/s，相邻采样间隔不超过 0.15 s，且反馈在有效开闭范围内。短暂停顿、持续运动、振荡、非法读数及闭合时反向张开不能作为这种完成结果。
- 稳定停在中途返回 `ok=true`、`completion=settled_short_of_closed`、`target_reached=false`、`grasp_verified=false`、`contact_detected=null`、`requires_grasp_verification=true`。这是有明确反馈的闭合动作完成，原因仍可能是物体、其他阻挡或卡滞；不能把它当作持物成功。无运动的重复闭合也不产生新的持物证据，回执记录 `initial_rad` 和 `closing_travel_rad`。
- 不追加夹紧／释放指令，不进入保护闭锁，保留现有 SDK 持续位置目标。Agent 收到回执后可以读取场景、确认持物，再决定后续动作；交接释放仍须接收侧独立持物证据。
- 非零 `set_gripper` 和张开仍要求目标到位；手动标定仍采用原严格条件。SDK 故障、拒绝、反馈异常和超时仍返回失败并停止派发。没有调整电流阈值，也没有把 `contact_grasp` 改成 `true`。

`ok` 表示此命令的执行结果，不是任务成功或抓住物体。达到空载闭位也不会设置 `grasp_verified=true`。另一次持物转移的 J5 超差是独立问题，本次未放宽机械臂 1° 到位阈值。

`tests/test_arx_gripper_grasp.py` 用模拟 SDK 重现上述位置条件，并覆盖接口别名、双爪不同结果、重复闭合、瞬时停顿、振荡、反馈异常、故障、暂停和隔离执行器继续读取状态。测试不连接硬件，不构成真机抓取验收。代码需服务重启后加载，当前会话不热更新。
