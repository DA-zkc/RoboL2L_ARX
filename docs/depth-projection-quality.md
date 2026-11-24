# RGB-D 投影质量与显式过滤（第 3 项）

**后续部署更新（2026-11-24）：** 用户确认摆位并授权维护重启后，新投影/定位 API 已在 Runtime `1f27ffc087fc4f8993f46995259a6b77` 在线核对。重复去畸变修复和原始同帧采集也已加载；剩余米级值在软件对齐前的原生深度中已存在，详见 `rgbd-depth-source.md` 最新进展。下文未部署描述为早期实施记录。

2026-11-24：实现与离线验证完成，未加载在线 Runtime，未操作真机。此项补齐投影层的数据质量证据与可选处理能力；不等于已修复相机原始异常深度的生成来源。

## 接口与职责

`mask_to_point_cloud(mask, camera_name, target_frame=..., *, depth_range_m=None, seed_px=None, max_depth_jump_m=None)` 和 `localize_object(..., depth_range_m=None, seed_px=None, max_depth_jump_m=None)` 支持相同的可选参数。工具描述已同步给 Agent。

- `depth_range_m=[min,max]`：闭区间，单位米，始终是相机光轴 Z 深度，不是输出坐标系 Z，也不是相机到点的欧氏距离。
- `seed_px=[x,y]` 与 `max_depth_jump_m` 必须一起传入。仅保留包含指定整数像素的四邻域连通区域；相邻像素深度差不超过给定值。搜索范围严格限制在输入掩码、有限正深度及指定深度区间内。
- 不默认选最大簇、不向掩码外扩展、不侵蚀边缘、不重新分割。种子在掩码外、无深度或超出指定范围时明确失败，不吸附到最近像素，不改用其他候选。
- 连续性只是相邻点深度差约束，可以沿缓坡走到较远深度；需要限制总深度跨度时应显式给出深度区间。它也不能区分相互接触、深度相近的物体与桌面。
- 不传过滤参数时保持旧投影结果：按像素顺序投影掩码内所有有限正深度。`estimate_geometry()` 保持原有 PCA/min-max 算法，使用全部输入点。

选择掩码、参数和后续动作仍由调用方决定。没有改 Agent 提示语言、重试流程、抓取或运动许可。

## 质量证据

`cloud.diagnostics['projection']` 和 `localized.diagnostics['projection']` 包含：

- 掩码像素数、有效正深度数、无效数；输入与保留深度的 min/p01/p05/median/p95/p99/max。
- 实际参数、深度范围排除数量、连通性排除数量、保留数量；`retained_fraction` 的分母为输入掩码总像素数，包含无效深度像素。
- 来源相机坐标系、时间戳和掩码携带的 observation_id（可能为空或为调用方保留的旧 ID，并非新做的对应关系认证）。
- `target_identity_verified=false`、`depth_accuracy_verified=false`：分布和过滤成功不认证目标类别或传感器精度。

空结果直接投影抛 `ValueError` 子类 `DepthSelectionError`，携带 `diagnostics`；组合定位返回可恢复 `POINT_CLOUD_FAILED`，保留诊断，不返回几何成功。剩余点不足以估计几何时同样保留投影诊断。

刚体变换保留相机 Z 的来源统计。裁剪额外记录 `crop` 输入/输出数量；`projection` 始终描述最初投影，不能当成后续裁剪云的新统计。现有颜色、坐标和输入掩码保持不变。

例如，调用方已从冻结图像确定掩码、目标种子及适用深度区间后：

```python
localized = localize_object(query, camera_name=name, segmentation=selected,
    depth_range_m=depth_bounds, seed_px=target_pixel,
    max_depth_jump_m=adjacent_depth_limit)
result = localized
```

本例不规定任何物体的通用阈值，也不包含运动。

## 存档验证与剩余边界

`tools/check_depth_archive.py` 只读取历史 RGB-D、掩码，另对副本做明确标记的故障注入，不调用 SAM3 服务或硬件。使用同一份深度单位、内外参，不再次去畸变。

| 场景 | 最大包围盒边长／结果 |
| --- | --- |
| 原错误桌面掩码，默认投影 | 5.97360 m，与历史结果一致 |
| 同掩码显式限制相机 Z ≤ 1 m | 1.27801 m，仍是错误目标 |
| 同掩码以柿子中心 `[325,200]` 作种子 | 拒绝，种子不属于该掩码；不自动换目标 |
| 正确点提示柿子掩码，原始深度 | 0.0903029 m |
| 正确掩码显式使用 0.3–0.5 m | 点集完全不变，0.0903029 m |
| 正确掩码同时使用种子和 0.03 m 邻接差 | 0.0844985 m；部分边缘被排除，不视为更准确的实测尺寸 |
| 正确掩码内人为注入一个 4.5 m 深度点 | 4.16994 m（合成故障） |
| 合成故障显式使用 0.3–0.5 m | 仅删除注入点，0.0903005 m |

所有输出均验证没有掩码外点。原错误候选的主要问题仍是目标归属，深度过滤不能取代第 1、2 项。新接口不默认拒绝大尺寸，不认证柿子类别；调用方可以不使用过滤。相机端异常可能产生在传感器、RGB-D 对齐或重映射阶段，现有存档仅有处理后深度，不能据此完成源头归因。这部分仍需独立采集原始与处理后同帧证据。

验证命令：

```bash
PYTHONPATH=. .venv/bin/python -m unittest discover -s tests -p test_depth_projection.py
PYTHONPATH=. .venv/bin/python tools/check_depth_archive.py \
  --output .franka-player/diagnostics/20261124-depth-fixes/archive-regression.json
```

另运行 SAM3 confidence 和 ASPIRE API parity 回归。后者用 `ASPIRE_CHECKOUT` 指向锁定源码。上游补丁已重新生成并从锁定原始文件应用验证，22 个来源/修改后哈希匹配。额外的 `test_arx_rgbd.py`：1 项通过，1 项因环境缺少 `bimanual` 在初始化阶段报错；没有将其计入通过项。未改动该硬件适配器或相机采集代码。

本次不重启、不迁移、不加载在线硬件 Runtime；部署仍需遵守常驻 Runtime 的迁移边界。反馈摘要第 4 项不在本次范围。
