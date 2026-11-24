# SAM3 输入、拒绝与掩码复用

**后续部署更新（2026-11-24）：** SAM3 服务与 Runtime 已按用户授权重启加载。历史右腕图像实测：原中文查询返回可恢复 `NO_SEGMENTATION`，`orange fruit` 返回 0.9609375；服务阈值 0.5、tokenizer 计数和新 API 参数均已在线验证。Runtime `1f27ffc087fc4f8993f46995259a6b77`，证据在 `.franka-player/diagnostics/20261124-perception-load/sam3-live-validation.json`。下文未部署描述是此前离线实施阶段的记录。

2026-11-24 第 1、2 项修复：代码与离线验收已完成，尚未部署到在线 SAM3／Runtime。没有重载服务、停止硬件 owner、清故障或执行真机动作。RGB-D 上游深度调查及反馈摘要预算修复仍是后续独立任务。

这些是工具能力与输入输出契约，不是固定 Agent 流程。提示语言、是否选点、是否重试、调用其他工具或继续任务由 Agent 决定；没有加入自动翻译、自动选点、掩码替换、抓取／运动许可门禁。

## 文本候选与返回语义

`config/perception.json` 的 `sam3_text_confidence_threshold` 默认 **0.5**，恢复锁定厂商 `Sam3Processor` 的默认设置。启动脚本将此值传给服务 `--text-confidence-threshold`，客户端读取同一配置；直接创建客户端默认也为 0.5。只接受有限的 `0 < threshold < 1`。比较严格采用厂商实现的 `score > threshold`，等于阈值仍拒绝。阈值属于部署配置，Agent 调用没有降低门槛的参数。

服务继续使用原厂处理器过滤掩码。轻量模型代理仅截取同一次 `forward_grounding` 的过滤前分数（类别概率 × presence），不再次推理、不改变上游模型源码。客户端对服务返回再次过滤，取客户端和服务声明阈值的较大者，再排序取前五个；旧服务没有诊断字段时仍执行客户端阈值，并报告 `token_length_checked=false`。完整长度检查需新版服务生效。

无可信候选返回 `ok=False`、可恢复的 `NO_SEGMENTATION`，定位直接传递失败和诊断，不执行投影。保留 `raw_scores`、`returned_scores`、实际阈值、拒绝原因和输入。服务过滤前分数包括被拒绝候选，但不会保存其完整掩码。`label` 是查询文字的回显，不是类别确认。定位成功诊断明确为 `success_semantics=selected_mask_geometry`、`class_identity_verified=false`。

输入示例：`localize_object(original_query, camera_name=name, model_query=short_visual_phrase)`；`segment_text(name, original_query, model_query=short_visual_phrase)` 同样适用。不提供 `model_query` 时沿用原查询。记录同时保留 `original_query` 和实际 `model_query`，不静默翻译，不硬编码名称或选择第二候选。

模型查询应为简短可见属性／名词，不附任务步骤、空间推理或长句。服务使用**当前模型自己的 tokenizer 和 context_length**，在图像推理前计数；本机锁定模型上限为 32 token（含起止符）。过长返回 `INVALID_REQUEST`、`rejection_reason=prompt_too_long`、计数、上限和 `would_truncate=true`，不静默截断。短英文描述在本次存档上有效，但不意味着翻译必然可信，也不意味着模型普遍不支持中文。

## 点提示与复用

当前 `segment_points(camera_name, [[x,y]])` 只支持一个正点：x 是列，y 是行，单位为该相机图像像素；不支持负点或多点。点由 Agent 根据已有图像选择。候选得分表示点提示掩码质量，**不应用文本 0.5 阈值、不解释为目标类别置信度**；调用方仍需结合图像选择对应目标的掩码。

已选择的 `Segmentation` 可走两条等价的显式路径：

```python
# name、target_x、target_y 来自同一轮图像；该示例没有运动指令。
points = segment_points(name, [[target_x, target_y]])
if points.ok:
    selected = points.segmentations[selected_index]  # 调用方根据图像确认的候选
    cloud = mask_to_point_cloud(selected, name)
    geometry = estimate_geometry(cloud)
    # 或用组合接口；不要再独立文本查询覆盖这个掩码：
    localized = localize_object(original_query, camera_name=name, segmentation=selected)
    result = localized
else:
    result = points
```

传 `segmentation` 时组合接口不调用文本 provider；不允许同时传 `model_query`。保留 `selection_source=caller_selected_mask`、观测来源和原分数。组合接口与直接投影使用同一份冻结 RGB-D。接口不新增旧掩码自动失效机制，调用方承担跨快照对应责任；传入旧掩码并不自动证明它适用于新图像。

收到低分／超长反馈后，可缩短或修改可见描述，或从图像选点并复用成功掩码。提高阈值只实现正确拒绝，不保证自动找到物体；几何尺寸仍须结合场景判断。本修复不隐式删点、不取最大簇、不再次去畸变或换算深度单位。

## 离线验证与部署边界

新增客户端／API／隔离程序回归：`tests/test_sam3_confidence.py`。锁定 tokenizer 和原厂处理器的无权重 CPU 替身回归：`tests/test_sam3_service.py`，在 SAM3 环境运行。没有加载模型权重、提交在线推理或连接新控制器。

存档验收命令（只读取历史结果，输出写到新目录）：

```bash
PYTHONPATH=. .venv/bin/python tools/check_sam3_archive.py \
  --output .franka-player/diagnostics/20261124-sam3-fixes/archive-regression.json
PYTHONPATH=. .venv/bin/python -m unittest discover -s tests -p test_sam3_confidence.py
PYTHONPATH=.:vendor/sam3 .franka-player/venv-sam3/bin/python \
  -m unittest discover -s tests -p test_sam3_service.py
```

同一历史 RGB-D 的 11 组结果：7 组低分文本（含不存在的紫色大象）均拒绝；2 组可信文本通过；2 组点掩码直接复用成功，几何与原存档一致。对原最高分桌面候选的归因保持不变：主要是目标归属错误，异常深度放大错误区域。不能套用旧实验全图颜色阈值的结论。

部署与实物验收分开。在线 Runtime 仍为旧的进程内硬件控制结构时，不得调用旧 reload/stop；遵循 [常驻 Runtime 迁移说明](arx-resident-runtime.md)。本次不授权迁移、重启或真机验收。
