# 实验 ZIP v1

从 `real_expriment/green-cube-to-plate.zip` 开始。面板“加载实验”选择 ZIP → “上传并加载 ZIP”；随后才可手动开始。此目录名按用户指定保留 `real_expriment`。

ZIP 根目录必需 `manifest.json` 和 `experiment.json`，不要再套一层目录。

```json
{
  "schema": "franka-experiment-bundle/v1",
  "experiment": "experiment.json",
  "files": {"experiment.json": "其 SHA-256", "TASK.md": "其 SHA-256", "guidance/AC.md": "其 SHA-256", "guidance/C.md": "其 SHA-256"},
  "context_files": ["TASK.md"],
  "guidance_files": [{"kind":"AC","path":"guidance/AC.md"},{"kind":"C","path":"guidance/C.md"}]
}
```

`files` 必须列出除 manifest 本身以外全部文件，值为文件字节 SHA-256。`context_files` 指定给模型的 UTF-8 任务参考文本（总计不超过 150 KB）；其他附件只归档。空 AC/C 不导入为指导条目；非空指导冻结原文与校验值。照片、深度、标定快照与示例代码可以打包，但上传不执行代码、不安装参数；历史图像不是实时证据。

experiment 字段：name、goal、success_criteria（物理目标）、validation_criteria（本轮验收）、mode=live、pipeline=cap、execution_policy=read_perceive_plan、guidance=[]。演示可用 mode=demo、pipeline=demo。模型配置与密钥不从 ZIP 应用。

上限：20 MiB 压缩、50 MiB 解压、128 文件、单文件 10 MiB。拒绝重复路径、路径穿越、符号链接、加密 ZIP 和校验不符。文件只保存在原始 ZIP 中，不直接解压到仓库。原包及每个附件摘要保存到实验记录；因此后续可复查完整输入。

源码生成器 `scripts/build-green-bundle.py` 可重建当前任务包；它仅打包参考资料，不运行实验。
