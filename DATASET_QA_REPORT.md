# 数据集质量报告

## 正式训练数据集

截至 2026-08-06，SmolVLA 与 ACT 使用同一个 clean 数据集，保证模型对比不受
样本差异影响。

| 项目 | 当前值 |
|---|---|
| repo_id | `local/piper_purple_bag_two_handle_lift_manual_v1_clean` |
| 本地目录 | `datasets/piper_purple_bag_two_handle_lift_manual_v1_clean` |
| episode / 总帧数 | 51 / 42,066 |
| FPS | 30 |
| 图像 | `front`、`wrist`，RGB 640×480 |
| 向量 | 7 维 `observation.state`、7 维 `action` |
| 自动 QA | 51 通过 / 0 失败 |
| 报告 | `logs/piper_manual_v1_clean_batch_qa.json` |

原始目录 `datasets/piper_purple_bag_two_handle_lift_manual_v1` 有 52 条，包含一条
已排除的失败样本，只用于保留采集历史，不能直接训练或生成安全基线。

## 复验

```bash
source /home/ubuntu22/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
cd /home/ubuntu22/Piper-VLA

python scripts/verify_piper_dataset.py \
  --repo-id local/piper_purple_bag_two_handle_lift_manual_v1_clean \
  --root datasets/piper_purple_bag_two_handle_lift_manual_v1_clean \
  --output logs/piper_manual_v1_clean_batch_qa.json
```

自动 QA 覆盖 episode/帧/task 索引、共享 MP4 解码、视频和 Parquet 行数、
30 FPS 时序以及 7 维向量有限性。它不能判断动作语义；仍需人工确认每条示教
都夹住两根提带、提离约 10 厘米、悬停约 2 秒、放回、松开并退离。

## 新增数据准入

- 创建新版本，不原地修改 v1 clean。
- 两路画面清晰，关键操作同时处于可观察区域。
- task 文本完全一致，状态/action 单位与现有数据相同。
- 自动 QA 全通过，再逐条做人工语义验收。
- 排除失败 episode 后重新生成安全基线，再分别训练两个模型。
- 模型对比必须使用相同的数据版本、相机摆放和实机任务条件。

早期 3 条试采曾暴露视频编码队列丢帧，现行批量 QA 已覆盖该故障。历史报告
仅用于追溯，不代表正式数据质量。
