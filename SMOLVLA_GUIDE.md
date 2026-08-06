# SmolVLA 训练与部署基线

## 训练结果

正式输出：

```text
outputs/train/smolvla_purple_bag_b32_10k_v1
batch_size: 32
steps: 10,000
chunk_size: 50
```

| checkpoint | eval loss |
|---|---:|
| `002000` | 0.1631 |
| `004000` | **0.1572** |
| `006000` | 0.1616 |
| `008000` | 0.1810 |
| `010000` | 0.1920 |

部署和对照使用 `004000`。正式训练命令保存在该 checkpoint 的
`pretrained_model/train_config.json`；续训只能从 checkpoint 配置恢复，不能
覆盖现有输出目录。

按已解析配置复现实验时必须换一个新输出目录：

```bash
HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 \
lerobot-train \
  --config_path outputs/train/smolvla_purple_bag_b32_10k_v1/checkpoints/004000/pretrained_model/train_config.json \
  --output_dir outputs/train/smolvla_purple_bag_b32_10k_rerun
```

该配置记录了数据路径、batch 32、10K 步、seed 1000、45/6 episode 划分、
SmolVLA 基座以及全部 optimizer/scheduler 参数，是复现依据。

## 历史诊断

2026-08-04 的 `004000/010000 × sync/rtc × 3 姿态` 只读影子矩阵全部未达到
当时阈值：Sync 约 12～13 Hz 且有动作包络/跳变警告；RTC 因旧执行窗口配置未
产生有效动作。该结果属于早期部署诊断，不再代表当前主动控制能力。

2026-08-05 完成 v2 标定后，`004000` 使用 RTC、10 Hz 策略动作和 3 倍插值，
以约 30 Hz 连续发送 888 条指令并运行 30 秒，`fault=None`。它证明 SmolVLA、
RTC、插值、CAN、SDK 和反馈链能够主动工作；现场任务效果仍不佳，特别是抓取
起始、夹爪时序和 J6 跟踪滞后。

## 当前运行入口

SmolVLA 的分级主动脚本保留为：

```bash
bash scripts/run_piper_active_micro.sh --profile full --start-pose current
bash scripts/run_piper_active_micro.sh --profile diagnostic30 --start-pose current
```

`diagnostic30` 使用 RTC 和 3 倍插值；每轮结束后必须人工运行：

```bash
python scripts/recover_piper_emergency_stop.py
```

恢复后重新摆放、关闭或隔离 master，并再次完成当前姿态部署预检。

## 已知问题

- SmolVLA 单次推理较慢，必须使用异步 RTC 保持控制队列连续。
- 10% 速度下 J6 曾出现显著目标跟踪滞后。
- 模型早期动作可能直接进入抬升，夹爪未形成清晰的张开—闭合序列。
- 实机运行成功只说明控制链稳定，不能替代任务成功率统计。
