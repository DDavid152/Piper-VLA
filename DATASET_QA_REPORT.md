# 数据集质量报告

当前状态：手动数据集已有 3 条 episode。批量自动 QA 于 2026-07-30
完成，机器可读报告为 `logs/piper_manual_batch_qa.json`。

## 最新批量自动 QA

| episode | 数据行数 | front 区间 | wrist 区间 | 自动结论 |
|---|---:|---:|---:|---|
| 0 | 786 | 786 | 783 | 不通过：wrist 少 3 帧 |
| 1 | 561 | 559 | 561 | 不通过：front 少 2 帧 |
| 2 | 856 | 856 | 856 | 通过 |

episode 0、1 是流式编码器旧的“队列满后静默丢帧”行为产生的，不能直接
用于训练；修复没有修改原始数据。episode 2 仍需完成人工动作语义检查。

以后每次采集结束执行：

```bash
python scripts/verify_piper_dataset.py \
  --output logs/piper_manual_batch_qa.json
```

下方保留人工验收记录模板。

## 试采信息

```text
日期时间：
数据集 repo_id：
本地 root：
task：
episode 数量与时长：
操作人员：
```

## 自动检查

| 项目 | 期望 | 实际 | 结论 |
|---|---|---|---|
| 数据集 FPS | 30 | 待填写 | 待检查 |
| front 视频 | 可完整解码 | 待填写 | 待检查 |
| wrist 视频 | 可完整解码 | 待填写 | 待检查 |
| observation.state | 7 维、有限值 | 待填写 | 待检查 |
| action | 7 维、有限值 | 待填写 | 待检查 |
| timestamp | 单调、无明显间断 | 待填写 | 待检查 |
| task | 与示教完全一致 | 待填写 | 待检查 |
| 主机 CAN TX | 增量 0 | 待填写 | 待检查 |

## 人工检查

- [ ] 两路视角均清晰、方向正确、无冻结或无关人员。
- [ ] 关键操作位于 front 与 wrist 的重叠视野内。
- [ ] master action 随示教变化，follower observation 合理跟随。
- [ ] 失败示教已明确标记，不混入成功数据。
- [ ] `lerobot-dataset-viz` 可打开 episode。

## 结论

```text
通过 / 不通过：
需要重录的 episode：
问题与后续处理：
```
