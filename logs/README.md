# 本机日志

除本 README 外，`logs/` 全部被 `.gitignore` 忽略。日志用于本机诊断和安全
追溯，不提交 Git；正式数据必须放在 `datasets/`，模型放在 `outputs/`。

| 目录/模式 | 内容 |
|---|---|
| `piper_active_micro/act_*.log` | ACT runtime、停止原因和 fault |
| `piper_active_micro/act_*.commands.jsonl` | ACT 每步目标、限速、反馈和发送结果 |
| `piper_active_micro/004000_*.log` | SmolVLA 主动 runtime |
| `piper_calibration/*.jsonl` | v2 只读映射与 commissioning 原始证据 |
| `piper_shadow_qa/` | 2026-08-04 只读影子矩阵历史证据 |
| `*_batch_qa.json` | 数据集自动 QA 报告 |
| `orbbec_*.json`、`*_snapshots/` | 相机只读验收与快照 |
| `smolvla_*.log`、`act_*.log` | 训练/评估日志 |

2026-08-06 的 ACT 070000 有一次起始包络拒绝，发生在使能前、发送 0 条命令；
有效结果为 50/50 动作成功，以及三次 30 秒运行发送 850、845、848 条动作且
`fault=None`。SmolVLA 004000 的 30 秒 RTC 诊断发送 888 条命令且
`fault=None`。模型任务表现结论见各自指南，不只看退出码。

定期删除空文件、重复失败日志和可重建缓存；不要删除仍被
`piper_active_calibration_v2.json` 的证据哈希引用的标定 JSONL。2026-07-30
之前的硬件安装日志位于本机外部归档
`/home/ubuntu22/Piper-VLA-local-archive/2026-07-30/`。
