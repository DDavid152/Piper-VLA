# 本机运行日志

脚本将环境验收、相机检查、CAN 抓包摘要和快照写入本目录。除本 README
外，目录内容均被 `.gitignore` 忽略，不应提交 GitHub。

常见产物：

| 名称模式 | 来源 |
|---|---|
| `environment_verify_*.log` | `scripts/verify_environment.sh` |
| `orbbec_*.json` | `scripts/verify_orbbec_cameras.py` |
| `*_snapshots/` | 相机验收快照 |
| `candump-*.log` | 人工执行的被动 CAN 抓包 |
| `*_summary.log` | 硬件或网页验收摘要 |

2026-07-30 以前的 34 个历史文件已移到本机外部归档：

```text
/home/ubuntu22/Piper-VLA-local-archive/2026-07-30/
```

归档中的 `MANIFEST.sha256` 记录哈希、字节数和原相对路径。运行产物应定期移
出项目或删除；正式数据集必须写入 `datasets/`，不能写入本目录。
