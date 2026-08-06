# 配置目录

设备身份始终使用固定序列号，不依赖 `/dev/video*`、`can0` 的插入顺序或人工
记忆。更改相机键、单位、FPS 或数据 schema 后必须创建新数据集版本。

| 文件 | 用途 |
|---|---|
| `cameras.json` | front/wrist Orbbec 序列号与 RGB 640×480@30 参数 |
| `piper_native_master_slave.json` | 原生遥操作采集的共享 CAN、7 维特征和只读边界 |
| `record_piper_native.example.yaml` | 新任务最小录制模板，task/repo/root 必须先修改 |
| `record_piper_purple_bag_lift_manual_v1.yaml` | 当前紫色手提袋手动切分采集配置 |
| `piper_safety_baseline_v1.json` | 从 51 条 clean episode 生成的物理/数据分布安全基线 |
| `piper_active_calibration_v1.json` | `verified=false` 的 fail-closed 历史骨架，不可部署 |
| `piper_active_calibration_v2.json` | 当前经只读映射与十二项 commissioning 验证的主动标定 |
| `train_piper_purple_bag_act_v1.yaml` | ACT batch 8、100K 步正式训练配置 |

SmolVLA 正式训练使用命令行参数，见
[SMOLVLA_GUIDE.md](../SMOLVLA_GUIDE.md)；ACT 使用仓库内 YAML，见
[ACT_GUIDE.md](../ACT_GUIDE.md)。

## 安全配置来源

`piper_safety_baseline_v1.json` 由下列命令从 clean 数据生成；数据变化后要写入
新版本，不能手改旧文件：

```bash
python scripts/generate_piper_safety_baseline.py \
  --output config/piper_safety_baseline_v1.json
```

v2 标定只能由 `capture_piper_passive_mapping.py`、
`commission_piper_calibration.py` 和 active 插件的校准生成器产生。配置保存原始
证据 SHA-256，证据缺失或被修改时主动插件会拒绝运动。绝不能把 v1 的
`verified` 手工改为 `true`。

## 录制配置

当前 manual 配置的 `num_episodes=0` 表示持续采集：在 `READY` 按 Enter 开始和
结束一条，在 `READY` 按 `q` 退出。追加已有数据集时必须明确传入 `--resume=true`。
完整操作见 [RECORDING_GUIDE.md](../RECORDING_GUIDE.md)。
