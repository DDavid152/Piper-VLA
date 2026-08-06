# 脚本目录

## 环境与设备

| 脚本 | 作用 |
|---|---|
| `install_*.sh` | 安装 Miniforge、系统依赖、Python 环境和 Orbbec udev |
| `verify_environment.sh` | 验证依赖、CUDA、视频、插件和测试 |
| `freeze_environment.sh` | 更新 Conda/Pip 可重建清单 |
| `verify_orbbec_cameras.py` | 双相机只读 FPS、帧连续性和快照验收 |
| `view_piper_dataset_dashboard.py` | 只读展示双相机、follower state 和 master action |
| `verify_piper_dataset.py` | 全 episode 的 Parquet/MP4/时序/7 维结构 QA |

## 标定与安全

| 脚本 | 作用 |
|---|---|
| `generate_piper_safety_baseline.py` | 从 51 条 clean 数据生成安全分布 |
| `capture_piper_passive_mapping.py` | 零主机 TX 采集 master/follower v2 映射证据 |
| `commission_piper_calibration.py` | receive-only 预检或十二项 ±0.5° commissioning |
| `check_piper_qa_pose.py` | 只读显示当前姿态与训练首帧包络偏差 |
| `recover_piper_emergency_stop.py` | 经反馈和 CAN 门禁后清除软件急停，不使能/回零 |

## 模型部署

| 脚本 | 作用 |
|---|---|
| `run_piper_act_rollout.sh` | ACT 070000 的 50 动作或 30 秒 sync 实机运行 |
| `run_piper_active_micro.sh` | SmolVLA 004000 的分级微动、RTC 与 30 秒诊断 |

ACT：

```bash
bash scripts/run_piper_act_rollout.sh --profile 50
bash scripts/run_piper_act_rollout.sh --profile 30s
```

也可用 `--checkpoint PATH` 测试其他本地 ACT checkpoint。脚本使用 current-pose
receive-only 预检、10% 速度、独立时间戳 runtime/command 日志和人工确认；不
自动恢复或循环。

SmolVLA：

```bash
bash scripts/run_piper_active_micro.sh --profile diagnostic30 --start-pose current
```

标定证据的采集和 commissioning 是低频维护流程，参数较多时直接运行对应脚本
的 `--help`。实际运动前始终先阅读 [SAFETY_CHECKLIST.md](../SAFETY_CHECKLIST.md)。

所有脚本假设项目路径为 `/home/ubuntu22/Piper-VLA`、Conda 环境为 `lerobot`。
安装脚本不配置 CAN；主动脚本不允许非交互执行。
