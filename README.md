# Piper-VLA

基于 LeRobot v0.6.0 的双 Piper 实机模仿学习工程，覆盖双 Orbbec 数据采集、
SmolVLA/ACT 训练、Piper 标定、安全限速和真实机械臂 rollout。

## 当前状态（2026-08-06）

- 正式 clean 数据集：51 episodes、42,066 帧、30 FPS、双路 RGB 640×480、
  7 维状态与 7 维绝对动作。
- SmolVLA：10,000 步训练完成，`004000` 验证损失最低；RTC 主动控制链已连续
  运行 30 秒，但任务效果不佳，保留为 VLA 对照基线。
- ACT：100,000 步训练完成，`070000` 验证损失最低（0.1225）；50 动作实机
  测试通过，三次完整 30 秒运行正常结束，发送 845～850 条动作且 `fault=None`。
- 主动控制：`piper_active` 使用生成并验证的 v2 标定、USB-CAN 身份校验、
  Piper 物理限位、训练数据 p99 单步限速、反馈/动作看门狗和预算急停。
- ACT checkpoint 全部保存在 `outputs/`，由 `.gitignore` 排除，不提交 Git。

## 数据流

```text
采集：Piper master ──原生共享 CAN──> follower
          │                           │
          └─ action[7]                └─ observation.state[7]
front RGB + wrist RGB ────────────────┴─> LeRobotDataset

部署：双相机 + follower state ──> SmolVLA / ACT ──> piper_active 安全层 ──> follower
```

## 快速入口

```bash
cd /home/ubuntu22/Piper-VLA
source /home/ubuntu22/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
```

- [ACT_GUIDE.md](ACT_GUIDE.md)：ACT 训练结果、频率语义、50 动作和可重复 30 秒部署。
- [SMOLVLA_GUIDE.md](SMOLVLA_GUIDE.md)：SmolVLA 训练、影子诊断和 RTC 实机基线。
- [TRAINING_GUIDE.md](TRAINING_GUIDE.md)：共享数据准入和两种策略的对照原则。
- [RECORDING_GUIDE.md](RECORDING_GUIDE.md)：数据采集与 QA。
- [SAFETY_CHECKLIST.md](SAFETY_CHECKLIST.md)：每轮实机操作前检查。
- [PROJECT_STATUS.md](PROJECT_STATUS.md)：当前结果和待解决问题。

## 常用命令

```bash
# 环境与测试
./scripts/verify_environment.sh
python -m unittest discover -s tests -v

# clean 数据复验
python scripts/verify_piper_dataset.py \
  --repo-id local/piper_purple_bag_two_handle_lift_manual_v1_clean \
  --root datasets/piper_purple_bag_two_handle_lift_manual_v1_clean \
  --output logs/piper_act_dataset_qa.json

# ACT 50 动作 / 30 秒
bash scripts/run_piper_act_rollout.sh --profile 50
bash scripts/run_piper_act_rollout.sh --profile 30s
```

## 安全边界

- 数据采集使用只读 `piper`/`piper_master`；主动部署只使用独立
  `piper_active`。
- 主动运行前 master 必须关闭或物理隔离，并通过当前姿态部署预检。
- 每轮主动运行达到动作或时间预算后锁存软件急停；下一轮必须人工恢复、重新
  摆放并再次预检，禁止无人值守循环。
- 数据集、模型、运行日志和标定原始证据均为本机资产；其中 checkpoint、
  dataset 和 logs 已被 Git 忽略。
