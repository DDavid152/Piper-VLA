# Piper 紫色手提袋训练指南

## 当前结论

当前手动数据集共有 3 条 episode。自动 QA 的训练准入结果为：

- 可读取并通过自动结构 QA：episode 2；
- 禁止直接用于训练：episode 0、1，因为各有一路视频缺帧；
- episode 2 仍须人工确认夹持对象、提起高度、2 秒悬停、放回和退离。

已使用 episode 2 完成一次离线训练冒烟测试：TorchCodec 成功读取首、中、
末帧及批数据，ACT 在 RTX 4000 Ada 上完成 2 次前向、反向和优化器更新，
保存的 checkpoint 可重新加载并输出有限的 7 维动作。

这证明当前数据格式和软件训练链路可用，不代表只有一条合格示教就能训练出
可部署策略。

## 训练准入流程

每次训练前先运行：

```bash
source /home/ubuntu22/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
cd /home/ubuntu22/Piper-VLA

python scripts/verify_piper_dataset.py \
  --output logs/piper_manual_batch_qa_latest.json
```

JSON 中：

- `passed_episode_indices` 是自动结构检查通过的编号；
- `failed_episode_indices` 必须从训练配置中排除；
- 只有自动 QA 和人工动作语义 QA 都通过的编号才能进入训练。

训练集与验证集应按完整 episode 划分，不能把同一条 episode 的前后帧随机
分到两边。对于这个固定单任务，建议先获得至少 50 条成功且通过 QA 的示教，
将其作为第一轮工程基线，再根据离线验证和实机成功率决定是否扩充至更多
初始位置、提带形态和操作轨迹。50 条是起步建议，不是模型保证。

## 已执行的训练冒烟测试

配置：
`config/train_piper_purple_bag_act_smoke_v1.yaml`

重新运行时必须换一个尚不存在的输出目录：

```bash
lerobot-train \
  --config_path config/train_piper_purple_bag_act_smoke_v1.yaml \
  --output_dir outputs/train_smoke/act_episode2_smoke_v2
```

该配置只训练 2 步、只选择 episode 2，并关闭预训练权重下载。它用于验证：

- 两路 640×480 视频可被训练 DataLoader 解码；
- 7 维状态和动作可形成 16 步 action chunk；
- CUDA 前向、反向和优化器更新正常；
- checkpoint、归一化处理器和训练状态可保存、重新加载。

它不是正式 ACT 配置，也不应把这个 2 步模型用于实机控制。

本次结果：

```text
数据帧：856
训练 episode：1
更新步数：2
可训练参数：12,309,383
峰值显存：约 2.46 GB
checkpoint：outputs/train_smoke/act_episode2_smoke_v1/checkpoints/000002
```

## 正式 SmolVLA 微调

正式训练前必须满足：

1. 形成足够数量且双重 QA 通过的 episode；
2. 将所有通过编号写入 `--dataset.episodes`，不得包含失败编号；
3. 为训练留出独立的验证 episode；建议第一轮采用
   `--dataset.eval_split=0.1`；
4. 下载 `lerobot/smolvla_base`。当前机器尚未下载该权重，第一次运行
   `--policy.path=lerobot/smolvla_base` 会需要网络和磁盘空间；
5. 输出目录必须是新目录，避免覆盖旧实验。

示例命令如下。先把 `PIPER_TRAIN_EPISODES` 替换为实际通过人工和自动 QA 的
编号列表：

```bash
PIPER_TRAIN_EPISODES='[2,3,4,5]'

lerobot-train \
  --policy.path=lerobot/smolvla_base \
  --policy.device=cuda \
  --policy.push_to_hub=false \
  --dataset.repo_id=local/piper_purple_bag_two_handle_lift_manual_v1 \
  --dataset.root=/home/ubuntu22/Piper-VLA/datasets/piper_purple_bag_two_handle_lift_manual_v1 \
  --dataset.episodes="$PIPER_TRAIN_EPISODES" \
  --dataset.video_backend=torchcodec \
  --dataset.eval_split=0.1 \
  --batch_size=1 \
  --num_workers=2 \
  --steps=30000 \
  --eval_steps=1000 \
  --max_eval_samples=1000 \
  --save_freq=5000 \
  --log_freq=100 \
  --output_dir=outputs/train/smolvla_purple_bag_v1 \
  --wandb.enable=false
```

RTX 4000 Ada 有 20 GB 显存，首次正式运行从 `batch_size=1` 开始。确认显存、
吞吐和稳定性后再尝试提高到 2；不要根据冒烟 ACT 的显存占用推断 SmolVLA
也能使用相同 batch size。

如果通过 checkpoint 续训：

```bash
lerobot-train \
  --config_path outputs/train/smolvla_purple_bag_v1/checkpoints/last/pretrained_model/train_config.json \
  --resume=true
```

## 训练完成后的判断

训练 loss 下降只表示模型更贴合训练数据，不能单独证明机械臂任务成功。至少
还要检查：

- 保留的验证 episode 上 loss 没有持续恶化；
- 随机抽取视频帧时，模型输出的 7 维动作均为有限值且范围合理；
- 训练和验证 episode 没有重叠；
- checkpoint 能重新加载；
- 最终实机评估使用低速、可急停、人员在场的独立安全流程。

在正式 SmolVLA checkpoint 产生并通过离线评估前，不把模型接到实体
follower 的运动控制路径。
