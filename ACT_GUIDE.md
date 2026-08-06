# ACT 训练与实机部署

## 正式训练

配置：`config/train_piper_purple_bag_act_v1.yaml`

```text
output: outputs/train/act_purple_bag_b8_100k_v1
architecture: LeRobot ACT + ImageNet ResNet18
parameters: 51,573,639
batch_size: 8
steps: 100,000
chunk_size: 50
n_action_steps: 50
temporal_ensemble: disabled
train/eval episodes: 45 / 6
```

ACT Transformer、VAE 和动作头从头训练；只有 ResNet18 使用 Torchvision 官方
ImageNet 权重，不加载其他机器人上的社区 ACT checkpoint。

```bash
mkdir -p logs
HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 \
lerobot-train \
  --config_path config/train_piper_purple_bag_act_v1.yaml \
  --output_dir outputs/train/act_purple_bag_b8_100k_v1 \
  2>&1 | tee logs/act_purple_bag_b8_100k_v1.log
```

重新训练时必须使用新的 `output_dir`，不要覆盖当前 checkpoint。

| checkpoint | eval loss |
|---|---:|
| `010000` | 0.1694 |
| `020000` | 0.1473 |
| `030000` | 0.1418 |
| `040000` | 0.1306 |
| `050000` | 0.1349 |
| `060000` | 0.1347 |
| `070000` | **0.1225** |
| `080000` | 0.1333 |
| `090000` | 0.1245 |
| `100000` | 0.1263 |

当前部署 checkpoint：

```text
outputs/train/act_purple_bag_b8_100k_v1/checkpoints/070000/pretrained_model
```

全部 checkpoint 继续保留在本机。`outputs/`、`**/checkpoints/` 和权重文件模式
已写入 `.gitignore`，执行 Git 操作不会提交或删除这些权重。

## 推理与控制频率

- `fps=30`、`interpolation_multiplier=1`：不增加中间插值点。
- 每次 ACT 推理输出 50 个动作，由同步队列逐个发送。
- 理论网络推理频率 `30/50=0.6 Hz`，视觉闭环约每 1.67 秒更新一次。
- 30 秒实测发送 845～850 条动作，即约 28.2～28.3 Hz；网络推理约 0.57 Hz。
- `piper_active` 的 p99 单步限速可能裁剪过大变化，这属于安全平滑，不是插值。

## 实机结果

- `50` profile：50/50 动作发送成功，动作预算正常停止，`fault=None`。
- `30s` profile：三次有效运行分别发送 850、845、848 条动作，均运行约 30 秒、
  无安全拒绝、由时间预算停止且 `fault=None`。
- 一次训练首帧包络拒绝发生在使能前；改用与 SmolVLA 对照一致的
  `current_physical` 起始模式后通过。

以上是控制链技术结果，完整任务表现仍需人工记录。

## 推荐命令

主动运行前 master 必须关闭或物理隔离，CAN 为 1 Mbps/ERROR-ACTIVE，操作员
持有物理急停。

```bash
# 首次或改动配置后先跑 50 动作
bash scripts/run_piper_act_rollout.sh --profile 50

# 恢复急停、重新摆放、关闭 master 后运行完整 30 秒
bash scripts/run_piper_act_rollout.sh --profile 30s
```

脚本每次都会执行当前姿态部署预检、要求交互确认并生成独立日志。每轮结束后：

```bash
python scripts/recover_piper_emergency_stop.py
```

禁止用 shell 循环自动重复主动运行；每轮之间必须人工检查、恢复和重新摆放。
需要比较其他本地 checkpoint 时可传入
`--checkpoint outputs/.../checkpoints/080000/pretrained_model`，日志名会自动包含
checkpoint 编号。

## 后续实验

如果 50 步开环窗口导致视觉纠偏不足，可在不重训的前提下尝试覆盖
`n_action_steps=10`，但这会把推理频率提高到约 3 Hz并增加同步停顿。调整前后
必须分别记录动作频率、块边界间隔和任务成功率。
