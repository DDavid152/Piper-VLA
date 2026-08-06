# 训练与对照实验入口

模型专用内容已拆分：

- [SMOLVLA_GUIDE.md](SMOLVLA_GUIDE.md)：SmolVLA 配方、checkpoint 和部署基线。
- [ACT_GUIDE.md](ACT_GUIDE.md)：ACT 配方、checkpoint、频率与当前部署命令。

## 共享正式数据

```text
repo_id: local/piper_purple_bag_two_handle_lift_manual_v1_clean
root: datasets/piper_purple_bag_two_handle_lift_manual_v1_clean
episodes: 51
frames: 42,066
fps: 30
inputs: front RGB + wrist RGB + observation.state[7]
target: action[7]
```

每次新实验前运行：

```bash
python scripts/verify_piper_dataset.py \
  --repo-id local/piper_purple_bag_two_handle_lift_manual_v1_clean \
  --root datasets/piper_purple_bag_two_handle_lift_manual_v1_clean \
  --output logs/piper_training_dataset_qa.json
```

`eval_split=0.1` 按完整 episode 划分为 45 条训练、6 条验证，不进行帧级随机
拆分。两个模型使用同一 clean 数据、双相机键、7 维绝对动作、任务布置与现场
评分标准。

## 对照原则

- SmolVLA 与 ACT 都使用 `chunk_size=50`，但模型目标和推理机制不同。
- 两种 loss 不可直接比较；每种模型内部用验证 loss 选择 checkpoint。
- 实机比较固定相机、袋子、起始姿态、10% 速度和安全处理器。
- 完整成功必须同时满足：夹住两根提带、抬升约 10 cm、保持约 2 秒、放回、
  松开并退离。
- 单次成功或失败只作诊断；结论至少基于同条件重复运行。

## 本地资产

`datasets/`、`outputs/`、`logs/` 和所有 `*.safetensors` 已由 `.gitignore`
排除。ACT checkpoint 全部保留在本机，不因 Git 清理而删除。
