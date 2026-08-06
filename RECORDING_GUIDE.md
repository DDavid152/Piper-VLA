# Piper 数据采集指南

## 当前采集协议

正式数据由原生 Piper master/follower 遥操作采集：follower 提供
`observation.state`，master 提供 `action`，两者均为 J1～J6（度）加夹爪
（毫米）的 7 维向量；`front` 和 `wrist` 同步录制 RGB 640×480@30 FPS。

固定任务文本：

```text
夹住紫色手提袋顶部订合在一起的两根橙黄色提带，将袋子竖直提离桌面约10厘米，保持悬空2秒，再将袋子放回原位，松开提带并将夹爪退离。
```

正式配置为 `config/record_piper_purple_bag_lift_manual_v1.yaml`。该数据同时供
SmolVLA 与 ACT 使用；不要为某个模型单独改变采集定义。

## 采集前

1. follower 先上电并等待约 5 秒，master 后上电并等待约 5 秒。
2. 两臂共享官方主从 CAN，总线周围无第三方控制器。
3. 确认 front/wrist 序列号、安装位置、画面和曝光正常。
4. 清空工作区，测试物理急停，确认夹爪和提带处于统一起始条件。
5. 启动 CAN 并检查 1 Mbps、`UP`、`ERROR-ACTIVE`、零总线错误。

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details -statistics link show can0
```

可先运行只读网页检查两路图像、state/action 和 CAN TX：

```bash
python scripts/view_piper_dataset_dashboard.py --task "上述固定任务文本"
```

录制前退出网页，避免争用相机。

## 录制

```bash
source /home/ubuntu22/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
cd /home/ubuntu22/Piper-VLA
lerobot-record --config_path config/record_piper_purple_bag_lift_manual_v1.yaml
```

每条示教应从相近但不机械复制的起点开始，完整包含接近、夹取、提起、悬停、
放回、松开和退离。碰撞、夹错、画面冻结、未完成或明显失去同步的 episode
标为失败，不进入 clean 数据集。

## 验收与版本化

```bash
python scripts/verify_piper_dataset.py \
  --repo-id local/piper_purple_bag_two_handle_lift_manual_v1_clean \
  --root datasets/piper_purple_bag_two_handle_lift_manual_v1_clean \
  --output logs/piper_manual_v1_clean_batch_qa.json
```

自动 QA 后仍要逐条看视频。新增数据创建 v2 或后续版本，不能覆盖 v1 clean；
重新排除失败条目、生成安全基线，并让 SmolVLA 和 ACT 使用同一版本训练。
完整质量标准见 [DATASET_QA_REPORT.md](DATASET_QA_REPORT.md)。
