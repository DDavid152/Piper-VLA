# 配置目录

本目录保存可提交、可审查的硬件身份和 LeRobot 录制模板。设备选择必须使用
固定序列号，不能依赖 `/dev/video*` 或 USB 插入顺序。

## 文件

### `cameras.json`

训练数据网页和相机验收脚本的共享配置：

- `capture`：RGB 640×480@30、颜色格式、超时和预热时间；
- `depth_enabled=false`：第一阶段数据集不采集深度；
- `front=CP28563000XR`：远程固定相机；
- `wrist=CP28563000XP`：机械臂末端相机。

修改分辨率、帧率或数据键会改变数据集 schema，必须建立新数据集版本并重新
做双相机验收。

### `piper_native_master_slave.json`

训练数据网页使用的 Piper 共享 CAN 配置：

- `can.interface=can0`、`bitrate=1000000`；
- `expected_adapter_serial` 是最终共享总线 USB-CAN；
- `features` 固定为 J1～J6 和 `gripper.pos`；
- 关节单位为度，夹爪单位为毫米；
- `safety` 明确禁止软件运动、使能和回零。

该配置只描述程序期望使用的接口和身份，不会自动设置或启动 `can0`。每次
系统重启或 USB-CAN 重新插拔后，须按
[RECORDING_GUIDE.md](../RECORDING_GUIDE.md) 中的固定顺序先上电两臂、配置
CAN，再启动 dashboard 或录制程序。若 dashboard 早于 CAN 启动，须退出并
重新运行，不能依赖现有进程自动重连。

### `record_piper_native.example.yaml`

`lerobot-record` 的最小本地试采模板，组合：

- `robot.type=piper` 和两台 Orbbec 相机；
- `teleop.type=piper_master`；
- 本地数据集根目录、30 FPS、episode 时长和数量；
- `push_to_hub=false`，不会上传 Hugging Face。

使用前必须：

1. 将 `REPLACE_WITH_THE_EXACT_TASK_INSTRUCTION` 替换为准确且全数据集一致的
   任务文本，或通过 `--dataset.single_task` 明确覆盖；
2. 为每个试验设置独立的 `repo_id` 和 `root`；
3. 首次仅录制 1 个、10 秒的 episode；
4. 确保训练数据网页已退出并释放相机。

解析配置但不连接硬件：

```bash
source /home/ubuntu22/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
python -m unittest discover -s tests -p 'test_piper_plugins.py' -v
```

### `record_piper_purple_bag_lift_manual_v1.yaml`

紫色手提袋任务的命令行手动切分配置。设备连接后程序停在 `READY`，不会在
后台自动开始写入；按 Enter 开始当前 episode，再按 Enter 结束并保存。
`num_episodes=0` 表示本次进程可持续采集任意数量，直到在 `READY` 阶段按
`q` 退出。首次运行创建独立的 `*_manual_v1` 数据集；后续追加必须传入
`--resume=true`。

### `train_piper_purple_bag_act_smoke_v1.yaml`

仅用于验证训练链路的 2 步 ACT 配置。它只选择当前自动结构 QA 通过的
episode 2，关闭预训练权重下载，并使用较小网络在 CUDA 上完成前向、反向
和 checkpoint 保存。该配置不是正式策略配方，不能用于实机部署。

正式 SmolVLA 训练步骤见
[TRAINING_GUIDE.md](../TRAINING_GUIDE.md)。
