# Piper-VLA

Piper-VLA 是一套基于 LeRobot v0.6.0 的双 Piper 实机数据采集工程。两台
Piper 使用官方原生 master/follower 共享 CAN 遥操作，主机只被动记录
follower 状态和 master 目标；两台 Orbbec Gemini 335L 分别提供远程固定
视角和末端视角。

## 当前状态

- `lerobot` Conda 环境、CUDA 12.8、视频编解码和 LeRobot CLI 已验收。
- 两台 Orbbec 可同时以 RGB 640×480@30 FPS 运行。
- `piper` Robot、`piper_master` Teleoperator 和 `orbbec` Camera 三个项目
  插件已安装。
- 训练数据网页已实机同时读取两路图像、7 维 follower observation 和
  7 维 master action，采样约 30 FPS，主机 CAN TX 增量为 0。
- 当前手动数据集有 3 条，其中 episode 2 通过自动结构 QA，episode 0、1
  因视频缺帧不可训练；尚未形成足量正式训练数据，也未下载 SmolVLA 权重。
- episode 2 已完成 2 步离线 ACT 训练冒烟测试和 checkpoint 重载推理，
  证明本机数据加载、CUDA 训练和模型保存链路正常。

数据流如下：

```text
Piper master ──官方共享 CAN──> Piper follower
       │                           │
       └─ master target(action)    └─ joint/gripper feedback(observation)
                         │
              LeRobot 被动读取插件
                         │
front RGB + wrist RGB ───┴──> LeRobotDataset episode
```

## 目录

| 路径 | 作用 |
|---|---|
| `config/` | 相机、Piper 原生主从及 LeRobot 录制配置 |
| `environment/` | Conda/Pip 版本约束、精确锁文件和激活钩子 |
| `plugins/` | Orbbec Camera、Piper Robot、Piper Master Teleoperator |
| `scripts/` | 环境安装、验收、相机检查和训练数据网页 |
| `tests/` | 插件、协议解析和网页数据结构测试 |
| `logs/` | 本机运行日志输出目录；产物不提交 Git |
| `third_party/lerobot/` | 固定提交的 LeRobot 上游源码，editable 安装依赖 |
| `datasets/` | 本地录制数据集，自动忽略且当前可不存在 |

`third_party/lerobot` 虽不提交本仓库，但不能在当前环境中删除：
`lerobot==0.6.0` 以 editable 方式直接指向该目录。

## 常用命令

所有 Python 命令先进入项目环境：

```bash
cd /home/ubuntu22/Piper-VLA
source /home/ubuntu22/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
```

完整软件验收：

```bash
./scripts/verify_environment.sh
```

双相机 30 秒检查：

```bash
python scripts/verify_orbbec_cameras.py \
  --duration-s 30 \
  --minimum-fps 28 \
  --output logs/orbbec_dual_rgb_latest.json \
  --snapshot-dir logs/orbbec_dual_rgb_latest_snapshots
```

### 每日实机启动顺序

系统重启或重新插拔 USB-CAN 后，Linux 不会自动恢复 `can0` 的 1 Mbps 配置和
UP 状态。每次实机运行都按以下顺序重新准备：

1. 完成 [SAFETY_CHECKLIST.md](SAFETY_CHECKLIST.md)，确认急停、工作区和接线。
2. 确认两臂已按 Piper 官方原生 master/follower 方案接入共享 CAN。
3. follower 先上电，等待约 5 秒至状态稳定。
4. master 后上电，再等待约 5 秒。
5. 配置电脑端 SocketCAN，并检查状态：

   ```bash
   sudo ip link set can0 type can bitrate 1000000
   sudo ip link set can0 up
   ip -details -statistics link show can0
   ```

6. 确认输出包含 `state UP`、`can state ERROR-ACTIVE` 和
   `bitrate 1000000`，且 `bus-errors`、`error-warn`、`error-pass`、
   `bus-off` 均为 0。
7. 最后才启动训练数据网页或 `lerobot-record`。

上述命令只配置并启动电脑端 CAN 控制器，不会设置两臂的主从角色，也不会
发送使能、回零或运动指令。在当前共享 CAN 接线中，`can0` 为 UP 是原生遥操作
稳定工作的必要条件之一；只读程序运行时主机 TX 应保持为 0。

训练数据实时网页：

```bash
python scripts/view_piper_dataset_dashboard.py \
  --task "准确、固定的任务指令"
```

浏览器地址为 `http://127.0.0.1:8765/`。只检查相机时追加
`--camera-only`。网页和 `lerobot-record` 都会独占相机，二者不得同时运行。
网页启动时只尝试连接 Piper 一次；若网页早于 `can0` 启动，或 USB-CAN
重新枚举，先按 `Ctrl+C` 退出，再在 CAN 状态正常后重新运行网页。

首次本地试采保持模板不变，通过命令行覆盖 task、数据集名称和独立输出
目录，例如：

```bash
lerobot-record \
  --config_path config/record_piper_native.example.yaml \
  --dataset.repo_id local/piper_trial_001 \
  --dataset.root /home/ubuntu22/Piper-VLA/datasets/piper_trial_001 \
  --dataset.single_task "准确、固定的任务指令"
```

运行全部测试：

```bash
python -m unittest discover -s tests -v
```

刷新环境锁文件：

```bash
./scripts/freeze_environment.sh
```

从零重建环境的完整顺序见 [ENVIRONMENT.md](ENVIRONMENT.md)。

## 安全边界

- 每次实机运行都必须重新完成 [SAFETY_CHECKLIST.md](SAFETY_CHECKLIST.md)。
- 软件不会自动启动 CAN；共享接口必须是 1 Mbps，并核对 USB-CAN 序列号。
- Piper 插件禁止软件使能、回零和运动控制；`send_action()` 只验证待记录数据。
- 官方原生 master/follower 链路负责实体遥操作，主机 CAN TX 应保持为 0。
- 未确认急停、工作空间和上电顺序时，不运行 dashboard、record、回放或推理。

## 专项文档

- [ENVIRONMENT.md](ENVIRONMENT.md)：软件版本、重建和验收结果。
- [HARDWARE_INVENTORY.md](HARDWARE_INVENTORY.md)：设备身份与实机验收。
- [RECORDING_GUIDE.md](RECORDING_GUIDE.md)：采集前检查和录制接口。
- [PROJECT_STATUS.md](PROJECT_STATUS.md)：当前阶段、已完成项和下一步。
- [SAFETY_CHECKLIST.md](SAFETY_CHECKLIST.md)：每次实机操作前的安全确认。
- [DATASET_QA_REPORT.md](DATASET_QA_REPORT.md)：首次试采后的数据质量记录模板。
- [TRAINING_GUIDE.md](TRAINING_GUIDE.md)：训练准入、冒烟验证和 SmolVLA
  正式微调命令。
