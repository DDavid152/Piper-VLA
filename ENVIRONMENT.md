# 软件环境

## 当前基线

| 项目 | 版本 / 位置 |
|---|---|
| OS / kernel | Ubuntu 22.04.5 LTS / 6.8.0-136-generic |
| GPU | NVIDIA RTX 4000 Ada，20,475 MiB |
| Python / Conda | 3.12.13 / Miniforge 26.3.2-3 |
| LeRobot | v0.6.0，提交 `30da8e687a6dfc617fcd94afc367ac7071c376ce` |
| PyTorch / torchvision | 2.11.0+cu128 / 0.26.0+cu128 |
| Piper SDK / python-can | 0.6.1 / 4.6.1 |
| Orbbec | pyorbbecsdk2 2.0.18，SDK 2.7.6 |
| 环境目录 | `/home/ubuntu22/miniforge3/envs/lerobot` |

LeRobot、Orbbec、Piper 被动 Robot、Piper master Teleoperator 和 Piper active
Robot 均以 editable 模式从本项目安装。没有安装系统 CUDA Toolkit；PyTorch
使用官方 CUDA 12.8 wheel。

SmolVLA 基座、SmolVLM2 和 torchvision ImageNet ResNet18 权重均已缓存在本机。
SmolVLA 和 ACT 的训练、评估与实机推理可使用离线模式：

```bash
source /home/ubuntu22/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
export HF_HUB_OFFLINE=1
export TRANSFORMERS_OFFLINE=1
```

## 重建与验证

```bash
./scripts/install_miniforge.sh
pkexec ./scripts/install_system_dependencies.sh
./scripts/install_python_environment.sh
pkexec ./scripts/install_orbbec_udev.sh
./scripts/verify_environment.sh
./scripts/freeze_environment.sh
```

安装脚本不修改 NVIDIA 驱动，不配置或启动 CAN，也不对机械臂发送控制命令。
精确依赖位于 `environment/`：

- `environment.yml` 和 `constraints.txt`：最小声明与兼容约束；
- `conda-explicit-linux-64.txt`：Conda 精确包 URL；
- `environment.resolved.yml`、`requirements.lock.txt`、`pip-inspect.json`：当前解析结果。

## 已验收能力

- CUDA GPU 运算、LeRobot CLI、PyAV/TorchCodec 640×480@30 FPS 编解码通过。
- 两台 Orbbec 以固定序列号并发 RGB 640×480@30 FPS 运行通过。
- 被动 Piper Robot/Teleoperator 只读取 follower/master，主机 CAN TX 不增长。
- 51 条 clean 数据集自动 QA 全通过。
- SmolVLA 10K 与 ACT 100K 离线训练完成。
- v2 主动标定已验证；SmolVLA RTC 30 秒和 ACT 50 动作/30 秒受控实机链路通过。

`config/piper_active_calibration_v1.json` 是故意 fail-closed 的历史骨架；当前主动
部署只使用经过证据工具生成并验证的 `piper_active_calibration_v2.json`。
模型和 checkpoint 保留在本机 `outputs/`，由 `.gitignore` 排除。
