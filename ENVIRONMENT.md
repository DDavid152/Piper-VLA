# Piper-VLA 软件环境

## 主机基线

| 项目 | 当前值 |
|---|---|
| 操作系统 | Ubuntu 22.04.5 LTS |
| 内核 | 6.8.0-136-generic |
| 架构 | x86_64 |
| GPU | NVIDIA RTX 4000 Ada Generation |
| GPU 显存 | 20475 MiB |
| NVIDIA 驱动 | 595.84 |
| 驱动报告 CUDA 上限 | 13.2 |
| 内存 | 62 GiB |
| 项目磁盘可用空间 | 约 1.7 TiB |

## 已安装软件基线

| 组件 | 实际版本 / 来源 |
|---|---|
| Miniforge | 26.3.2-3，`/home/ubuntu22/miniforge3` |
| Conda | 26.3.2 |
| Python | 3.12.13 |
| FFmpeg | Conda 7.1.1（TorchCodec 运行库）；系统 4.4.2 |
| LeRobot | v0.6.0 / `30da8e687a6dfc617fcd94afc367ac7071c376ce` |
| PyTorch | 2.11.0+cu128 |
| torchvision | 0.26.0+cu128 |
| torchcodec | 0.11.1（运行时模块报告 `0.11.1+cpu`） |
| PyAV | 15.0.0 |
| Piper SDK | 0.6.1 |
| python-can | 4.6.1 |
| pyorbbecsdk2 | 2.0.18（Orbbec SDK 2.7.6） |

LeRobot 以 editable 模式从 `third_party/lerobot` 安装，启用
`core_scripts,training,smolvla` extras。环境中未安装 LIBERO 或 MuJoCo，
也未下载模型权重。

## 环境入口

```bash
source /home/ubuntu22/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
```

激活脚本会把 `${CONDA_PREFIX}/lib` 放在 `LD_LIBRARY_PATH` 前部，使
TorchCodec 使用 Conda FFmpeg 7 及配套的 C++ 运行库；退出环境时恢复原值。

## 重建顺序

```bash
./scripts/install_miniforge.sh
pkexec ./scripts/install_system_dependencies.sh
./scripts/install_python_environment.sh
pkexec ./scripts/install_orbbec_udev.sh
./scripts/verify_environment.sh
./scripts/freeze_environment.sh
```

管理员脚本只安装通用系统依赖和 Orbbec 设备访问规则，不安装系统 CUDA
Toolkit，不修改 NVIDIA 驱动，也不配置或启动 CAN。

## 可重建清单

- `environment/environment.yml`：最小 Conda 环境声明；
- `environment/constraints.txt`：关键 Python 兼容性约束；
- `environment/conda-explicit-linux-64.txt`：当前平台的精确 Conda URL 清单；
- `environment/environment.resolved.yml`：含 Pip 依赖的解析结果；
- `environment/requirements.lock.txt`：完整 Pip 版本锁；
- `environment/pip-inspect.json`：Pip 包元数据和依赖来源。

固定下载来源写在安装脚本中：Miniforge 使用 conda-forge GitHub release
并校验 SHA-256；LeRobot 使用 Hugging Face 官方 GitHub 仓库及固定提交；
PyTorch 使用官方 CUDA 12.8 wheel 索引；Orbbec udev 规则使用
pyorbbecsdk v2.0.18 官方仓库文件并校验 SHA-256。其余 Python 包来自官方
PyPI。

## 验收结果

2026-07-28 最终验收通过：

- `pip check` 无冲突，固定包全部可导入；
- CUDA 12.8 可用，识别 NVIDIA RTX 4000 Ada Generation，并完成 GPU
  矩阵运算；
- `lerobot-record`、`lerobot-train`、`lerobot-dataset-viz` 均可启动；
- PyAV 成功编码并解码 30 帧 640×480、30 FPS 合成视频，TorchCodec
  再次解码出 30 帧；
- Orbbec 原生扩展的 `ldd` 无缺失项；
- 只读 Orbbec 枚举为 0 台，因此本阶段没有序列号或固件结论；
- Piper 仅做包导入，没有打开 CAN、创建控制接口或发送报文。

原始日志：

- `logs/software_setup_20260728_155401.log`
- `logs/system_dependencies_20260728_163755.log`
- `logs/environment_verify_20260728_164416.log`

安装日志保留了 TorchCodec 在缺少 FFmpeg 7 运行库时的首次失败，以及安装
Conda FFmpeg 7 和激活钩子后的成功复验，便于追溯。
