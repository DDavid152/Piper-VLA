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
| LeRobot Orbbec 插件 | 0.1.0，项目内 editable 安装 |
| LeRobot Piper Robot 插件 | 0.1.0，项目内 editable 安装 |
| LeRobot Piper Master Teleoperator 插件 | 0.1.0，项目内 editable 安装 |

LeRobot 以 editable 模式从 `third_party/lerobot` 安装，启用
`core_scripts,training,smolvla` extras。环境中未安装 LIBERO 或 MuJoCo，
也未下载模型权重。三个项目内插件均以 editable 模式安装，分别为 LeRobot
注册 `orbbec` 相机、`piper` Robot 和 `piper_master` Teleoperator 类型。

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

2026-07-28 软件基线验收通过：

- `pip check` 无冲突，固定包全部可导入；
- CUDA 12.8 可用，识别 NVIDIA RTX 4000 Ada Generation，并完成 GPU
  矩阵运算；
- `lerobot-record`、`lerobot-train`、`lerobot-dataset-viz` 均可启动；
- PyAV 成功编码并解码 30 帧 640×480、30 FPS 合成视频，TorchCodec
  再次解码出 30 帧；
- Orbbec 原生扩展的 `ldd` 无缺失项；
- Piper 仅做包导入，没有打开 CAN、创建控制接口或发送报文。

2026-07-29 两台 Orbbec Gemini 335L 接入后完成相机专项验收：

- `pip check` 无依赖冲突，插件的 5 项单元测试全部通过；
- LeRobot 第三方插件发现可加载 `lerobot_camera_orbbec`，`orbbec` 类型可由
  `CameraConfig` 和相机工厂正确实例化；
- 两台设备均通过固定序列号找到，连接为 USB 3.2；
- 两路 RGB 以 640×480、30 FPS 并发运行 30 秒，各获得 900 个后台帧，
  帧率均为 30.0 FPS；
- 两路均未出现帧号间断、重复帧、时间戳回退、SDK 超时、读取失败或错误
  图像尺寸；
- 额外的两路 RGB+深度并发 10 秒带宽测试同样达到 30.0 FPS 且无上述错误；
- 正式项目配置仍保持 RGB-only，不把深度写入第一阶段 SmolVLA 数据集。

2026-07-29 Piper 数据接口和实时网页软件验收通过：

- `pip check` 无冲突，项目全部 16 项单元测试通过；
- 第三方插件发现、Robot/Teleoperator 工厂实例化和录制 YAML 解析通过；
- follower 观测和 master 动作均固定为 J1～J6 加夹爪的 7 维向量，单位
  分别为度和毫米；
- Piper Robot 与 Master Teleoperator 的运行路径均为被动接收，源码扫描
  未发现 CAN 发送、使能、回零或运动控制调用；
- 训练数据网页的双相机实机模式、单相机模式和 CAN 不可用降级模式通过；
- 双相机网页测试期间相机持续出帧、CAN 保持 DOWN、TX 计数保持 0；
- 共享 CAN 上的关节数据网页实机验收需在下一次现场安全确认后完成。

原始日志：

- `logs/software_setup_20260728_155401.log`
- `logs/system_dependencies_20260728_163755.log`
- `logs/environment_verify_20260728_164416.log`
- `logs/orbbec_dual_rgb_20260729_144204.json`
- `logs/orbbec_dual_rgb_depth_20260729_144321.json`
- `logs/orbbec_dual_camera_20260729_summary.log`
- `logs/orbbec_post_mount_rgb_20260729_161521.json`
- `logs/orbbec_post_mount_20260729_161521_summary.log`
- `logs/dataset_dashboard_20260729_summary.log`
- `logs/environment_verify_20260729_172421.log`

安装日志保留了 TorchCodec 在缺少 FFmpeg 7 运行库时的首次失败，以及安装
Conda FFmpeg 7 和激活钩子后的成功复验，便于追溯。
