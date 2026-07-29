# Piper-VLA 项目状态

## 当前阶段

统一软件环境配置已经完成并通过验收。项目可进入只读硬件审计和设备插件
设计阶段，尚未进入主动设备测试或实机采集。

## 已确认

- 主机系统：Ubuntu 22.04.5 LTS，x86_64。
- GPU：NVIDIA RTX 4000 Ada Generation，20 GB。
- NVIDIA 驱动：595.84。
- `lerobot` 环境：Python 3.12.13、LeRobot v0.6.0、
  PyTorch 2.11.0+cu128。
- 软件依赖、Orbbec udev 规则、环境锁文件和验证日志已经落地。
- CUDA、LeRobot CLI、视频编解码及 Orbbec 原生依赖检查通过。
- 项目不包含 LIBERO、MuJoCo 或其他仿真环境。
- 本阶段不下载模型权重、不连接 CAN、不发送机械臂指令、不升级相机固件。

## 当前阻塞

- 当前未检测到 Orbbec 相机，无法记录相机序列号和固件。
- Piper、USB-CAN、急停和工作空间状态尚未完成只读现场审计。
- Piper、Teleoperator 和 Orbbec 的 LeRobot 插件尚未实现。

## 下一步

1. 在断开机械臂动力或确保急停有效的条件下完成只读硬件清点。
2. 固定两台 Piper、USB-CAN 和两台 Gemini 335L 的身份映射。
3. 根据审计结果实现设备插件，再单独审批主动测试和实机采集。
