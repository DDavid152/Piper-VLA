# 测试目录

项目使用 Python `unittest`。默认测试通过 fake SDK/设备运行，不连接真实
机械臂或相机，也不发送 CAN。

| 文件 | 覆盖范围 |
|---|---|
| `test_orbbec_camera_plugin.py` | Camera 注册、配置与旋转尺寸 |
| `test_piper_plugins.py` | 被动 Robot/Teleoperator、单位、健康和 CAN 解析 |
| `test_dataset_dashboard.py` | 网页字段与模拟 7 维数据流 |
| `test_batch_dataset_qa.py` | 共享 MP4、episode 区间和缺帧检测 |
| `test_piper_active_plugin.py` | 主动准入、限位/限速、SDK 顺序、预算和看门狗 |
| `test_piper_calibration_tools.py` | v2 证据发布、commissioning 与急停门禁 |

```bash
source /home/ubuntu22/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
python -m unittest discover -s tests -v
```

2026-08-06 复验：73 项全部通过。

故障注入测试会故意输出 fault/error 日志；最终 `OK` 才表示拦截符合预期。
真实 FPS、零发送和运动表现必须分别由相机工具、CAN 统计与实机日志验证。
