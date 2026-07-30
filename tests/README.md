# 测试目录

项目使用 Python 标准库 `unittest`，默认测试不需要连接真实机械臂或相机。

| 文件 | 覆盖范围 |
|---|---|
| `test_orbbec_camera_plugin.py` | Camera 注册、配置校验、工厂实例化和旋转尺寸 |
| `test_piper_plugins.py` | Robot/Teleoperator 注册、单位、反馈健康、CAN 协议解析、只读 action 和 YAML |
| `test_dataset_dashboard.py` | 训练数据网页字段和模拟的 7 维 observation/action 数据流 |

运行全部测试：

```bash
source /home/ubuntu22/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
python -m unittest discover -s tests -v
```

测试中的 `FakePiperInterface`、`FakeRobot` 和 `FakeTeleoperator` 不打开 CAN，
只用于验证数据单位、完整性和错误处理。真实设备性能验收由
`scripts/verify_orbbec_cameras.py` 和训练数据网页完成。
