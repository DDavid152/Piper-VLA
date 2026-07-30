# LeRobot Orbbec Camera 插件

该包为 LeRobot v0.6.0 注册 Camera 类型 `orbbec`，面向 Orbbec SDK v2。
设备按不可变序列号选择，不依赖 `/dev/video*` 枚举顺序。

项目配置：

- `front`：Gemini 335L `CP28563000XR`，远程固定视角；
- `wrist`：Gemini 335L `CP28563000XP`，末端视角；
- 正式格式：RGB 640×480@30 FPS；
- 第一阶段数据集关闭深度。

## 文件与接口

### `pyproject.toml`

声明包 `lerobot_camera_orbbec==0.1.0`，依赖 LeRobot、NumPy 和
`pyorbbecsdk2==2.0.18`。

### `lerobot_camera_orbbec/__init__.py`

稳定导出：

```python
OrbbecCamera
OrbbecCameraConfig
```

### `configuration_orbbec.py`

`OrbbecCameraConfig` 继承 LeRobot `CameraConfig` 并注册类型 `orbbec`。
配置包含序列号、宽高、FPS、颜色模式、深度开关、SDK 超时和预热时间；
`__post_init__()` 拒绝空序列号和无效时间参数。

### `camera_orbbec.py`

`OrbbecCamera` 实现 LeRobot Camera 接口。

主要方法：

- `find_cameras()`：只读枚举设备身份、固件和 USB 信息；
- `connect()`：按序列号选机、选择视频 profile、启动后台采集并预热；
- `_read_loop()`：持续获取 frameset、解码 RGB/深度并维护健康统计；
- `read()` / `async_read()`：返回下一张 RGB 帧；
- `read_latest()`：返回不超过指定年龄的最新 RGB 帧；
- `read_depth()` / `read_latest_depth()`：仅在启用深度时返回 `uint16`；
- `get_last_frame_metadata()`：返回帧号和硬件/系统时间戳；
- `get_health_stats()`：返回帧数、超时、坏帧、帧号间断和时间戳回退；
- `disconnect()`：停止线程、关闭 pipeline 并释放设备。

内部 `_select_video_profile()`、`_decode_color()`、`_decode_depth()` 和
`_update_frame_stats()` 分别负责 profile 匹配、格式转换和连续性检查。

## 安装与验证

```bash
source /home/ubuntu22/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
python -m pip install --editable \
  /home/ubuntu22/Piper-VLA/plugins/lerobot_camera_orbbec
python -m unittest discover -s tests -p 'test_orbbec_camera_plugin.py' -v
```

深度 API 只用于诊断；除非显式升级数据集 schema，正式录制必须保持
`use_depth=false`。
