# LeRobot Piper Master Teleoperator 插件

该包注册 Teleoperator 类型 `piper_master`，在已经运行的官方共享 CAN 上
被动重建 master 目标：

- 关节目标：`0x155`、`0x156`、`0x157`；
- 夹爪目标：`0x159`；
- follower 健康与保持值：`0x2A1`、`0x2A5`～`0x2A8`。

输出为 J1～J6（度）和夹爪（毫米）的 7 维 LeRobot action。

## 文件与接口

### `pyproject.toml`

声明 `lerobot_teleoperator_piper==0.1.0`，要求 Python 3.12、LeRobot 和
`python-can==4.6.1`。

### `lerobot_teleoperator_piper/__init__.py`

稳定导出：

```python
PiperMasterTeleoperator
PiperMasterTeleoperatorConfig
```

### `configuration_piper_master.py`

`PiperMasterTeleoperatorConfig` 注册类型 `piper_master`。配置包含 CAN
接口、USB 序列号、反馈最大年龄、关节目标组合时间窗和数值安全范围；
`__post_init__()` 只接受官方原生主从链路。

### `piper_master_teleoperator.py`

主要函数和方法：

- `read_usb_serial_for_network_interface()`：核对 CAN 适配器身份；
- `decode_signed_32()`：按大端有符号 32 位解析 Piper 协议值；
- `connect()`：打开带 ID 过滤器的 receive-only SocketCAN，并启动接收线程；
- `_receive_loop()`：持续接收但不发送报文；
- `_process_message()`：分发状态、反馈、关节目标和夹爪目标；
- `_publish_coherent_joint_target()`：只在三个关节目标帧位于时间窗内时发布；
- `_fresh_feedback_values()`：取得新鲜 follower 保持状态；
- `_current_action()`：优先使用最后一组完整 master 目标；目标出现前使用
  follower 保持值；
- `_validate_action()`：校验有限值、非全零和安全范围；
- `get_action()`：返回当前 7 维 action；
- `get_health_stats()`：返回接收帧、错误帧、完整目标数和 action 来源；
- `disconnect()`：停止接收线程并关闭 SocketCAN。

`send_feedback()` 为 no-op。包内没有 `Bus.send()` 调用；master 静止时最后一
组完整目标继续作为原生控制器的保持目标。

## 验证

```bash
source /home/ubuntu22/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
python -m unittest discover -s tests -p 'test_piper_plugins.py' -v
```
