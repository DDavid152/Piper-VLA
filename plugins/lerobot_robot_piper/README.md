# LeRobot Piper Robot 插件

该包注册 Robot 类型 `piper`，用于官方原生 master/follower 共享 CAN
录制。实体遥操作由两台 Piper 自身完成，插件只读取 follower 反馈和相机。

状态与动作字段固定为：

```text
joint_1.pos ... joint_6.pos, gripper.pos
```

关节单位为度，夹爪单位为毫米。

## 文件与接口

### `pyproject.toml`

声明 `lerobot_robot_piper==0.1.0`，要求 Python 3.12、LeRobot 和
`piper-sdk==0.6.1`。

### `lerobot_robot_piper/__init__.py`

稳定导出：

```python
PiperRobot
PiperRobotConfig
```

### `configuration_piper.py`

`PiperRobotConfig` 继承 LeRobot `RobotConfig` 并注册类型 `piper`。
配置项包括 CAN 接口、预期 USB-CAN 序列号、相机字典、反馈最大年龄、最低
频率和状态安全范围。`__post_init__()` 只接受
`native_master_slave`，并校验所有阈值。

### `robot_piper.py`

`PiperRobot` 实现 LeRobot Robot 接口。

主要函数和方法：

- `read_usb_serial_for_network_interface()`：沿 sysfs 查找 CAN 背后的 USB
  序列号；
- `observation_features`：声明 7 维状态和配置相机图像；
- `action_features`：声明相同的 7 维动作；
- `connect()`：核对适配器身份，以 `piper_init=False` 打开 SDK 读线程，
  等待健康反馈后连接相机；
- `_read_arm_state()`：读取状态、六关节与夹爪并转换为度/毫米；
- `_validate_wrapper()`：检查时间戳年龄和反馈频率；
- `_read_arm_state_with_recovery()`：主机编码器启动造成瞬时调度停顿时，
  等待 SDK 恢复并只返回重新通过年龄校验的新鲜状态；
- `_validate_values()`：检查特征集合、有限数值和安全范围；
- `get_observation()`：组合 follower 状态及相机图像；
- `send_action()`：仅验证并原样返回 action，不发送 CAN；
- `disconnect()`：关闭相机和 SDK 接收线程。

`calibrate()` 和 `configure()` 是明确的非运动操作，不执行软件校准、使能、
回零或角色设置。

## 安全保证

- CAN 接口必须对应配置的共享总线序列号；
- `connect()` 不发送 SDK 初始化查询；
- 代码没有 `JointCtrl`、`GripperCtrl`、`MotionCtrl` 或使能路径；
- 单帧最大年龄仍由 `max_state_age_s` 严格限制；瞬时恢复机制不会返回旧帧，
  持续超过 `state_recovery_timeout_s` 无新鲜反馈时仍停止录制；
- 本插件不实现主动控制。增加主动模式必须另行安全评审。

## 验证

```bash
source /home/ubuntu22/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
python -m unittest discover -s tests -p 'test_piper_plugins.py' -v
```
