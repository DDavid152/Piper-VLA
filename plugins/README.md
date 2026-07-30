# LeRobot 插件

本目录包含 Piper-VLA 自己维护的三个 Python 包，均由
`scripts/install_python_environment.sh` 以 editable 模式安装。

| 插件 | LeRobot 类型 | 作用 |
|---|---|---|
| `lerobot_camera_orbbec` | Camera `orbbec` | 按序列号连接 Orbbec SDK v2 相机 |
| `lerobot_robot_piper` | Robot `piper` | 被动读取 follower 状态和配置相机 |
| `lerobot_teleoperator_piper` | Teleoperator `piper_master` | 被动解析 master 目标 |

## 数据接口

Robot observation：

```text
joint_1.pos ... joint_6.pos, gripper.pos, front RGB, wrist RGB
```

Teleoperator action：

```text
joint_1.pos ... joint_6.pos, gripper.pos
```

J1～J6 使用度，夹爪使用毫米。LeRobot 录制后对应
`observation.state[7]`、`action[7]` 和
`observation.images.{front,wrist}`。

## 安全设计

- 两个 Piper 插件打开 CAN 前核对 USB-CAN 序列号。
- Robot 使用 `piper_init=False`，不执行 SDK 初始化动作。
- Teleoperator 的 SocketCAN 总线仅接收，不调用 `Bus.send()`。
- `send_action()` 仅做数值和 schema 校验，不向 follower 转发。
- 软件校准、配置、使能、回零和主动控制均不在本项目范围内。

每个插件的模块、类和函数说明见其子目录 README。
