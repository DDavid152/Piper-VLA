# LeRobot 插件

四个项目内插件均以 editable 模式安装：

| 包 | LeRobot 类型 | 作用 |
|---|---|---|
| `lerobot_camera_orbbec` | Camera `orbbec` | 按序列号读取 Gemini 335L |
| `lerobot_robot_piper` | Robot `piper` | 被动读取 follower 和相机 |
| `lerobot_teleoperator_piper` | Teleoperator `piper_master` | 被动解析 master 目标 |
| `lerobot_robot_piper_active` | Robot `piper_active` | 通过官方 SDK 受控发送模型动作 |

数据接口固定为 J1～J6（度）和夹爪（毫米），录制后形成 7 维
`observation.state`、7 维 `action` 与 `observation.images.{front,wrist}`。

## 边界

- 被动 Robot 与 Teleoperator 打开 CAN 前核对 USB-CAN 序列号，不发送报文。
- active 插件默认 `motion_enabled=false`；真实运动还要求 v2 标定、双相机、
  新鲜反馈、操作员确认、限位/限速、动作与时间预算。
- active 只允许官方 SDK 的使能、模式、关节、夹爪和急停调用；不做 raw CAN、
  角色重配置、设零、reset、自动恢复或自动回位。
- 数据采集时 master 开启并共享总线；模型部署时 master 必须关闭或物理隔离。

当前 v2 标定和主动链已通过实机验收。ACT 与 SmolVLA 的推理调度不同，但都经过
同一个 active 安全层；部署入口见 [ACT 指南](../ACT_GUIDE.md) 和
[SmolVLA 指南](../SMOLVLA_GUIDE.md)。
