# Piper-VLA 项目状态

## 当前阶段

统一软件环境、Piper 官方原生主从遥操链路、双 Orbbec 相机、LeRobot
只读实机插件及训练数据实时网页均已完成并通过联合实机验收。项目已具备
`lerobot-record` 本地采集条件。现已增加命令行手动切分、不限 episode
数量的独立采集模式，并已产生 3 条状态/动作有合理变化的数据。批量 QA
确认 episode 0 和 1 各有一路旧流式视频缺帧，episode 2 的自动结构 QA
通过。流式编码器现已修复并增加保存前完整性保护，等待继续采集新样本并做
动作语义人工验收。

## 已确认

- 主机系统：Ubuntu 22.04.5 LTS，x86_64。
- GPU：NVIDIA RTX 4000 Ada Generation，20 GB。
- NVIDIA 驱动：595.84。
- `lerobot` 环境：Python 3.12.13、LeRobot v0.6.0、
  PyTorch 2.11.0+cu128。
- 软件依赖、Orbbec udev 规则、环境锁文件和验证日志已经落地。
- CUDA、LeRobot CLI、视频编解码及 Orbbec 原生依赖检查通过。
- 两台 USB-CAN 和两台 Gemini 335L 的唯一序列号已经识别。
- 末端相机固定为 `CP28563000XP`，远程相机固定为
  `CP28563000XR`，两台固件均为 1.4.60。
- 两台相机当前均通过 USB 3.2 接入，并固定为数据键
  `wrist=CP28563000XP`、`front=CP28563000XR`。
- 项目内 Orbbec LeRobot 插件 0.1.0 已实现并 editable 安装，支持按序列号
  选机、后台采集、最新帧读取、显式错误和健康统计。
- 项目内 Piper Robot 插件 0.1.0 已实现并 editable 安装，只读取 follower
  的 6 个关节角和夹爪位置；不配置、使能、回零或发送运动命令。
- 项目内 Piper Master Teleoperator 插件 0.1.0 已实现并 editable 安装，
  从共享 CAN 被动重建 master 的 7 维目标；没有 `Bus.send()` 调用。
- 两个 Piper 插件都会核对共享 USB-CAN 序列号
  `002900225547571120343930`，因此不依赖 USB 插入顺序猜测设备身份。
- `lerobot-record` 已发现 `piper` 与 `piper_master` 类型，示例 YAML 可
  解析为双相机 Robot 和 Teleoperator 完整配置。
- 紫色手提袋任务已固定为 1 个、20 秒、30 FPS 的本地试采，task、repo_id、
  独立输出目录、300 ml 固定负载和两根提带夹持方式均已明确；专用配置可由
  LeRobot 完整解析。
- 新增试采自动 QA，可检查 episode/帧数、有效帧率、双路视频、7 维状态/
  动作、task、时间戳和索引；10厘米提起、2秒悬停及放回仍由人工复核。
- 新增 `manual_episode_control`：设备连接后由终端 Enter 明确开始/结束每条
  episode，`r` 丢弃当前条、`q` 丢弃未完成条并退出；手动配置使用
  `num_episodes=0` 持续采集任意数量，固定时长旧模式保持兼容。
- 新增独立的 `piper_purple_bag_two_handle_lift_manual_v1` 数据集配置，
  不覆盖已有的单条试采。配置解析、模拟手动录制闭环及 19 项项目回归测试
  均通过。
- 修复流式视频队列满时静默丢帧的问题：队列改为无丢帧背压等待，持续阻塞
  或编码线程故障会明确中止当前条；保存前严格要求数据行数、编码线程帧数
  和 MP4 可解码帧数完全一致，否则不写入 Parquet。
- 手动与固定时长配置的双相机编码队列均提高到每路 60 帧；1 帧极小队列的
  双相机 80 帧压力测试仍得到两路完整 80 帧。
- 新增 `scripts/verify_piper_dataset.py`，可自动遍历任意实际 episode 数量，
  并理解多条 episode 共用一个 MP4 的 LeRobot 布局；输出逐条结果、
  `failed_episode_indices` 和完整 JSON。
- 当前手动数据集共 3 条、2203 行：episode 0 为 786 行但 wrist 视频区间
  仅 783 帧；episode 1 为 561 行但 front 视频区间仅 559 帧；episode 2
  为 856 行且双路均为 856 帧，自动 QA 通过。报告位于
  `logs/piper_manual_batch_qa.json`。
- 已用 episode 2 完成 ACT 两步离线训练冒烟测试：TorchCodec 训练读取、
  action chunk、CUDA 前向/反向、优化器更新、checkpoint 保存和重载推理
  全部通过；输出位于
  `outputs/train_smoke/act_episode2_smoke_v1/checkpoints/000002`。
- 当前未下载 `lerobot/smolvla_base`，也未开始正式 SmolVLA 微调；一条自动
  QA 合格 episode 只足以验证软件通路，不足以形成可部署策略。
- 修复流式编码器首次启动导致 Piper SDK 反馈短暂变旧的问题：单帧年龄上限
  仍为 0.25 秒，插件只等待新鲜反馈恢复，不返回旧状态；持续 1 秒未恢复
  仍会中止。
- 任务配置改用 H.264 `ultrafast`。同配置的 20 秒临时完整录制成功退出，
  获得 579 帧（28.95 FPS），双路视频均为 640×480/579 帧且可完整解码，
  主机 CAN TX 增量为 0。
- 训练数据实时网页已显示双/单相机画面、7 维 observation、7 维 action、
  差值、task、采样健康状态和 LeRobot 时间/索引字段；CAN 不可用时会保留
  相机功能并明确降级。
- 双相机网页实机验证与单相机降级验证通过；CAN 不可用时可保留相机功能。
- 共享 CAN 联合验收已真实读取 follower observation 和 master action：
  两组 7 维数据约 30 FPS、年龄约 5～30 ms、CAN 错误帧为 0，主机 TX
  增量为 0。
- 两路 RGB 640×480@30 并发 30 秒，各获得 900 个后台帧，帧率 30.0 FPS，
  丢帧、重复帧、时间戳回退、超时、读取失败和坏帧均为 0。
- 两路 RGB+深度并发 10 秒诊断也通过，但正式配置继续关闭深度。
- 相机固定并调整 USB 后已完成第二次 30 秒验收：两台均为 5 Gbps，
  `front` 为 29.97 FPS、`wrist` 为 30.0 FPS，全部错误项为 0。
- `front` 覆盖全局任务区；固定安装的 `wrist` 提供末端前向桌面视角，
  但不包含夹爪尖端，因此任务必须布置在两路视野重叠区。
- 两台 Piper 曾由官方上位机分别配置为 master/leader 和
  follower/slave，最终采用 Piper 官方原生主从控制链。
- follower 已完成单独被动 CAN 和 SDK 验证：反馈为 200 Hz、状态正常、
  无关节通信或角度限位错误，测试期间未发送报文。
- 计划 master 初次单独测试时仍发送普通反馈；其实体臂与 USB-CAN 映射
  已确认，随后经明确授权重新写入 master 角色。
- 经用户明确授权，已对计划 master 单独发送一次官方 `0x470/0xFA` 角色
  配置帧；普通反馈随后停止，未发送任何运动、使能或回零命令。
- master 断电重启后的 5 秒纯监听中 RX/TX 增量均为 0，普通反馈没有恢复，
  说明角色配置没有表现出回退；单臂状态下尚未出现 master 控制帧。
- 共享 CAN 上两次约 8 秒静止纯监听均收到 follower 完整反馈，状态正常、
  主机 TX 为 0 且无总线错误。
- 用户已实际拖动 master 并确认 follower 正常跟随，官方原生遥操作的实机
  功能验证通过。
- 动态纯监听已捕获官方预期的 `0x151`、`0x155`～`0x157`、`0x159`，
  follower 反馈约 200 Hz、状态正常、总线零错误且主机 TX 为 0；协议验收
  同步通过。
- 项目不包含 LIBERO、MuJoCo 或其他仿真环境。
- 本阶段不下载模型权重、不升级相机固件。

## 当前阻塞

- 两台 Piper 的固件和夹爪状态尚未完整验证。
- 每次试采前仍需重新确认急停、工作空间、设备身份和上电顺序。
- 早期固定时长试采的 578 帧 `observation.state`/`action` 全零，仍为无效
  数据；当前手动数据已确认状态和动作有变化，不再把该旧样本作为反馈链路
  阻塞依据。
- 手动数据的 episode 0 和 1 存在修复前产生的视频缺帧，不能直接用于训练；
  修复不会追溯填补不存在的图像帧。
- episode 2 虽通过自动结构 QA，仍需人工确认夹对两根提带、提起高度、
  2 秒悬停、放回、松开和退离均符合任务。

## 下一步

1. 人工查看 episode 2 的双路视频，确认完整动作语义；episode 0 和 1
   标记为不可训练并安排重录。
2. 按 `SAFETY_CHECKLIST.md` 完成本轮现场确认，并关闭网页释放双相机。
3. 续录时使用 `config/record_piper_purple_bag_lift_manual_v1.yaml` 并
   添加 `--resume=true`：在
   `READY` 按 Enter 开始、完成动作后再按 Enter 结束保存。
4. 每条失败样本在录制中按 `r` 丢弃；完成所需条数后只在 `READY` 按 `q`
   退出。
5. 对全部数据运行 `scripts/verify_piper_dataset.py`，再对自动通过的条目
   做人工语义 QA；只有两类检查均通过的 episode 才进入训练集。
