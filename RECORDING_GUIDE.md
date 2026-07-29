# 实机录制指南

当前状态：双 Orbbec 相机、只读 Piper Robot 插件、只读 Piper Master
Teleoperator 插件和训练数据实时网页均已实现。YAML 录制模板可被 LeRobot
完整解析；在完成一次共享 CAN 实机被动数据网页验收和现场安全确认前，仍不
启动正式数据集录制。

软件环境激活方式：

```bash
source /home/ubuntu22/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
```

## 双相机固定映射

配置文件：`config/cameras.json`

| 数据键 | 序列号 | 物理位置 | 正式采集格式 |
|---|---|---|---|
| `front` | `CP28563000XR` | 远程固定相机 | RGB 640×480@30 |
| `wrist` | `CP28563000XP` | 机械臂末端相机 | RGB 640×480@30 |

设备始终按序列号选择，不使用 `/dev/video*` 作为身份。第一阶段 SmolVLA
数据集关闭深度流。

## 实时查看全部训练数据

网页展示一个 LeRobot 训练样本需要检查的全部输入：

- `observation.images.front` 与 `observation.images.wrist`；
- follower 的 `observation.state[7]`：J1～J6 角度及夹爪位置；
- master 的 `action[7]` 及逐项 `action - observation`；
- `task` 指令；
- 相机与 CAN 的帧率、数据年龄、异常计数和本程序 CAN TX 增量；
- 录制时由 LeRobot 生成的 `timestamp`、`frame_index`、`episode_index`
  字段提示。

机械臂读取器要求共享总线的 `can0` 已由操作者配置为 1 Mbps 且为 UP。网页
不会配置或启动 CAN，不会使能、回零或移动机械臂，也没有 CAN 发送路径。
只有在工作区清空、急停可用、两臂按官方主从方式连接并由现场人员确认安全
后，才能准备 CAN：

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details -statistics link show can0
```

确保没有其他程序占用相机，然后运行：

```bash
cd /home/ubuntu22/Piper-VLA
python scripts/view_piper_dataset_dashboard.py \
  --task "将物块放入目标区域"
```

程序会自动打开 `http://127.0.0.1:8765/`，并每秒刷新硬件采集帧率、累计
帧数、最新帧年龄、关节角、夹爪、master 动作和 CAN 健康状态。运行时会按
序列号检查设备：连接一台时显示单画面，连接两台时自动显示 `front` 和
`wrist` 双画面，未连接的配置相机只提示并跳过。浏览器显示默认限制为每路
15 FPS，以降低 JPEG 编码和浏览器开销；相机后台仍按配置的 30 FPS 采集。

正常的机械臂面板应满足：

- 7 项 follower 数据持续更新，J1～J6 单位为度，夹爪单位为毫米；
- master 被拖动时 `Master action` 更新；静止且本轮尚无 target 帧时，
  动作来源显示“follower 保持值”；
- 数据年龄通常低于 250 ms，采样接近 30 FPS；
- “本程序新增 TX”始终为 0；若非 0，立即停止并排查；
- 页面显示的 USB-CAN 必须对应序列号
  `002900225547571120343930`。

如果浏览器没有自动打开，可手动访问上述地址，或使用：

```bash
python scripts/view_piper_dataset_dashboard.py --no-browser
```

CAN 未启动或机械臂未连接时，相机网页仍会运行，并在机械臂区域明确显示
失败原因。纯相机检查可完全跳过 CAN：

```bash
python scripts/view_piper_dataset_dashboard.py --camera-only
```

两台相机都连接时，也可以只测试其中一路；该模式仍可展示机械臂数据：

```bash
python scripts/view_piper_dataset_dashboard.py --camera front
python scripts/view_piper_dataset_dashboard.py --camera wrist
```

结束时在启动程序的终端按 `Ctrl+C`。正常退出会停止后台线程并释放所有已
打开的相机。

## 录制接口与模板

- Piper 配置：`config/piper_native_master_slave.json`；
- LeRobot 示例模板：`config/record_piper_native.example.yaml`；
- Robot 类型：`piper`；
- Teleoperator 类型：`piper_master`。

模板默认只录制到本机、不登录 Hugging Face、不上传数据，任务文本仍是
`REPLACE_WITH_THE_EXACT_TASK_INSTRUCTION`，正式使用前必须替换。软件接口
的 `send_action()` 仅校验记录到的数据并原样返回，不会向 follower 发送。

## 采集前相机复验

确保没有其他程序占用相机，然后运行：

```bash
cd /home/ubuntu22/Piper-VLA
python scripts/verify_orbbec_cameras.py \
  --duration-s 30 \
  --minimum-fps 28 \
  --output logs/orbbec_dual_rgb_latest.json \
  --snapshot-dir logs/orbbec_dual_rgb_latest_snapshots
```

通过标准：

- JSON 顶层 `passed` 为 `true`；
- `front` 和 `wrist` 均不低于 28 FPS；
- 丢帧、重复帧、时间戳回退、超时、读取失败和坏帧全部为 0；
- 两张快照清晰、方向正确，`front` 覆盖完整任务区和直接接触过程；
- 当前固定结构下，`wrist` 应覆盖末端前方任务区，但不会包含夹爪尖端；
  任务、目标物和主要操作轨迹必须布置在 `front` 与 `wrist` 的重叠视野内；
- 正式数据不得包含无关人员，任务区中的纸箱、线缆等无关遮挡物应移除；
- 快照不可用时必须调整物理安装后重测，禁止以黑帧、复制旧帧或错误相机
  替代。

仅诊断 USB 带宽时可追加 `--include-depth`。这不会修改
`config/cameras.json`，也不代表正式数据集启用深度。

## 当前禁止事项

- 共享 CAN 实时网页尚未在本轮重新通过前，不运行面向正式数据集的
  `lerobot-record`；
- 不升级相机固件；
- 不通过软件重新向 follower 发送运动目标，保留已验证的官方原生主从链路。
