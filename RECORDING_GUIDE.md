# 实机录制指南

当前状态：双 Orbbec 相机、只读 Piper Robot 插件、只读 Piper Master
Teleoperator 插件和训练数据实时网页均已实现。YAML 录制模板可被 LeRobot
完整解析；共享 CAN 实机网页已验证两路图像、7 维状态/动作约 30 FPS 且
主机 TX 为 0。当前可进入首次 1 个、20 秒的本地试采，正式批量采集仍需
先通过试采 QA。

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

### 每日共享 CAN 启动顺序

系统重启或重新插拔 USB-CAN 后，按以下顺序操作：

1. 完成本轮 [实机安全检查表](SAFETY_CHECKLIST.md)，清空工作区并确认急停。
2. 确认两臂按 Piper 官方原生 master/follower 方式接入共享 CAN。
3. follower 先上电，等待约 5 秒至状态稳定。
4. master 后上电，再等待约 5 秒。
5. 在电脑上配置并启动 `can0`：

   ```bash
   sudo ip link set can0 type can bitrate 1000000
   sudo ip link set can0 up
   ip -details -statistics link show can0
   ```

6. 确认输出包含 `state UP`、`can state ERROR-ACTIVE` 和
   `bitrate 1000000`，且 `bus-errors`、`error-warn`、`error-pass`、
   `bus-off` 均为 0。
7. 最后才启动训练数据网页或 `lerobot-record`。

这三条命令只设置电脑端 SocketCAN 的波特率、启动接口并读取状态，不会改变
机械臂中已保存的主从配置，也不会发送使能、回零或运动数据。在本项目当前
共享 CAN 接线中，电脑端控制器进入 UP 状态是原生遥操作稳定工作的必要条件
之一；dashboard 的只读联合验收中“本程序新增 TX”必须保持为 0。

确保没有其他程序占用相机，然后运行：

```bash
cd /home/ubuntu22/Piper-VLA
python scripts/view_piper_dataset_dashboard.py \
  --task "将物块放入目标区域"
```

Piper 连接只在网页进程启动时尝试一次。如果先启动了网页、后启动 `can0`，
或者运行期间重新插拔了 USB-CAN，仅执行上述 CAN 命令不会让现有网页进程
自动重连；必须在网页终端按 `Ctrl+C`，确认 CAN 正常后重新运行网页。

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
`REPLACE_WITH_THE_EXACT_TASK_INSTRUCTION`，正式使用前必须修改，或通过
`--dataset.single_task` 明确覆盖。软件接口的 `send_action()` 仅校验记录
到的数据并原样返回，不会向 follower 发送。

## 紫色手提袋首次试采

任务专用配置：
`config/record_piper_purple_bag_lift_trial_v1.yaml`。

统一任务文本：

> 夹住紫色手提袋顶部订合在一起的两根橙黄色提带，将袋子竖直提离桌面约
> 10厘米，保持悬空2秒，再将袋子放回原位，松开提带并将夹爪退离。

数据集固定为
`local/piper_purple_bag_two_handle_lift_trial_v1`，只录制到
`datasets/piper_purple_bag_two_handle_lift_trial_v1`，参数为 30 FPS、
20 秒、1 个 episode。双路 RGB 使用 H.264 `ultrafast` 流式编码，避免
SVT-AV1 初始化与 Piper SDK 接收线程发生较长调度竞争。

布置要求：

- 紫色袋子直立放在标记区域，正面朝向 `front`；
- 同一瓶 300 ml 矿泉水直立放在袋底中央；
- 两根橙黄色提带顶端按相同方式订合，订书钉尖锐端不得外露；
- 夹爪接触订合处的软带中部，不直接压在金属订书钉上。

先在实时网页下低速演练一次。确认两路视野、夹持、10厘米提起和2秒悬停
均正常后，在网页终端按 `Ctrl+C` 释放相机，再运行：

```bash
lerobot-record \
  --config_path config/record_piper_purple_bag_lift_trial_v1.yaml
```

编码器第一次接收图像时，终端可能出现一次循环低于 30 FPS 和“等待新鲜
Piper feedback”的警告；插件仍坚持单帧年龄不超过 0.25 秒，不会保存旧
状态，只会等待最多 1 秒让 SDK 接收线程恢复。出现随后
`Piper feedback recovered` 属于已验收的启动瞬态；若未恢复并退出，则禁止
反复运行，应先保留完整报错并处理零帧残留目录。

可靠的交互控制键为：`n` 提前完成当前阶段、`r` 放弃并重录当前 episode、
`q` 停止录制。出现滑脱、倾倒、错误夹持、人员进入画面或安全风险时立即
停止，不保留该 episode。

录制结束后执行自动结构验收：

```bash
python scripts/verify_piper_trial_dataset.py \
  --output logs/piper_purple_bag_trial_v1_qa.json
```

再用 LeRobot 可视化工具完成动作语义人工验收：

```bash
lerobot-dataset-viz \
  --repo-id local/piper_purple_bag_two_handle_lift_trial_v1 \
  --root /home/ubuntu22/Piper-VLA/datasets/piper_purple_bag_two_handle_lift_trial_v1 \
  --episode-index 0
```

自动验收必须确认 1 个 episode、560～620 帧、有效频率不低于 28 FPS、
两路 640×480 视频可解码、7 维状态/动作有限且有变化、task 与索引正确。
人工验收必须确认夹取对象、约 10 厘米高度、2 秒悬停、放回、松开和退离。
正式批量采集只能在本条试采通过后规划。

## 命令行手动控制多条采集

不希望用固定时长切分 episode 时，使用
`config/record_piper_purple_bag_lift_manual_v1.yaml`。该配置启用
`manual_episode_control: true`，并用 `num_episodes: 0` 表示本次运行不限制
episode 数量；`episode_time_s` 与 `reset_time_s` 在此模式下不会生效。

首次创建这个独立数据集时运行：

```bash
lerobot-record \
  --config_path config/record_piper_purple_bag_lift_manual_v1.yaml
```

设备连接完成后，终端会依次给出双语状态提示：

- `READY`：数据尚未写入，可不限时布置场景；按 Enter（或 `s`）开始；
- `RECORDING STARTED`：正在采集；完成动作后按 Enter（或 `s`）结束并保存；
- `RECORDING ENDED`：已停止采帧，正在保存；
- `EPISODE SAVED`：该条已落盘，随后回到下一条的 `READY`。

录制期间按 `r` 会丢弃当前未保存 episode 并返回 `READY`；按 `q` 会丢弃
当前未完成 episode 并退出。建议只在 `READY` 状态按 `q` 正常结束整次采集。
硬件安全风险仍必须使用机械臂急停，终端按键不具备硬件急停能力。

以后再次向同一数据集追加时，必须显式启用续录：

```bash
lerobot-record \
  --config_path config/record_piper_purple_bag_lift_manual_v1.yaml \
  --resume=true
```

手动模式要求直接运行在交互式终端中；不能把标准输入重定向或通过管道运行。
原来的定时试采配置未改变，仍可用于固定 20 秒、单 episode 的基准测试。

### 流式编码完整性保护

手动和固定时长配置均把每路编码队列设为 60 帧（30 FPS 下约 2 秒）。
编码器处理不过来时，录制循环会等待队列腾出空间，不再静默丢弃图像；若
持续阻塞超过保护时间或编码线程崩溃，本条 episode 会明确报错并被视为
不完整。

每次保存前还会严格比较以下三个值：

1. 本条 episode 的数据行数；
2. 编码线程实际处理的帧数；
3. 临时 MP4 实际能够完整解码的帧数。

三者必须完全相等才会写入 Parquet。任何一路不相等时，临时视频会被清理，
本条不会进入数据集，避免产生关节数据完整但相机视频缺帧的样本。

### 任意数量 episode 的批量 QA

采集结束后运行：

```bash
python scripts/verify_piper_dataset.py \
  --output logs/piper_manual_batch_qa.json
```

该命令会从元数据自动取得实际 episode 数量并逐条检查，不要求固定为 1 条，
也不要求一条 episode 对应一个 MP4。它会正确解析多条 episode 共用 MP4
时各自的起止帧区间，最后输出总通过数和 `failed_episode_indices`。

返回码：

- `0`：所有自动检查通过；
- `1`：检查正常完成，但至少一个 episode 或全局项目失败；
- `2`：路径、元数据或运行环境错误，QA 未完成。

自动 QA 覆盖结构、数值、时序和视频完整性。夹持对象、约 10 厘米提起、
连续悬停约 2 秒、放回和退离仍需要人工逐条观看视频确认。

截至 2026-07-30，现有手动数据集有 3 条：episode 0 的 wrist 区间比数据
少 3 帧，episode 1 的 front 区间少 2 帧，episode 2 的自动 QA 通过。
完整机器报告保存在 `logs/piper_manual_batch_qa.json`；修复不会追溯改写
旧视频，因此 episode 0 和 1 不得直接用于训练。

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

- 本轮安全检查未完成或模板仍含 task 占位文本时，不运行 `lerobot-record`；
- 训练数据网页未退出时，不启动录制，避免两程序争用相机；
- 不升级相机固件；
- 不通过软件重新向 follower 发送运动目标，保留已验证的官方原生主从链路。
