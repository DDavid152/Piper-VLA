# 脚本目录

## 环境安装与维护

| 文件 | 权限 | 作用 |
|---|---|---|
| `install_miniforge.sh` | 普通用户 | 下载并校验 Miniforge 26.3.2-3 |
| `install_system_dependencies.sh` | root | 安装编译、Git/LFS、FFmpeg、CAN、USB 和视频工具 |
| `install_python_environment.sh` | 普通用户 | 创建 `lerobot`，安装固定 Torch/LeRobot/硬件依赖和三个插件 |
| `install_orbbec_udev.sh` | root | 下载、校验并安装 Orbbec 2.0.18 udev 规则 |
| `verify_environment.sh` | 普通用户 | 验证依赖、CUDA、CLI、视频、插件、测试和相机只读枚举 |
| `freeze_environment.sh` | 普通用户 | 刷新 Conda/Pip 锁文件和 Pip 审计信息 |

安装脚本不安装系统 CUDA Toolkit、不修改 NVIDIA 驱动，也不配置 CAN。

## `verify_orbbec_cameras.py`

对一台或两台配置相机做并发、定时、只读验收并生成 JSON/快照。

主要函数：

- `parse_args()`：解析时长、最低 FPS、深度诊断和输出路径；
- `load_config()`：将 `config/cameras.json` 转为 Orbbec 配置；
- `consume_camera()`：消费帧并统计帧数、时间和像素摘要；
- `stats_delta()`：计算健康计数器增量；
- `main()`：枚举、并发连接、运行检查、判定通过并安全释放设备。

默认不会升级固件。`--include-depth` 只用于带宽诊断，不修改正式 RGB 配置。

## `view_piper_dataset_dashboard.py`

在本机 HTTP 服务中实时展示训练样本需要检查的数据。

主要类和函数：

- `PiperDataSource`：加载共享 CAN 身份，连接只读 Robot/Teleoperator，
  以约 30 FPS 保存 observation、action、差值和 TX 计数；
- `StreamBuffer`：保存每路相机最新 JPEG；
- `ViewerState`：汇总相机和 Piper 健康状态；
- `build_index_html()`：生成双/单相机和 7 维数据表页面；
- `ViewerRequestHandler`：提供 `/`、`/status.json` 和 MJPEG；
- `load_camera_configs()`、`select_connected_configs()`：按序列号自动适配
  一台或两台相机；
- `main()`：连接资源、启动服务并在退出时释放。

关键选项：

```text
--task TEXT        页面显示的任务指令
--camera NAME      仅选择指定相机，可重复
--camera-only      完全跳过 CAN
--require-piper    Piper 不可用时直接退出
--no-browser       不自动打开浏览器
```

该网页不会启动 CAN。与 `lerobot-record` 同时运行会争用相机，启动录制前
必须先按 `Ctrl+C` 退出网页。

启动实机网页时固定采用：follower 先上电并等待约 5 秒，master 后上电并
再次等待约 5 秒，随后依次执行：

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details -statistics link show can0
python scripts/view_piper_dataset_dashboard.py \
  --task "准确、固定的任务指令"
```

应先确认 CAN 为 `UP`、`ERROR-ACTIVE`、1 Mbps 且总线错误为 0。Piper 连接
只在网页启动时尝试一次；若网页早于 CAN 启动，必须按 `Ctrl+C` 退出后重启。
完整操作与验收标准见 [RECORDING_GUIDE.md](../RECORDING_GUIDE.md)。

## `verify_piper_dataset.py`

对 Piper 数据集中的全部 episode 做一次批量自动 QA，不限制 episode 数量。
它理解 LeRobot 会把多条 episode 拼接进同一个物理 MP4，并根据 episode
元数据中的 `from_timestamp` / `to_timestamp` 检查各自的视频区间。

默认检查手动采集数据集：

```bash
python scripts/verify_piper_dataset.py \
  --output logs/piper_manual_batch_qa.json
```

检查内容包括：

- 数据集、episode、帧和任务索引连续性；
- 每条 episode 的 7 维 `observation.state` / `action` 形状、有限值和变化量；
- 每条 episode 内的时间戳、30 FPS 节奏及最低有效帧率；
- 两路共享 MP4 的完整解码、分辨率、帧率、时间戳和 episode 帧区间；
- 所有 episode 区间是否连续、是否完整覆盖对应物理 MP4；
- 是否存在未被元数据引用的 MP4。

返回码为 `0=全部自动检查通过`、`1=QA 已完成但有失败项`、
`2=数据集缺失或工具无法运行`。JSON 中的 `failed_episode_indices` 可直接
定位需要排除或重录的条目。自动 QA 不能判断是否真的夹对提带、提起约
10 厘米或悬停 2 秒，这些仍需逐条查看视频。
