# 实机安全检查表

本表必须在每次 dashboard、`lerobot-record`、回放或模型推理前重新确认。
历史测试通过不能替代本轮现场确认。任何一项不满足都应停止，不启动 CAN。

## 现场与人员

- [ ] 操作人员在机械臂旁，可立即断电或触发急停。
- [ ] 急停可触达且功能已由现场人员确认。
- [ ] 两臂工作空间内没有人员、线缆、箱体或其他碰撞物。
- [ ] 台面、目标物和容器固定，任务不会超出安全工作区。
- [ ] 两台相机固定，画面不包含无关人员或隐私信息。

## 设备身份与接线

- [ ] follower USB-CAN 序列号为 `002900225547571120343930`。
- [ ] master 诊断适配器序列号为 `0033002F5547571120343930`。
- [ ] 两臂按 Piper 官方原生 master/follower 方式共享 CAN。
- [ ] follower 先上电，master 后上电。
- [ ] `can0` 为经典 CAN 1 Mbps、`ERROR-ACTIVE`，总线错误为 0。

## 固定启动顺序

以下步骤必须按顺序完成，不能先启动 dashboard 或 `lerobot-record` 再准备
CAN：

1. [ ] follower 已先上电，并等待约 5 秒至状态稳定。
2. [ ] master 已后上电，并再次等待约 5 秒。
3. [ ] 已在电脑上依次执行：

   ```bash
   sudo ip link set can0 type can bitrate 1000000
   sudo ip link set can0 up
   ip -details -statistics link show can0
   ```

4. [ ] 输出包含 `state UP`、`can state ERROR-ACTIVE` 和
   `bitrate 1000000`。
5. [ ] `bus-errors`、`error-warn`、`error-pass`、`bus-off` 均为 0。
6. [ ] 已确认原生遥操作正常，再启动只读 dashboard 或正式采集。

如果 dashboard 曾在 CAN 启动前运行，或 USB-CAN 运行中重新枚举，必须先在
其终端按 `Ctrl+C`，然后在 CAN 状态正常后重新启动；现有进程不会自动重连。

## 软件与数据

- [ ] 已激活 `/home/ubuntu22/miniforge3/envs/lerobot`。
- [ ] 没有其他程序占用两台相机或 CAN 读取器。
- [ ] task、数据集名称和输出目录已确认，模板不含占位文本。
- [ ] 数据集输出目录为空或明确选择了 `resume`，不会覆盖已有 episode。
- [ ] dashboard 联合检查中双路图像和 7 维数据正常、主机 TX 增量为 0。
- [ ] dashboard 已按 `Ctrl+C` 退出并释放相机，再启动 `lerobot-record`。

## 本轮记录

```text
日期时间：
操作人员：
任务指令：
数据集目录：
设备与工作区确认：
异常或停止原因：
```
