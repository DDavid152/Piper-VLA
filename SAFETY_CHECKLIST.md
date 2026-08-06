# Piper 实机安全清单

本清单区分“原生遥操作采集”和“模型主动部署”。两种模式不能同时运行。

## 每次共同检查

- [ ] 操作员熟悉物理急停，已现场测试且全程握在手中。
- [ ] 工作区无人员、线缆和障碍物，机械臂/夹爪没有明显损伤。
- [ ] `can0` 为 1 Mbps、`UP`、`ERROR-ACTIVE`，总线错误为 0。
- [ ] USB-CAN 序列号与本轮角色匹配，两台相机序列号和画面正确。
- [ ] 任务物体、相机、桌面和机械臂起始条件与数据采集保持一致。

## 数据采集模式

- [ ] 使用官方 master/follower 共享 CAN；follower 先上电、master 后上电。
- [ ] 只读 `piper` 与 `piper_master` 插件；主机 CAN TX 不应增长。
- [ ] `piper_active` 和所有模型 rollout 均未运行。
- [ ] 录制完成后运行批量自动 QA，再做逐条人工语义验收。

## 模型主动部署模式

- [ ] master 已断电或与 follower 控制总线物理隔离。
- [ ] follower 使用适配器 `002900225547571120343930`。
- [ ] `config/piper_active_calibration_v2.json` 与
  `config/piper_safety_baseline_v1.json` 存在且来源未被手工篡改。
- [ ] 当前物理姿态无碰撞风险，双相机反馈新鲜。
- [ ] 运行脚本的 receive-only current-pose preflight 通过，主机 CAN TX 增量为 0。
- [ ] 先运行 50 动作；只有检查正常后才运行 30 秒完整任务。

```bash
bash scripts/run_piper_act_rollout.sh --profile 50
bash scripts/run_piper_act_rollout.sh --profile 30s
```

SmolVLA 的当前基线入口是 `scripts/run_piper_active_micro.sh`，参数和结论见
[SMOLVLA_GUIDE.md](SMOLVLA_GUIDE.md)。不要把 SmolVLA 的 RTC/插值参数复制到
ACT：ACT 当前为 sync、30 FPS、`interpolation_multiplier=1`。

## 运行中与运行后

- [ ] 手始终放在物理急停上；异常运动先按物理急停，再终止程序。
- [ ] 不在机械臂运动时触碰设备、重新接线或启动第二个控制进程。
- [ ] 每轮检查日志的停止原因、发送动作数和 `fault`。
- [ ] 每轮后检查机械臂和物体，执行恢复命令，再人工重新摆放。

```bash
python scripts/recover_piper_emergency_stop.py
```

恢复命令只清除符合条件的软件急停，不使能、不回零、不发送关节目标。禁止用
shell 循环无人值守重复主动运行；每轮都必须重新通过预检和人工确认。

以下任一情况立即停止：CAN 错误或身份不符、相机冻结、反馈不新鲜、标定拒绝、
异常声音/振动、动作越界、人与障碍物进入工作区，或操作员无法持续控制急停。
