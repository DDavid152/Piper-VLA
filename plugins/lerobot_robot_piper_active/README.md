# Piper Active Robot 插件

`robot.type=piper_active` 是 LeRobot 策略到 Piper follower 的硬件安全边界。
checkpoint 加载、pre/postprocessor、sync/RTC 和动作队列由 LeRobot 0.6.0 提供；
本插件只负责硬件身份、状态、单位转换、限速、发送和急停。

## 动作链

```text
7 维策略输出（度、毫米）
  -> schema / finite / physical-limit 检查
  -> clean 数据 p99 单步限速与可选起点位移窗口
  -> v2 标定映射
  -> Piper SDK 0.001° / 0.001 mm 整数单位
  -> JointCtrl / GripperCtrl
```

`connect()` 只读取 follower、核对适配器、连接 front/wrist 并验证起点，不发送
控制帧。`arm_on_first_action=true` 时，第一条有效动作通过全部检查后才使能。
预算到达、反馈/相机超时、CAN/SDK 异常或安全拒绝会锁存一次急停；进程不会
自动恢复、reset、回零或回初始姿态。

## 关键配置

- `motion_enabled=false`：默认零运动。
- `start_pose_mode=training_envelope`：要求位于 51 条训练首帧包络。
- `start_pose_mode=current_physical`：验证物理限位并以当前反馈作为位移锚点。
- `safety_profile=strict`：任务包络和历史跳变均为硬拒绝。
- `safety_profile=micro_observe`：物理限位仍为硬边界，异常目标先记录/裁剪，再
  应用 p99 限速和启动姿态位移窗口。
- `max_active_actions`、`max_motion_duration_s`：非零时为硬预算。
- `active_command_log_path`：逐帧保存原始目标、限速结果、SDK 目标、反馈与结果。

## v2 标定

`config/piper_active_calibration_v1.json` 故意保持 `verified=false`；active 运动只
接受 schema v2。v2 由至少 20 个只读 master/follower 稳定姿态和六关节各
±0.5° commissioning 生成，并保存两份证据的绝对路径和 SHA-256。插件加载时
重新验证证据，因此不能靠手工改 JSON 绕过门禁。

相关工具：

```bash
python scripts/capture_piper_passive_mapping.py --output logs/piper_calibration/passive_mapping.jsonl
python scripts/commission_piper_calibration.py --preflight-only --current-pose-deployment-preflight
python scripts/recover_piper_emergency_stop.py
```

commissioning 的详细参数使用 `--help` 查看；它会要求额外的操作员确认，并在
结束时保持急停锁存。

## 已验证入口

```bash
# ACT：先 50 动作，再 30 秒
bash scripts/run_piper_act_rollout.sh --profile 50
bash scripts/run_piper_act_rollout.sh --profile 30s

# SmolVLA：当前 RTC 实机基线
bash scripts/run_piper_active_micro.sh --profile diagnostic30 --start-pose current
```

ACT 使用 sync、30 FPS、无额外插值；SmolVLA diagnostic30 使用 RTC、10 Hz
策略动作和 3 倍插值形成约 30 Hz 控制。两者不可混用参数。运行前 master 必须
关闭或物理隔离，并遵循根目录 [安全清单](../../SAFETY_CHECKLIST.md)。

## 测试

```bash
python -m unittest discover -s tests -p 'test_piper_active_plugin.py' -v
python -m unittest discover -s tests -p 'test_piper_calibration_tools.py' -v
```

Fake SDK 测试不接触真实 CAN，覆盖连接零发送、懒武装、单位、限位/限速、
预算、看门狗、异常急停与 v2 证据门禁。
