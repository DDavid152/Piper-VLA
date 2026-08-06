#!/usr/bin/env bash

set -o errexit
set -o nounset
set -o pipefail

PROJECT_DIR=/home/ubuntu22/Piper-VLA
CONDA_SCRIPT=/home/ubuntu22/miniforge3/etc/profile.d/conda.sh
CHECKPOINT_PATH="$PROJECT_DIR/outputs/train/smolvla_purple_bag_b32_10k_v1/checkpoints/004000/pretrained_model"
CALIBRATION_PATH="$PROJECT_DIR/config/piper_active_calibration_v2.json"
SAFETY_BASELINE_PATH="$PROJECT_DIR/config/piper_safety_baseline_v1.json"
CAN_INTERFACE=can0
EXPECTED_ADAPTER_SERIAL=002900225547571120343930
TASK_TEXT='夹住紫色手提袋顶部订合在一起的两根橙黄色提带，将袋子竖直提离桌面约10厘米，保持悬空2秒，再将袋子放回原位，松开提带并将夹爪退离。'
MICRO_PROFILE=full
START_POSE=current
INFERENCE_TYPE=sync
RTC_QUEUE_THRESHOLD=40
RTC_EXECUTION_HORIZON=10
ENFORCE_DISPLACEMENT_WINDOW=true
CONTROL_HZ=10
while [[ $# -gt 0 ]]; do
    case "$1" in
        --profile)
            MICRO_PROFILE="$2"
            shift 2
            ;;
        --start-pose)
            START_POSE="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: $0 [--profile single|full|extended|extended15|rtc15|diagnostic30] [--start-pose current|training]"
            exit 0
            ;;
        *)
            echo "ERROR: unknown argument: $1" >&2
            echo "Usage: $0 [--profile single|full|extended|extended15|rtc15|diagnostic30] [--start-pose current|training]" >&2
            exit 2
            ;;
    esac
done
case "$MICRO_PROFILE" in
    single)
        MAX_ACTIVE_ACTIONS=1
        MAX_MOTION_DURATION_S=0.5
        MOTION_SPEED_PERCENT=5
        ROLLOUT_DURATION_S=8
        MAX_JOINT_DISPLACEMENT_DEG=5
        MAX_GRIPPER_DISPLACEMENT_MM=15
        POLICY_FPS=10
        INTERPOLATION_MULTIPLIER=1
        LOG_RATE_TAG=10hz
        ;;
    full)
        MAX_ACTIVE_ACTIONS=20
        MAX_MOTION_DURATION_S=2.5
        MOTION_SPEED_PERCENT=10
        # The first synchronous SmolVLA action chunk can take about six seconds
        # on this machine. Leave enough rollout time for the 20-action motion
        # budget to run after that initial inference completes.
        ROLLOUT_DURATION_S=12
        MAX_JOINT_DISPLACEMENT_DEG=5
        MAX_GRIPPER_DISPLACEMENT_MM=15
        POLICY_FPS=10
        INTERPOLATION_MULTIPLIER=1
        LOG_RATE_TAG=10hz
        ;;
    extended)
        MAX_ACTIVE_ACTIONS=50
        MAX_MOTION_DURATION_S=5
        MOTION_SPEED_PERCENT=10
        ROLLOUT_DURATION_S=15
        MAX_JOINT_DISPLACEMENT_DEG=10
        MAX_GRIPPER_DISPLACEMENT_MM=15
        POLICY_FPS=10
        INTERPOLATION_MULTIPLIER=1
        LOG_RATE_TAG=10hz
        ;;
    extended15)
        # One 50-step SmolVLA chunk, stretched to ~15 seconds without a
        # second blocking inference: 1 + 49 * 3 = 148 motor commands.
        MAX_ACTIVE_ACTIONS=148
        MAX_MOTION_DURATION_S=15
        MOTION_SPEED_PERCENT=10
        ROLLOUT_DURATION_S=25
        MAX_JOINT_DISPLACEMENT_DEG=10
        MAX_GRIPPER_DISPLACEMENT_MM=15
        POLICY_FPS=3.3333333333333335
        INTERPOLATION_MULTIPLIER=3
        LOG_RATE_TAG=3p33hz_x3
        ;;
    rtc15)
        # Asynchronous RTC predicts and merges overlapping 50-step chunks.
        # A 3.33 Hz policy rate with 3x interpolation keeps motor TX at 10 Hz.
        MAX_ACTIVE_ACTIONS=150
        MAX_MOTION_DURATION_S=15
        MOTION_SPEED_PERCENT=10
        ROLLOUT_DURATION_S=25
        MAX_JOINT_DISPLACEMENT_DEG=10
        MAX_GRIPPER_DISPLACEMENT_MM=15
        POLICY_FPS=3.3333333333333335
        INTERPOLATION_MULTIPLIER=3
        LOG_RATE_TAG=3p33hz_x3
        INFERENCE_TYPE=rtc
        ;;
    diagnostic30)
        # Full diagnostic: 10 Hz RTC policy actions, interpolated to 30 Hz
        # motor commands for 30 seconds, with no start-relative window.
        MAX_ACTIVE_ACTIONS=900
        MAX_MOTION_DURATION_S=30
        MOTION_SPEED_PERCENT=10
        ROLLOUT_DURATION_S=45
        MAX_JOINT_DISPLACEMENT_DEG=10
        MAX_GRIPPER_DISPLACEMENT_MM=15
        POLICY_FPS=10
        INTERPOLATION_MULTIPLIER=3
        CONTROL_HZ=30
        LOG_RATE_TAG=10hz_x3_30hz
        INFERENCE_TYPE=rtc
        RTC_QUEUE_THRESHOLD=49
        ENFORCE_DISPLACEMENT_WINDOW=false
        ;;
    *)
        echo "ERROR: --profile must be single, full, extended, extended15, rtc15, or diagnostic30." >&2
        exit 2
        ;;
esac
case "$START_POSE" in
    current)
        START_POSE_MODE=current_physical
        PREFLIGHT_MODE=current-pose
        PREFLIGHT_ARG=--current-pose-deployment-preflight
        ;;
    training)
        START_POSE_MODE=training_envelope
        PREFLIGHT_MODE=training-envelope
        PREFLIGHT_ARG=--deployment-preflight
        ;;
    *)
        echo "ERROR: --start-pose must be current or training." >&2
        exit 2
        ;;
esac
if [[ "$MICRO_PROFILE" == diagnostic30 ]]; then
    RUN_CONFIRMATION="RUN_30_SECOND_RTC_NO_WINDOW_PIPER_${START_POSE^^}_POSE_TEST"
elif [[ "$MICRO_PROFILE" == rtc15 ]]; then
    RUN_CONFIRMATION="RUN_15_SECOND_RTC_PIPER_${START_POSE^^}_POSE_TEST"
elif [[ "$MICRO_PROFILE" == extended15 ]]; then
    RUN_CONFIRMATION="RUN_15_SECOND_PIPER_${START_POSE^^}_POSE_TEST"
else
    RUN_CONFIRMATION="RUN_${MAX_ACTIVE_ACTIONS}_ACTION_PIPER_${START_POSE^^}_POSE_TEST"
fi
RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$PROJECT_DIR/logs/piper_active_micro"
LOG_PATH="$LOG_DIR/004000_${INFERENCE_TYPE}_${LOG_RATE_TAG}_${MAX_ACTIVE_ACTIONS}actions_$RUN_STAMP.log"
COMMAND_LOG_PATH="$LOG_DIR/004000_${INFERENCE_TYPE}_${LOG_RATE_TAG}_${MAX_ACTIVE_ACTIONS}actions_$RUN_STAMP.commands.jsonl"

if [[ ! -t 0 ]]; then
    echo "ERROR: the active micro test requires an interactive terminal." >&2
    exit 2
fi
if [[ ! -f "$CHECKPOINT_PATH/model.safetensors" ]]; then
    echo "ERROR: checkpoint model not found: $CHECKPOINT_PATH" >&2
    exit 2
fi
if [[ ! -f "$CALIBRATION_PATH" ]]; then
    echo "ERROR: verified v2 calibration not found: $CALIBRATION_PATH" >&2
    exit 2
fi

# shellcheck disable=SC1090
source "$CONDA_SCRIPT"
conda activate lerobot
cd "$PROJECT_DIR"

export HF_HUB_OFFLINE=1
export TRANSFORMERS_OFFLINE=1
export PYTHONUNBUFFERED=1

python scripts/commission_piper_calibration.py \
    --preflight-only \
    "$PREFLIGHT_ARG" \
    --can-interface "$CAN_INTERFACE" \
    --expected-adapter-serial "$EXPECTED_ADAPTER_SERIAL" \
    --safety-baseline "$SAFETY_BASELINE_PATH"

echo
echo "WARNING: the follower may move after policy loading and the first valid action."
echo "The master must be powered off or physically isolated from the shared CAN bus."
echo "Keep a tested physical emergency stop in hand."
echo "Policy loading can take 20-30 seconds; do not press Ctrl+C while it loads."
echo "For an emergency, press the physical stop first, then press Ctrl+C once and wait."
echo "Start pose: $PREFLIGHT_MODE; the measured pose becomes the displacement anchor."
if [[ "$ENFORCE_DISPLACEMENT_WINDOW" == true ]]; then
    WINDOW_DESCRIPTION="joint +/-${MAX_JOINT_DISPLACEMENT_DEG}deg; gripper +/-${MAX_GRIPPER_DISPLACEMENT_MM}mm"
else
    WINDOW_DESCRIPTION="disabled (physical limits and p99 slew limits remain active)"
fi
echo "Micro profile: $MICRO_PROFILE; actions=$MAX_ACTIVE_ACTIONS; motion limit=${MAX_MOTION_DURATION_S}s; speed=${MOTION_SPEED_PERCENT}%; displacement window=${WINDOW_DESCRIPTION}; policy fps=${POLICY_FPS}; interpolation=${INTERPOLATION_MULTIPLIER}x; control=${CONTROL_HZ}Hz; rollout=${ROLLOUT_DURATION_S}s."
echo "Inference: $INFERENCE_TYPE; RTC queue threshold=$RTC_QUEUE_THRESHOLD; RTC execution horizon=$RTC_EXECUTION_HORIZON."
read -r -p "Type $RUN_CONFIRMATION to continue: " typed_confirmation
if [[ "$typed_confirmation" != "$RUN_CONFIRMATION" ]]; then
    echo "Cancelled; lerobot-rollout was not started."
    exit 2
fi

mkdir -p "$LOG_DIR"
if [[ -e "$LOG_PATH" ]]; then
    echo "ERROR: refusing to overwrite log: $LOG_PATH" >&2
    exit 2
fi
if [[ -e "$COMMAND_LOG_PATH" ]]; then
    echo "ERROR: refusing to overwrite command log: $COMMAND_LOG_PATH" >&2
    exit 2
fi

set +o errexit
INFERENCE_ARGS=(--inference.type="$INFERENCE_TYPE")
if [[ "$INFERENCE_TYPE" == rtc ]]; then
    INFERENCE_ARGS+=(
        --inference.queue_threshold="$RTC_QUEUE_THRESHOLD"
        --inference.rtc.execution_horizon="$RTC_EXECUTION_HORIZON"
    )
fi
lerobot-rollout \
    --strategy.type=base \
    "${INFERENCE_ARGS[@]}" \
    --policy.path="$CHECKPOINT_PATH" \
    --robot.type=piper_active \
    --robot.id=piper_follower \
    --robot.can_interface="$CAN_INTERFACE" \
    --robot.expected_adapter_serial="$EXPECTED_ADAPTER_SERIAL" \
    --robot.control_chain=direct_sdk_follower \
    --robot.motion_enabled=true \
    --robot.arm_on_first_action=true \
    --robot.operator_confirmation=I_UNDERSTAND_PIPER_WILL_MOVE \
    --robot.start_pose_mode="$START_POSE_MODE" \
    --robot.safety_profile=micro_observe \
    --robot.max_active_actions="$MAX_ACTIVE_ACTIONS" \
    --robot.max_motion_duration_s="$MAX_MOTION_DURATION_S" \
    --robot.max_joint_displacement_deg="$MAX_JOINT_DISPLACEMENT_DEG" \
    --robot.max_gripper_displacement_mm="$MAX_GRIPPER_DISPLACEMENT_MM" \
    --robot.enforce_displacement_window="$ENFORCE_DISPLACEMENT_WINDOW" \
    --robot.motion_speed_percent="$MOTION_SPEED_PERCENT" \
    --robot.safety_baseline_path="$SAFETY_BASELINE_PATH" \
    --robot.calibration_path="$CALIBRATION_PATH" \
    --robot.active_command_log_path="$COMMAND_LOG_PATH" \
    --robot.cameras="{front: {type: orbbec, serial_number: CP28563000XR, width: 640, height: 480, fps: 30, color_mode: rgb, use_depth: false, timeout_ms: 1000, warmup_s: 2.0}, wrist: {type: orbbec, serial_number: CP28563000XP, width: 640, height: 480, fps: 30, color_mode: rgb, use_depth: false, timeout_ms: 1000, warmup_s: 2.0}}" \
    --task="$TASK_TEXT" \
    --fps="$POLICY_FPS" \
    --interpolation_multiplier="$INTERPOLATION_MULTIPLIER" \
    --duration="$ROLLOUT_DURATION_S" \
    --device=cuda \
    --return_to_initial_position=false \
    --play_sounds=false \
    --display_data=false \
    2>&1 | tee "$LOG_PATH"
rollout_status="${PIPESTATUS[0]}"
set -o errexit

echo "lerobot-rollout exit status: $rollout_status"
echo "Log: $LOG_PATH"
echo "Motor commands: $COMMAND_LOG_PATH"
exit "$rollout_status"
