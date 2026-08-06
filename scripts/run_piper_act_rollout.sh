#!/usr/bin/env bash

set -o errexit
set -o nounset
set -o pipefail

PROJECT_DIR=/home/ubuntu22/Piper-VLA
CONDA_SCRIPT=/home/ubuntu22/miniforge3/etc/profile.d/conda.sh
CHECKPOINT_PATH="$PROJECT_DIR/outputs/train/act_purple_bag_b8_100k_v1/checkpoints/070000/pretrained_model"
CALIBRATION_PATH="$PROJECT_DIR/config/piper_active_calibration_v2.json"
SAFETY_BASELINE_PATH="$PROJECT_DIR/config/piper_safety_baseline_v1.json"
CAN_INTERFACE=can0
EXPECTED_ADAPTER_SERIAL=002900225547571120343930
TASK_TEXT='夹住紫色手提袋顶部订合在一起的两根橙黄色提带，将袋子竖直提离桌面约10厘米，保持悬空2秒，再将袋子放回原位，松开提带并将夹爪退离。'
PROFILE=50

usage() {
    echo "Usage: $0 [--profile 50|30s] [--checkpoint PATH]"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --profile)
            PROFILE="$2"
            shift 2
            ;;
        --checkpoint)
            CHECKPOINT_PATH="$2"
            shift 2
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            echo "ERROR: unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

case "$PROFILE" in
    50)
        MAX_ACTIVE_ACTIONS=50
        MAX_MOTION_DURATION_S=2
        ROLLOUT_DURATION_S=10
        ENFORCE_DISPLACEMENT_WINDOW=true
        RUN_CONFIRMATION=RUN_ACT_50_ACTIONS
        ;;
    30s)
        MAX_ACTIVE_ACTIONS=900
        MAX_MOTION_DURATION_S=30
        ROLLOUT_DURATION_S=45
        ENFORCE_DISPLACEMENT_WINDOW=false
        RUN_CONFIRMATION=RUN_ACT_30_SECONDS
        ;;
    *)
        echo "ERROR: --profile must be 50 or 30s." >&2
        exit 2
        ;;
esac

if [[ ! -t 0 ]]; then
    echo "ERROR: ACT active rollout requires an interactive terminal." >&2
    exit 2
fi
if [[ ! -f "$CHECKPOINT_PATH/model.safetensors" ]]; then
    echo "ERROR: checkpoint model not found: $CHECKPOINT_PATH" >&2
    exit 2
fi
if [[ ! -f "$CALIBRATION_PATH" ]]; then
    echo "ERROR: verified calibration not found: $CALIBRATION_PATH" >&2
    exit 2
fi
if [[ ! -f "$SAFETY_BASELINE_PATH" ]]; then
    echo "ERROR: safety baseline not found: $SAFETY_BASELINE_PATH" >&2
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
    --current-pose-deployment-preflight \
    --can-interface "$CAN_INTERFACE" \
    --expected-adapter-serial "$EXPECTED_ADAPTER_SERIAL" \
    --safety-baseline "$SAFETY_BASELINE_PATH"

echo
echo "WARNING: the follower will move after policy loading and the first valid action."
echo "The master must be powered off or physically isolated from the shared CAN bus."
echo "Keep a tested physical emergency stop in hand and keep the workspace clear."
echo "Profile=$PROFILE; max actions=$MAX_ACTIVE_ACTIONS; motion limit=${MAX_MOTION_DURATION_S}s."
echo "Start pose=current physical; speed=10%; policy/action fps=30; interpolation=1x."
read -r -p "Type $RUN_CONFIRMATION to continue: " typed_confirmation
if [[ "$typed_confirmation" != "$RUN_CONFIRMATION" ]]; then
    echo "Cancelled; lerobot-rollout was not started."
    exit 2
fi

RUN_STAMP="$(date +%Y%m%d_%H%M%S_%N)_$$"
CHECKPOINT_LABEL="$(basename "$(dirname "$CHECKPOINT_PATH")")"
LOG_DIR="$PROJECT_DIR/logs/piper_active_micro"
LOG_PATH="$LOG_DIR/act_${CHECKPOINT_LABEL}_sync_30hz_${PROFILE}_${RUN_STAMP}.log"
COMMAND_LOG_PATH="$LOG_DIR/act_${CHECKPOINT_LABEL}_sync_30hz_${PROFILE}_${RUN_STAMP}.commands.jsonl"
mkdir -p "$LOG_DIR"

if [[ -e "$LOG_PATH" || -e "$COMMAND_LOG_PATH" ]]; then
    echo "ERROR: refusing to overwrite an existing rollout log." >&2
    exit 2
fi

set +o errexit
lerobot-rollout \
    --strategy.type=base \
    --inference.type=sync \
    --policy.path="$CHECKPOINT_PATH" \
    --robot.type=piper_active \
    --robot.id=piper_follower \
    --robot.can_interface="$CAN_INTERFACE" \
    --robot.expected_adapter_serial="$EXPECTED_ADAPTER_SERIAL" \
    --robot.control_chain=direct_sdk_follower \
    --robot.motion_enabled=true \
    --robot.arm_on_first_action=true \
    --robot.operator_confirmation=I_UNDERSTAND_PIPER_WILL_MOVE \
    --robot.start_pose_mode=current_physical \
    --robot.safety_profile=micro_observe \
    --robot.max_active_actions="$MAX_ACTIVE_ACTIONS" \
    --robot.max_motion_duration_s="$MAX_MOTION_DURATION_S" \
    --robot.max_joint_displacement_deg=10 \
    --robot.max_gripper_displacement_mm=15 \
    --robot.enforce_displacement_window="$ENFORCE_DISPLACEMENT_WINDOW" \
    --robot.motion_speed_percent=10 \
    --robot.safety_baseline_path="$SAFETY_BASELINE_PATH" \
    --robot.calibration_path="$CALIBRATION_PATH" \
    --robot.active_command_log_path="$COMMAND_LOG_PATH" \
    --robot.cameras="{front: {type: orbbec, serial_number: CP28563000XR, width: 640, height: 480, fps: 30, color_mode: rgb, use_depth: false, timeout_ms: 1000, warmup_s: 2.0}, wrist: {type: orbbec, serial_number: CP28563000XP, width: 640, height: 480, fps: 30, color_mode: rgb, use_depth: false, timeout_ms: 1000, warmup_s: 2.0}}" \
    --task="$TASK_TEXT" \
    --fps=30 \
    --interpolation_multiplier=1 \
    --duration="$ROLLOUT_DURATION_S" \
    --device=cuda \
    --return_to_initial_position=false \
    --play_sounds=false \
    --display_data=false \
    2>&1 | tee "$LOG_PATH"
rollout_status="${PIPESTATUS[0]}"
set -o errexit

echo "lerobot-rollout exit status: $rollout_status"
echo "Runtime log: $LOG_PATH"
echo "Command log: $COMMAND_LOG_PATH"
echo "Before another run: inspect the arm, run recover_piper_emergency_stop.py, and reset the task by hand."
exit "$rollout_status"
