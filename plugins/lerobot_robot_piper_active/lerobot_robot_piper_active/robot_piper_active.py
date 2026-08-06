from __future__ import annotations

import json
import logging
import math
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, TextIO

import can
from lerobot.types import RobotAction, RobotObservation
from lerobot_robot_piper import PiperRobot

from .configuration_piper_active import PiperActiveRobotConfig
from .safety import ActiveSafetyError, FEATURES, PiperSafetyProcessor


logger = logging.getLogger(__name__)
MASTER_COMMAND_IDS = frozenset((0x155, 0x156, 0x157, 0x159))


class PiperActiveRobot(PiperRobot):
    """Direct follower controller with a fail-closed, explicitly armed TX path."""

    config_class = PiperActiveRobotConfig
    name = "piper_active"

    def __init__(self, config: PiperActiveRobotConfig):
        super().__init__(config)
        self.config = config
        self.safety = PiperSafetyProcessor(
            config.safety_baseline_path,
            config.calibration_path,
            profile=config.safety_profile,
            start_pose_mode=config.start_pose_mode,
            max_joint_displacement_deg=config.max_joint_displacement_deg,
            max_gripper_displacement_mm=config.max_gripper_displacement_mm,
            enforce_displacement_window=config.enforce_displacement_window,
        )
        self._armed = False
        self._ever_armed = False
        self._fault_latched = False
        self._fault_reason: str | None = None
        self._emergency_stop_sent = False
        self._last_action_monotonic: float | None = None
        self._last_feedback_monotonic: float | None = None
        self._watchdog_stop = threading.Event()
        self._watchdog_thread: threading.Thread | None = None
        # Serializes SDK TX against watchdog/disconnect quick-stop paths.
        self._fault_lock = threading.RLock()
        self._motion_started_monotonic: float | None = None
        self._motion_stopped = False
        self._stop_reason: str | None = None
        self._active_action_count = 0
        self._suppressed_action_count = 0
        self._safety_warning_count = 0
        self._dry_run_rejections = 0
        self._dry_initial_state_ok: bool | None = None
        self._active_command_log: TextIO | None = None
        self._last_sent_calibrated_target: list[float] | None = None

    @property
    def rollout_stop_requested(self) -> bool:
        """Ask the rollout loop to exit after an intentional, non-fault stop."""
        return self._motion_stopped and not self._fault_latched

    @property
    def rollout_stop_reason(self) -> str | None:
        return self._stop_reason if self.rollout_stop_requested else None

    def _open_active_command_log(self) -> None:
        if not self.config.active_command_log_path or self._active_command_log is not None:
            return
        path = Path(self.config.active_command_log_path).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        self._active_command_log = path.open("x", encoding="utf-8", buffering=1)
        logger.info("Piper active command log: %s", path)

    def _close_active_command_log(self) -> None:
        if self._active_command_log is not None:
            self._active_command_log.flush()
            self._active_command_log.close()
            self._active_command_log = None

    def _record_active_command(
        self,
        command: dict[str, Any],
        *,
        result: str,
        sdk_error: str | None = None,
    ) -> None:
        """Persist the exact postprocessed motor target presented to the SDK."""
        feedback_before_command = None
        previous_target_tracking_error = None
        if self._last_observation_state is not None:
            feedback_before_command = [
                float(self._last_observation_state[feature]) for feature in FEATURES
            ]
            if self._last_sent_calibrated_target is not None:
                previous_target_tracking_error = [
                    target - actual
                    for target, actual in zip(
                        self._last_sent_calibrated_target,
                        feedback_before_command,
                        strict=True,
                    )
                ]
        self._open_active_command_log()
        if self._active_command_log is None:
            if result == "sent":
                self._last_sent_calibrated_target = [
                    *command["calibrated_joint_degrees"],
                    command["calibrated_gripper_mm"],
                ]
            return
        record = {
            "schema_version": 1,
            "sequence": self._validated_action_count,
            "wall_time_utc": datetime.now(timezone.utc).isoformat(),
            "result": result,
            "motion_enabled": self.config.motion_enabled,
            "safety_profile": self.config.safety_profile,
            "displacement_window_enabled": self.config.enforce_displacement_window,
            "raw_model_action": command["raw"],
            "physical_clipped_action": command["physical_clipped"],
            "was_physical_clipped": command["was_physical_clipped"],
            "limited_action": command["limited"],
            "was_slew_limited": command["was_slew_limited"],
            "was_displacement_clipped": command["was_displacement_clipped"],
            "warnings": command["warnings"],
            "calibrated_joint_degrees": command["calibrated_joint_degrees"],
            "calibrated_gripper_mm": command["calibrated_gripper_mm"],
            "joint_units_0_001_degree": command["joint_units_0_001_degree"],
            "gripper_units_0_001_mm": command["gripper_units_0_001_mm"],
            "feedback_before_command": feedback_before_command,
            "previous_sent_calibrated_target": self._last_sent_calibrated_target,
            "previous_target_tracking_error": previous_target_tracking_error,
            "sdk_error": sdk_error,
        }
        self._active_command_log.write(
            json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n"
        )
        if (
            self._validated_action_count + 1
        ) % self.config.active_command_log_flush_every == 0:
            self._active_command_log.flush()
        if self._validated_action_count == 0:
            logger.info(
                "First policy motor target: joints_deg=%s, gripper_mm=%.3f, result=%s.",
                command["calibrated_joint_degrees"],
                command["calibrated_gripper_mm"],
                result,
            )
        if result == "sent":
            self._last_sent_calibrated_target = [
                *command["calibrated_joint_degrees"],
                command["calibrated_gripper_mm"],
            ]

    def _record_active_rejection(
        self,
        action: RobotAction,
        reason: str,
    ) -> None:
        """Persist a model output that was rejected before any SDK command."""
        self._open_active_command_log()
        if self._active_command_log is None:
            return
        raw = None
        if set(action) == set(FEATURES):
            candidate = [float(action[feature]) for feature in FEATURES]
            if all(math.isfinite(value) for value in candidate):
                raw = candidate
        record = {
            "schema_version": 1,
            "sequence": self._validated_action_count,
            "wall_time_utc": datetime.now(timezone.utc).isoformat(),
            "result": "safety_rejected",
            "motion_enabled": self.config.motion_enabled,
            "safety_profile": self.config.safety_profile,
            "displacement_window_enabled": self.config.enforce_displacement_window,
            "raw_model_action": raw,
            "physical_clipped_action": None,
            "was_physical_clipped": None,
            "limited_action": None,
            "was_slew_limited": None,
            "was_displacement_clipped": None,
            "warnings": [],
            "calibrated_joint_degrees": None,
            "calibrated_gripper_mm": None,
            "joint_units_0_001_degree": None,
            "gripper_units_0_001_mm": None,
            "feedback_before_command": None,
            "previous_sent_calibrated_target": self._last_sent_calibrated_target,
            "previous_target_tracking_error": None,
            "sdk_error": None,
            "safety_error": reason,
        }
        self._active_command_log.write(
            json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n"
        )
        self._active_command_log.flush()

    def configure(self) -> None:
        logger.info(
            "piper_active configured with motion_enabled=%s; configure() sends no command.",
            self.config.motion_enabled,
        )

    def _listen_for_external_control(
        self,
        duration_s: float | None = None,
        *,
        bus_factory: Callable[..., Any] = can.Bus,
    ) -> None:
        duration = self.config.external_control_quiet_s if duration_s is None else duration_s
        deadline = time.monotonic() + duration
        bus = bus_factory(
            interface="socketcan",
            channel=self.config.can_interface,
            receive_own_messages=False,
        )
        try:
            while time.monotonic() < deadline:
                message = bus.recv(timeout=min(0.05, max(0.0, deadline - time.monotonic())))
                if message is not None and int(message.arbitration_id) in MASTER_COMMAND_IDS:
                    raise ActiveSafetyError(
                        f"External master command frame 0x{message.arbitration_id:03X} detected."
                    )
        finally:
            bus.shutdown()

    def connect(self, calibrate: bool = True) -> None:
        try:
            if self.config.motion_enabled:
                self._validate_adapter_identity()
                if not self.safety.calibration_verified:
                    raise ActiveSafetyError(
                        "Calibration is not verified; refusing before CAN TX."
                    )
                self._listen_for_external_control()
            super().connect(calibrate=calibrate)
            self._open_active_command_log()
            try:
                observation = self.get_observation()
                self.safety.validate_initial_state(
                    self._initial_state_from_observation(observation)
                )
                if not self.config.motion_enabled:
                    self._dry_initial_state_ok = True
            except ActiveSafetyError as exc:
                if self.config.motion_enabled:
                    raise
                self._dry_initial_state_ok = False
                logger.warning("Zero-motion initial-state safety preview failed: %s", exc)
        except Exception as exc:
            if self.config.motion_enabled:
                if self.is_connected:
                    super().disconnect()
                self._latch_fault(
                    f"connect failure: {exc}",
                    request_emergency_stop=False,
                )
            self._close_active_command_log()
            raise
        mode = "motion-capable but disarmed" if self.config.motion_enabled else "zero-motion dry run"
        logger.info("%s connected in %s mode.", self, mode)

    def get_observation(self) -> RobotObservation:
        try:
            observation = super().get_observation()
            for name, camera in self.cameras.items():
                stats = camera.get_health_stats()
                age_ms = stats.get("latest_frame_age_ms")
                if age_ms is None or float(age_ms) > self.config.feedback_watchdog_s * 1000.0:
                    raise TimeoutError(f"Camera {name} is stale: age_ms={age_ms}.")
            self._last_feedback_monotonic = time.monotonic()
            return observation
        except Exception as exc:
            if self._armed:
                self._latch_fault(f"feedback failure: {exc}", request_emergency_stop=True)
            raise

    def _initial_state_from_observation(self, observation: RobotObservation) -> dict[str, float]:
        return {feature: float(observation[feature]) for feature in FEATURES}

    def _check_arm_prerequisites(self) -> None:
        if self._fault_latched:
            raise ActiveSafetyError(f"Fault is latched: {self._fault_reason}")
        if self._motion_stopped:
            raise ActiveSafetyError(f"Motion is stopped: {self._stop_reason}")
        if self._armed:
            raise ActiveSafetyError("Piper is already armed.")
        if not self.config.motion_enabled:
            raise ActiveSafetyError("motion_enabled=false; refusing to arm.")
        if not self.safety.calibration_verified:
            raise ActiveSafetyError("Calibration is not verified; refusing before CAN TX.")
        self._validate_adapter_identity()
        if self._interface is None or not self.is_connected:
            raise ActiveSafetyError("Robot and cameras must be connected before arming.")
        if set(self.cameras) != {"front", "wrist"}:
            raise ActiveSafetyError(
                "Active motion requires fresh front and wrist cameras matching training."
            )

    def _arm_with_validated_state(self, initial_state: dict[str, float]) -> None:
        self.safety.validate_initial_state(initial_state)
        if self._interface is None:
            raise ActiveSafetyError("Piper SDK interface disappeared before arming.")

        self._armed = True
        self._ever_armed = True
        try:
            enable_deadline = time.monotonic() + self.config.enable_timeout_s
            while time.monotonic() < enable_deadline:
                if bool(self._interface.EnablePiper()):
                    break
                time.sleep(0.01)
            else:
                raise ActiveSafetyError(
                    "All six Piper motors did not report enabled within "
                    f"{self.config.enable_timeout_s:.1f}s."
                )

            hold_target = [
                float(initial_state[f"joint_{index}.pos"])
                for index in range(1, 7)
            ]
            hold_period_s = 1.0 / self.config.arm_hold_command_hz

            def acquire_stable_hold(
                target: list[float],
                *,
                max_departure_deg: float,
            ) -> dict[str, float]:
                hold_units = tuple(int(round(value * 1000.0)) for value in target)
                hold_deadline = time.monotonic() + self.config.arm_hold_s
                stable_samples = 0
                previous = target.copy()
                latest = initial_state.copy()
                while time.monotonic() < hold_deadline or stable_samples < 3:
                    iteration_started = time.monotonic()
                    self._interface.ModeCtrl(
                        1,
                        1,
                        self.config.motion_speed_percent,
                        0,
                    )
                    self._interface.JointCtrl(*hold_units)
                    time.sleep(
                        max(
                            0.0,
                            hold_period_s - (time.monotonic() - iteration_started),
                        )
                    )
                    latest = self._read_arm_state()
                    joints = [
                        float(latest[f"joint_{index}.pos"])
                        for index in range(1, 7)
                    ]
                    deltas = [
                        actual - requested
                        for actual, requested in zip(joints, target, strict=True)
                    ]
                    if max(abs(value) for value in deltas) > max_departure_deg:
                        raise ActiveSafetyError(
                            "Piper departed from its measured start pose while arming; "
                            f"target={target}, deltas={deltas}, "
                            f"allowed_departure={max_departure_deg:.3f}."
                        )
                    sample_change = max(
                        abs(actual - prior)
                        for actual, prior in zip(joints, previous, strict=True)
                    )
                    if sample_change <= self.config.arm_acquisition_stability_deg:
                        stable_samples += 1
                    else:
                        stable_samples = 0
                    previous = joints
                return {feature: float(latest[feature]) for feature in FEATURES}

            settled_state = acquire_stable_hold(
                hold_target,
                max_departure_deg=self.config.arm_acquisition_max_drift_deg,
            )
            settled_target = [
                float(settled_state[f"joint_{index}.pos"])
                for index in range(1, 7)
            ]
            settling_deltas = [
                settled - requested
                for settled, requested in zip(
                    settled_target,
                    hold_target,
                    strict=True,
                )
            ]
            if max(abs(value) for value in settling_deltas) > 1e-9:
                self.safety.validate_initial_state(settled_state)
                acquire_stable_hold(
                    settled_target,
                    max_departure_deg=self.config.arm_hold_tolerance_deg,
                )
                logger.info(
                    "Piper arm-enable settling stabilized; rebased hold deltas=%s.",
                    settling_deltas,
                )
        except Exception as exc:
            self._latch_fault(
                f"motion-mode setup failed: {exc}",
                request_emergency_stop=True,
            )
            raise
        now = time.monotonic()
        self._motion_started_monotonic = now
        self._last_action_monotonic = now
        self._last_feedback_monotonic = now
        self._start_watchdog()

    def arm_for_motion(self) -> None:
        try:
            self._check_arm_prerequisites()
            self._listen_for_external_control()
            observation = self.get_observation()
            self._arm_with_validated_state(
                self._initial_state_from_observation(observation)
            )
        except Exception as exc:
            self._latch_fault(str(exc), request_emergency_stop=self._armed)
            raise

    def _start_watchdog(self) -> None:
        self._watchdog_stop.clear()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            name="piper_active_watchdog",
            daemon=True,
        )
        self._watchdog_thread.start()

    def _watchdog_loop(self) -> None:
        period = min(self.config.action_watchdog_s, self.config.feedback_watchdog_s) / 4.0
        while not self._watchdog_stop.wait(max(0.005, period)):
            now = time.monotonic()
            if (
                self._last_action_monotonic is None
                or now - self._last_action_monotonic > self.config.action_watchdog_s
            ):
                self._latch_fault("action watchdog expired", request_emergency_stop=True)
                return
            if (
                self._last_feedback_monotonic is None
                or now - self._last_feedback_monotonic > self.config.feedback_watchdog_s
            ):
                self._latch_fault("feedback watchdog expired", request_emergency_stop=True)
                return

    def _latch_fault(self, reason: str, *, request_emergency_stop: bool) -> None:
        with self._fault_lock:
            if self._fault_latched:
                return
            self._fault_latched = True
            self._fault_reason = reason
            self._motion_stopped = True
            self._stop_reason = f"fault: {reason}"
            self._watchdog_stop.set()
            if (
                request_emergency_stop
                and self._armed
                and not self._emergency_stop_sent
                and self._interface is not None
            ):
                self._emergency_stop_sent = True
                try:
                    self._interface.EmergencyStop(1)
                except Exception:
                    logger.exception("Piper EmergencyStop(1) failed while latching a fault.")
            self._armed = False
            logger.error("Piper active fault latched: %s", reason)

    def stop_motion(self, reason: str) -> None:
        """Quick-stop once, then permanently suppress motion for this process."""
        with self._fault_lock:
            if self._motion_stopped:
                return
            self._motion_stopped = True
            self._stop_reason = reason
            self._watchdog_stop.set()
            if self._armed and not self._emergency_stop_sent and self._interface is not None:
                self._emergency_stop_sent = True
                try:
                    self._interface.EmergencyStop(1)
                except Exception as exc:
                    self._fault_latched = True
                    self._fault_reason = f"EmergencyStop failed: {exc}"
                    logger.exception("Piper EmergencyStop(1) failed.")
            self._armed = False
            logger.warning("Piper motion stopped and latched for this process: %s", reason)

    def _active_limit_reason(self, now: float) -> str | None:
        if (
            self.config.max_active_actions > 0
            and self._active_action_count >= self.config.max_active_actions
        ):
            return f"active action budget reached ({self.config.max_active_actions})"
        if (
            self.config.max_motion_duration_s > 0
            and self._motion_started_monotonic is not None
            and now - self._motion_started_monotonic >= self.config.max_motion_duration_s
        ):
            return (
                "motion duration limit reached "
                f"({self.config.max_motion_duration_s:.3f}s)"
            )
        return None

    def _record_suppressed_action(self, validated: RobotAction) -> RobotAction:
        self._suppressed_action_count += 1
        if self._suppressed_action_count == 1 or self._suppressed_action_count % 30 == 0:
            logger.warning(
                "Suppressed active action %d after stop (%s); no SDK command sent.",
                self._suppressed_action_count,
                self._stop_reason,
            )
        return validated

    def send_action(self, action: RobotAction) -> RobotAction:
        try:
            validated = {key: float(value) for key, value in action.items()}
        except Exception as exc:
            if self.config.motion_enabled:
                self._latch_fault(
                    f"action decoding failed: {exc}",
                    request_emergency_stop=self._armed,
                )
            raise
        if self._fault_latched:
            raise ActiveSafetyError(f"Fault is latched: {self._fault_reason}")
        if self._motion_stopped:
            return self._record_suppressed_action(validated)

        limit_reason = self._active_limit_reason(time.monotonic())
        if self.config.motion_enabled and limit_reason is not None:
            self.stop_motion(limit_reason)
            return self._record_suppressed_action(validated)

        if self.config.motion_enabled and not self._armed:
            if not self.config.arm_on_first_action:
                reason = "Active motion is not armed; no command was sent."
                self._latch_fault(reason, request_emergency_stop=False)
                raise ActiveSafetyError(reason)
            try:
                self._check_arm_prerequisites()
                self._listen_for_external_control()
                observation = self.get_observation()
                initial_state = self._initial_state_from_observation(observation)
                # No CAN TX occurs unless this first action passes the complete profile.
                self.safety.preview(validated, initial_state=initial_state)
                self._arm_with_validated_state(initial_state)
            except Exception as exc:
                self._record_active_rejection(validated, str(exc))
                self._latch_fault(str(exc), request_emergency_stop=self._armed)
                raise
        try:
            command = self.safety.prepare(validated)
        except Exception as exc:
            if self.config.motion_enabled:
                self._record_active_rejection(validated, str(exc))
                self._latch_fault(str(exc), request_emergency_stop=self._armed)
                raise
            if not isinstance(exc, ActiveSafetyError):
                raise
            self._dry_run_rejections += 1
            if self._dry_run_rejections == 1 or self._dry_run_rejections % 30 == 0:
                logger.warning(
                    "Zero-motion safety preview rejection %d: %s",
                    self._dry_run_rejections,
                    exc,
                )
            return validated

        if not self.config.motion_enabled:
            self._record_active_command(command, result="preview_only")
            self._validated_action_count += 1
            return {
                feature: float(command["limited"][index])
                for index, feature in enumerate(FEATURES)
            }
        if not self._armed or self._interface is None:
            reason = "Active motion lost its armed SDK interface; no command was sent."
            self._latch_fault(reason, request_emergency_stop=self._armed)
            raise ActiveSafetyError(reason)

        for warning in command["warnings"]:
            self._safety_warning_count += 1
            if self._safety_warning_count == 1 or self._safety_warning_count % 30 == 0:
                logger.warning(
                    "Piper %s safety observation %d: %s",
                    self.config.safety_profile,
                    self._safety_warning_count,
                    warning,
                )

        with self._fault_lock:
            if self._fault_latched:
                raise ActiveSafetyError(f"Fault is latched: {self._fault_reason}")
            if self._motion_stopped:
                return self._record_suppressed_action(validated)
            limit_reason = self._active_limit_reason(time.monotonic())
            if limit_reason is not None:
                self.stop_motion(limit_reason)
                return self._record_suppressed_action(validated)
            if not self._armed or self._interface is None:
                raise ActiveSafetyError("Active motion is not armed; no command was sent.")
            try:
                self._interface.ModeCtrl(
                    1,
                    1,
                    self.config.motion_speed_percent,
                    0,
                )
                self._interface.JointCtrl(*command["joint_units_0_001_degree"])
                self._interface.GripperCtrl(
                    command["gripper_units_0_001_mm"],
                    self.config.gripper_effort,
                    1,
                    0,
                )
            except Exception as exc:
                self._record_active_command(
                    command,
                    result="sdk_error",
                    sdk_error=str(exc),
                )
                self._latch_fault(
                    f"SDK command failure: {exc}",
                    request_emergency_stop=True,
                )
                raise
            self._last_action_monotonic = time.monotonic()
            self._record_active_command(command, result="sent")
            self._active_action_count += 1
            self._validated_action_count += 1
            limit_reason = self._active_limit_reason(self._last_action_monotonic)
            if limit_reason is not None:
                self.stop_motion(limit_reason)
        return {
            feature: float(command["limited"][index])
            for index, feature in enumerate(FEATURES)
        }

    def disconnect(self) -> None:
        if self._ever_armed and not self._motion_stopped:
            self.stop_motion("disconnect")
        self._watchdog_stop.set()
        if self._watchdog_thread is not None:
            self._watchdog_thread.join(timeout=1.0)
            self._watchdog_thread = None
        # Intentionally no reset, disable, homing, role change, or automatic recovery.
        try:
            super().disconnect()
        finally:
            self._close_active_command_log()
        logger.info(
            "%s disconnected; active_actions=%d, suppressed_actions=%d, "
            "dry-run safety rejections=%d, stop=%s, fault=%s.",
            self,
            self._active_action_count,
            self._suppressed_action_count,
            self._dry_run_rejections,
            self._stop_reason,
            self._fault_reason,
        )
