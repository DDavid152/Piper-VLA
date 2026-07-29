# LeRobot Piper robot plugin

This package registers `--robot.type=piper` for the Piper-VLA workstation.
It is intentionally limited to the verified Piper native master/slave control
chain:

- the two arms perform teleoperation directly on their shared CAN bus;
- the plugin passively reads follower feedback and the configured cameras;
- `connect()` uses `piper_init=False` and sends no SDK initialization queries;
- `send_action()` validates and returns the recorded action but never transmits
  it to the follower;
- calibration, configuration, enable, homing, reset and disconnect are all
  non-moving operations.

The seven scalar state/action fields are `joint_1.pos` through `joint_6.pos`
in degrees and `gripper.pos` in millimeters. The CAN interface is accepted only
when its USB adapter serial matches the configured shared-bus adapter.

This plugin does not implement LeRobot active control. Adding an active mode
would require a separate safety review and must not be combined with native
master/slave teleoperation.
