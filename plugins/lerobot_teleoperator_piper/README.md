# LeRobot Piper master teleoperator plugin

This package registers `--teleop.type=piper_master`. It opens a passive
SocketCAN receive socket on the already-running Piper shared bus and reads:

- master target frames `0x155`, `0x156`, `0x157`, and `0x159`;
- follower state frames `0x2A1`, `0x2A5`, `0x2A6`, `0x2A7`, and `0x2A8`.

Complete, coherent master target triplets become the seven-dimensional LeRobot
action in degrees and millimeters. Before the first master target arrives, the
fresh follower state is used as the held target so that a stationary recording
can start safely. The last complete master target remains valid while the
native controller holds it.

The plugin never calls `Bus.send()`. `send_feedback()`, calibration,
configuration and disconnect are non-transmitting operations. It also checks
the USB-CAN serial before opening the receive socket.
