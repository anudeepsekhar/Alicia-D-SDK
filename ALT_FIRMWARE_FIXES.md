# Alicia-D SDK — Alternate Firmware Compatibility Fixes

## Problem

The Alicia-D robot arm ships with firmware that uses the older **v5 serial protocol**, but the SDK (v6.1.0) expects the newer v6 protocol. This mismatch caused complete communication failure on both macOS and Linux — the SDK could not read joint data or send commands.

## Root Cause

The v5 firmware and v6 SDK differ in three critical areas:

| | v5 Firmware (actual) | v6.1.0 SDK (expected) |
|---|---|---|
| **Frame format** | `[AA][Cmd][Len][Data][CRC][FF]` (no Func byte) | `[AA][Cmd][Func][Len][Data][CRC][FF]` |
| **CRC algorithm** | `sum(data_bytes) % 2` (parity bit) | `CRC32(frame) & 0xFF` |
| **Command IDs** | Joint=`0x04`, Torque=`0x13`, Feedback=`0x14`/`0x12` | Joint=`0x06`, Torque=`0x05`, Feedback=`0x06`/`0x04` |
| **Joint data layout** | 9 servo positions (18 bytes) | 6 joints + gripper + status (15 bytes) |
| **Speed control** | Separate CMD `0x05` with func `0x2E` | Speed embedded in joint frame (4 bytes/joint) |

## Files Modified

### 1. `alicia_d_sdk/hardware/serial_comm.py`

**Incoming frame translation** — auto-detects v5 firmware and converts frames to v6 format so the rest of the SDK works unchanged.

- **Alt firmware detection**: When a frame with cmd `0x14` or `0x12` arrives with `Len` at byte[2] (no Func byte), the class sets `_alt_firmware_mode = True`.
- **Command ID mapping**: `0x14` → `0x06` (joint data), `0x12` → `0x04` (gripper/status).
- **9-servo → 6-joint conversion**: For cmd `0x14` (18 bytes = 9 servos), converts to 6 logical joints using the servo-to-joint map with direction multipliers, then builds a standard 15-byte data payload (6 joints + gripper + status) that `DataParser` can parse.
- **CRC bypass**: Skips CRC validation for all frames when alt firmware is active (different algorithm).

**Key constants added:**
```python
ALT_FW_CMD_TO_STANDARD = {0x14: (0x06, 0x00), 0x12: (0x04, 0x00)}
ALT_FW_SERVO_TO_JOINT = [
    (0, 1.0), (0, 1.0),       # Servos 0,1 → Joint 0 (same direction)
    (1, 1.0), (1, -1.0),      # Servos 2,3 → Joint 1 (mirrored)
    (2, 1.0), (2, -1.0),      # Servos 4,5 → Joint 2 (mirrored)
    (3, 1.0), (4, 1.0), (5, 1.0),  # Servos 6,7,8 → Joints 3,4,5
]
```

### 2. `alicia_d_sdk/hardware/servo_driver.py`

**Outgoing command building** — constructs v5-format frames when alt firmware is detected.

- **`_build_alt_fw_joint_frame()`**: Maps 6 logical joints → 9 servo positions using direction multipliers, builds `[AA][0x04][0x12][18 data bytes][CRC][FF]` with parity CRC.
- **`_build_v5_command_frame()`**: Generic v5 frame builder for any command.
- **`alt_fw_set_speed(deg_s)`**: Sends speed command `[AA][0x05][len][0x2E][speed×10 channels][CRC][FF]`. Speed range 1–360 deg/s, converted via `(rad_s / 2π) × 3400`.
- **`alt_fw_set_acceleration()`**: Sends max acceleration `[AA][0x05][len][0x29][254×18][CRC][FF]`.
- **Torque commands**: Routed through `ALT_FW_INFO_COMMAND_MAP` — torque on: `[AA][0x13][0x01][0x01][CRC][FF]`, torque off: `[AA][0x13][0x01][0x00][CRC][FF]`.
- **`set_joint_and_gripper()`**: Dispatches to `_build_alt_fw_joint_frame()` when `_alt_firmware_mode` is active.
- **`acquire_info()`**: Uses `ALT_FW_INFO_COMMAND_MAP` for torque commands when alt firmware is active.

### 3. `alicia_d_sdk/api/synria_robot_api.py`

**Connect flow optimization** and speed control API.

- **Skips version query**: The v5 firmware doesn't support version queries, so `connect()` skips `_robot_type()` when alt firmware is detected (saves ~70 seconds of timeout).
- **Auto-configures speed**: On connect, sets acceleration to max and servo speed to 200 deg/s.
- **`set_servo_speed(deg_s)`**: Public API to change servo speed at runtime.

## Protocol Reference

### V5 Frame Format
```
[0xAA] [CMD] [LEN] [DATA × LEN bytes] [CRC] [0xFF]
```
- **CRC**: `sum(DATA) % 2` (single parity bit)
- Total overhead: 5 bytes (header + cmd + len + crc + footer)

### V5 Command Summary

| Command | CMD ID | Data | Description |
|---|---|---|---|
| Joint control | `0x04` | 18 bytes (9 servos × 2) | Set all servo positions |
| Speed set | `0x05` | `[0x2E]` + 20 bytes (10 ch × 2) | Set servo speed (1–3400 raw) |
| Acceleration set | `0x05` | `[0x29]` + 18 bytes | Set acceleration (254 = max) |
| Torque on | `0x13` | `[0x01]` | Enable torque hold |
| Torque off | `0x13` | `[0x00]` | Disable torque (arm goes limp) |

### V5 Incoming Data (firmware → host)

| CMD ID | Len | Content | Maps to SDK |
|---|---|---|---|
| `0x14` | 18 | 9 servo positions (2 bytes each) | `CMD_JOINT` (0x06) |
| `0x12` | varies | Gripper/status data | `CMD_GRIPPER` (0x04) |

### Servo-to-Joint Mapping

The arm has 6 logical joints driven by 9 physical servos:

| Servo | Joint | Direction | Notes |
|---|---|---|---|
| 0 | 0 (Base) | +1 | Dual-servo joint |
| 1 | 0 (Base) | +1 | Dual-servo joint |
| 2 | 1 (Shoulder) | +1 | Mirrored pair |
| 3 | 1 (Shoulder) | −1 | Mirrored pair |
| 4 | 2 (Elbow) | +1 | Mirrored pair |
| 5 | 2 (Elbow) | −1 | Mirrored pair |
| 6 | 3 (Wrist 1) | +1 | Single servo |
| 7 | 4 (Wrist 2) | +1 | Single servo |
| 8 | 5 (Wrist 3) | +1 | Single servo |

### Hardware Value Conversion

```
hardware_value = (angle_rad + π) / (2π) × 4096
```
- Range: 0–4095
- 0 → −180°, 2048 → 0°, 4095 → +180°

## Usage Notes

- **Single commands**: Send one joint command per target pose. The firmware's internal motion planner handles interpolation. Sending commands at high frequency (>10 Hz) causes the planner to restart repeatedly, resulting in jerky motion.
- **Speed control**: Call `robot.set_servo_speed(deg_s)` to adjust motion speed. Default is 200 deg/s on connect. Range: 1–360 deg/s.
- **Safe angles**: Positive shoulder (joint 2) values raise the arm up. Negative values reach down — use caution to avoid collisions with the workspace.
- **macOS**: Requires the CH34x VCP driver (`cn.wch.CH34xVCPDriver`). The arm may need to be connected through a USB hub to be detected.
