"""Record joint + gripper from leader arm, then replay on follower arm."""
import alicia_d_sdk
import time
import json
import sys

RECORD_FILE = "recording.json"
RECORD_HZ = 20
REPLAY_SPEED_DEG_S = 100

def record():
    port = None
    import serial.tools.list_ports
    for p in serial.tools.list_ports.comports():
        if "wch" in p.device.lower():
            port = p.device
            break
    if not port:
        print("No robot found!")
        return

    print(f"Connecting to leader on {port}...")
    robot = alicia_d_sdk.create_robot(port=port)
    time.sleep(0.5)

    frames = []
    dt = 1.0 / RECORD_HZ

    print(f"\nRecording at {RECORD_HZ} Hz. Press Ctrl+C to stop.\n")
    try:
        start = time.time()
        while True:
            t = time.time() - start
            state = robot.get_robot_state("joint_gripper")
            gripper = robot.get_robot_state("gripper")
            angles = [round(a, 5) for a in state.angles]
            frames.append({"t": round(t, 4), "joints": angles, "gripper": round(gripper, 1)})
            count = len(frames)
            if count % RECORD_HZ == 0:
                print(f"  [{t:.1f}s] {count} frames, gripper={gripper}")
            time.sleep(dt)
    except KeyboardInterrupt:
        pass

    robot.disconnect()

    with open(RECORD_FILE, "w") as f:
        json.dump({"hz": RECORD_HZ, "frames": frames}, f)

    print(f"\nSaved {len(frames)} frames ({len(frames)/RECORD_HZ:.1f}s) to {RECORD_FILE}")


def replay():
    with open(RECORD_FILE, "r") as f:
        data = json.load(f)

    frames = data["frames"]
    hz = data["hz"]
    dt = 1.0 / hz

    port = None
    import serial.tools.list_ports
    for p in serial.tools.list_ports.comports():
        if "wch" in p.device.lower():
            port = p.device
            break
    if not port:
        print("No robot found!")
        return

    print(f"Connecting to follower on {port}...")
    robot = alicia_d_sdk.create_robot(port=port)
    time.sleep(0.5)

    print(f"\nReplaying {len(frames)} frames ({len(frames)/hz:.1f}s). Press Ctrl+C to stop.\n")
    try:
        for i, frame in enumerate(frames):
            robot.set_robot_state(
                target_joints=frame["joints"],
                gripper_value=frame["gripper"],
                speed_deg_s=REPLAY_SPEED_DEG_S,
                wait_for_completion=False,
            )
            if i % hz == 0:
                print(f"  [{frame['t']:.1f}s] frame {i}/{len(frames)}, gripper={frame['gripper']}")
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nStopped early.")

    robot.disconnect()
    print("\nReplay done!")


if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] not in ("record", "replay"):
        print("Usage:")
        print("  uv run python record_replay.py record   # record from leader")
        print("  uv run python record_replay.py replay   # replay on follower")
        sys.exit(1)

    if sys.argv[1] == "record":
        record()
    else:
        replay()
