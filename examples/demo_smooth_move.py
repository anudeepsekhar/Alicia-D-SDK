"""Smooth multi-joint movement demo with speed control."""
import alicia_d_sdk
import time
import math


def main():
    robot = alicia_d_sdk.create_robot()
    print("Connected!")

    def move(pose, wait=2.5):
        robot.set_robot_state(
            target_joints=pose,
            joint_format="deg",
            speed_deg_s=60,
            wait_for_completion=False,
        )
        t0 = time.time()
        while time.time() - t0 < wait:
            cur = robot.get_robot_state("joint")
            if cur:
                degs = [round(math.degrees(j), 1) for j in cur]
                print(f"\r  -> {degs}", end="", flush=True)
            time.sleep(0.1)
        print()

    try:
        print("Homing...")
        move([0, 0, 0, 0, 0, 0], 2)

        print("1. Base sweep right, shoulder up")
        move([40, 20, -15, 0, 0, 0], 2)

        print("2. Wrist twist")
        move([40, 20, -15, 30, -25, 40], 2)

        print("3. Base sweep left")
        move([-40, 20, -15, 30, -25, 40], 3)

        print("4. Reach up")
        move([-40, 35, -30, -20, 20, -30], 2.5)

        print("5. Wave right")
        move([0, 30, -25, 25, -20, 0], 2.5)

        print("6. Wave left")
        move([0, 30, -25, -25, 20, 0], 2)

        print("7. Compact pose")
        move([30, 40, -35, 15, -15, 25], 2.5)

        print("8. Home")
        move([0, 0, 0, 0, 0, 0], 2.5)

        print("Done!")
    except KeyboardInterrupt:
        print("\nInterrupted - homing...")
        move([0, 0, 0, 0, 0, 0], 3)
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
