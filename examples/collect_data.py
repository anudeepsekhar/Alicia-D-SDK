#!/usr/bin/env python3
"""VLA Data Collection Script

Collect demonstration episodes for VLA training using either:
  - Leader-follower teleoperation (two arms)
  - Drag teaching (single arm, torque off)

Examples:

  # Drag teaching with one camera at 10 Hz
  python collect_data.py --mode drag --camera 0 --fps 10

  # Leader-follower with two arms and one camera
  python collect_data.py --mode leader_follower \
      --follower-port /dev/ttyACM0 \
      --leader-port /dev/ttyACM1 \
      --camera 0 --fps 10

  # Multiple cameras, custom resolution
  python collect_data.py --mode drag --camera 0 2 \
      --image-width 320 --image-height 240 --fps 15

  # Resume collecting into an existing dataset
  python collect_data.py --mode drag --camera 0 \
      --dataset ./vla_dataset
"""

import argparse
from alicia_d_sdk import create_robot
from alicia_d_sdk.execution.data_collector import DataCollector


def main():
    parser = argparse.ArgumentParser(
        description="VLA Data Collection for Alicia-D",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        "--mode",
        choices=["drag", "leader_follower"],
        default="drag",
        help="Collection mode (default: drag)",
    )
    parser.add_argument("--follower-port", type=str, default="", help="Follower arm serial port")
    parser.add_argument("--leader-port", type=str, default="", help="Leader arm serial port (leader_follower mode)")
    parser.add_argument("--camera", type=int, nargs="*", default=[0], help="Camera device indices (default: 0)")
    parser.add_argument("--fps", type=float, default=10.0, help="Recording frequency in Hz (default: 10)")
    parser.add_argument("--dataset", type=str, default="./vla_dataset", help="Dataset output directory")
    parser.add_argument("--image-width", type=int, default=640, help="Image width (default: 640)")
    parser.add_argument("--image-height", type=int, default=480, help="Image height (default: 480)")

    args = parser.parse_args()

    print("Connecting follower arm...")
    follower = create_robot(port=args.follower_port)

    leader = None
    if args.mode == "leader_follower":
        if not args.leader_port:
            parser.error("--leader-port is required for leader_follower mode")
        print("Connecting leader arm...")
        leader = create_robot(port=args.leader_port, variant="leader_ur")

    try:
        collector = DataCollector(
            follower=follower,
            leader=leader,
            camera_indices=args.camera,
            dataset_dir=args.dataset,
            fps=args.fps,
            image_width=args.image_width,
            image_height=args.image_height,
        )
        collector.start()
    finally:
        follower.disconnect()
        if leader is not None:
            leader.disconnect()


if __name__ == "__main__":
    main()
