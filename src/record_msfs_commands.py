#!/usr/bin/env python3
import argparse
import datetime
import os
import signal
import subprocess
import sys


DEFAULT_TOPICS = [
    "/msfs/cmd/attitude",
    "/msfs/cmd/throttle",
    "/msfs/cmd/flaps",
    "/msfs/cmd/mixture",
    "/msfs/cmd/trim",
    "/msfs/cmd/parking_brake",
    "/msfs/cmd/baro",
    "/msfs/cmd/dg_deg",
]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Record MSFS command topics into a ROS 2 bag."
    )
    parser.add_argument(
        "--output-dir",
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "bags"),
        help="Parent directory where bag folders will be created.",
    )
    parser.add_argument(
        "--name",
        default=None,
        help="Optional bag name. If omitted, a timestamped name is used.",
    )
    parser.add_argument(
        "--storage",
        default="sqlite3",
        choices=["sqlite3", "mcap"],
        help="ROS 2 bag storage backend.",
    )
    parser.add_argument(
        "--include-sensors",
        action="store_true",
        help="Also record /msfs/sensor_readings.",
    )
    args = parser.parse_args()

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_name = args.name or f"msfs_commands_{timestamp}"
    os.makedirs(args.output_dir, exist_ok=True)
    bag_path = os.path.join(args.output_dir, bag_name)

    topics = list(DEFAULT_TOPICS)
    if args.include_sensors:
        topics.append("/msfs/sensor_readings")

    cmd = [
        "ros2",
        "bag",
        "record",
        "-o",
        bag_path,
        "--storage",
        args.storage,
        *topics,
    ]

    print("Starting bag recording:")
    print(" ".join(cmd))
    print("\nPress Ctrl+C to stop.\n")

    proc = subprocess.Popen(cmd)

    try:
        proc.wait()
    except KeyboardInterrupt:
        print("\nStopping recording...")
        if os.name == "nt":
            proc.send_signal(signal.CTRL_BREAK_EVENT)
        else:
            proc.send_signal(signal.SIGINT)
        proc.wait()

    print(f"\nBag saved to: {bag_path}")
    return proc.returncode


if __name__ == "__main__":
    sys.exit(main())