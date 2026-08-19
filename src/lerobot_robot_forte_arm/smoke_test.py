#!/usr/bin/env python
"""
Read-only hardware smoke test for the Forte master/slave rig, talking to the Teensy over its
single USB-serial port (teensy-forte, `teleop-bi` / `teleop-bi-p-t` branches).

Never sends 'e' -- doesn't enable or move anything. Just opens the serial link, waits long enough
to catch at least one status-print cycle from the Teensy (~1-2s, see LOG_PERIOD in teensy.ino),
and prints whatever master/slave positions it received. Also optionally grabs one RealSense frame.
"""

import argparse
import time

from lerobot_robot_forte_arm.config import JOINTS
from lerobot_robot_forte_arm.teensy_link import TeensyLink


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True, help="Teensy serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--wait-s", type=float, default=3.0, help="How long to wait for status lines")
    parser.add_argument("--skip-camera", action="store_true", help="Don't try to read from the RealSense camera")
    args = parser.parse_args()

    link = TeensyLink.get(args.port, args.baudrate)
    print(f"Connecting to Teensy on {args.port} @ {args.baudrate} (never sends 'e' -- nothing will move)...")
    link.connect()
    print(f"Waiting up to {args.wait_s:.1f}s for status lines...")
    time.sleep(args.wait_s)

    positions = link.get_positions_deg()
    print(
        f"\n{'joint':<16} {'slave id':>9} {'master id':>10} {'gear ratio':>11} "
        f"{'slave deg':>10} {'master deg':>11}"
    )
    missing = []
    for joint, (slave_id, master_id, ratio) in JOINTS.items():
        if slave_id not in positions or master_id not in positions:
            missing.append(joint)
            continue
        slave_link_deg = positions[slave_id] / ratio
        master_link_deg = positions[master_id] / ratio
        print(
            f"{joint:<16} {slave_id:>9} {master_id:>10} {ratio:>11.4f} "
            f"{slave_link_deg:>10.2f} {master_link_deg:>11.2f}"
        )

    if missing:
        print(f"\nNo data received yet for: {missing}")
        print("Check the Teensy is powered, running teensy-forte's bilateral firmware, and that")
        print(f"--wait-s (currently {args.wait_s}) covers at least one status-print cycle.")

    link.disconnect()
    print("\nSerial link disconnected.")

    if not args.skip_camera:
        from lerobot.cameras.configs import ColorMode, Cv2Rotation
        from lerobot.cameras.realsense import RealSenseCamera, RealSenseCameraConfig

        print("\nConnecting to RealSense camera...")
        cam = RealSenseCamera(
            RealSenseCameraConfig(
                serial_number_or_name="825312072171",
                fps=15,
                width=640,
                height=480,
                color_mode=ColorMode.RGB,
                use_depth=False,
                rotation=Cv2Rotation.NO_ROTATION,
            )
        )
        cam.connect()
        try:
            frame = cam.read()
            print(f"Camera OK -- frame shape {frame.shape}")
        finally:
            cam.disconnect()


if __name__ == "__main__":
    main()
