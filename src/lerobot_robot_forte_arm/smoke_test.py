#!/usr/bin/env python
"""
Read-only hardware smoke test for the Forte master/slave rig's LeRobot integration.

Opens both CAN buses, reads present position on all 4 joints for BOTH the master (leader) and
slave (follower) arms, and optionally grabs one frame from the RealSense camera. It talks to the
motors buses directly instead of going through ForteArm.connect()/configure(), so torque is never
enabled and neither arm can move -- use this to check wiring, CAN ids and the camera before
trying lerobot-record or lerobot-teleoperate.

Safe to run whether or not the Teensy's bilateral firmware (teensy-forte, teleop-bi-p-t branch) is
currently running -- this script never writes to the bus.
"""

import argparse

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.robstride.robstride import RobstrideMotorsBus

from lerobot_robot_forte_arm.config import CAN_BUSES, JOINTS


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port-can1", required=True, help="CAN interface for bus 'can1', e.g. /dev/ttyUSB0 or can0")
    parser.add_argument("--port-can2", required=True, help="CAN interface for bus 'can2', e.g. /dev/ttyUSB1 or can1")
    parser.add_argument("--can-interface", default="auto", choices=["auto", "slcan", "socketcan"])
    parser.add_argument("--motor-type", default="o0", help="Robstride motor type shared by all motors")
    parser.add_argument("--skip-camera", action="store_true", help="Don't try to read from the RealSense camera")
    args = parser.parse_args()

    ports = {"can1": args.port_can1, "can2": args.port_can2}

    for role, id_index in (("master", 2), ("slave", 1)):
        motors_by_bus: dict[str, dict[str, Motor]] = {bus: {} for bus in CAN_BUSES}
        gear_ratio: dict[str, float] = {}
        for joint_name, entry in JOINTS.items():
            bus, slave_id, master_id, ratio = entry
            can_id = slave_id if role == "slave" else master_id
            motors_by_bus[bus][joint_name] = Motor(
                id=can_id, model="robstride", norm_mode=MotorNormMode.DEGREES, motor_type_str=args.motor_type
            )
            gear_ratio[joint_name] = ratio

        print(f"\n=== {role.upper()} arm ===")
        buses = {
            bus: RobstrideMotorsBus(
                port=ports[bus], motors=motors, can_interface=args.can_interface, use_can_fd=False
            )
            for bus, motors in motors_by_bus.items()
        }
        for bus in buses.values():
            bus.connect(handshake=True)
        print(f"CAN handshake OK -- all {role} motors responded.\n")

        positions = {bus_name: bus.sync_read("Present_Position") for bus_name, bus in buses.items()}

        print(f"{'joint':<16} {'bus':>5} {'CAN id':>7} {'gear ratio':>11} {'link deg':>10} {'motor deg':>10}")
        for joint_name, entry in JOINTS.items():
            bus_name, slave_id, master_id, ratio = entry
            can_id = slave_id if role == "slave" else master_id
            motor_deg = positions[bus_name][joint_name]
            link_deg = motor_deg / ratio
            print(f"{joint_name:<16} {bus_name:>5} {can_id:>7} {ratio:>11.4f} {link_deg:>10.2f} {motor_deg:>10.2f}")

        for bus in buses.values():
            bus.disconnect(disable_torque=False)
        print(f"{role} CAN buses disconnected.")

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
