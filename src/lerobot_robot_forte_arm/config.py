from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.configs import ColorMode, Cv2Rotation
from lerobot.cameras.realsense import RealSenseCameraConfig
from lerobot.robots import RobotConfig

# joint name -> (slave CAN id, master CAN id, motor:link external gear ratio)
#
# The (slave = master + 10) id pairing is read directly from teensy-forte's bilateral firmware
# (`teleop-bi` branch, teensy/teensy.ino: MST_IDS_CAN*/SLV_IDS_CAN* arrays) and holds regardless
# of which physical CAN bus (CAN1/CAN2) a pair happens to be wired to on the Teensy -- that
# grouping is irrelevant here, since the host never touches CAN directly at all, only the
# Teensy's single serial port (see teensy_link.TeensyLink).
#
# joint-name <-> id mapping cross-checked against teensy-dash/src/main.cpp's JOINT_NAMES /
# JOINT_CAN_IDS / GEAR_RATIO arrays, which describe the same physical slave arm.
JOINTS: dict[str, tuple[int, int, float]] = {
    "shoulder_yaw": (11, 1, 4.8077),
    "shoulder_roll": (12, 2, 1.0),
    "shoulder_pitch": (13, 3, 3.180),
    "elbow_pitch": (14, 4, 1.0),
}


@RobotConfig.register_subclass("forte_arm")
@dataclass
class ForteArmConfig(RobotConfig):
    # Serial port of the Teensy running teensy-forte's bilateral firmware, e.g. /dev/ttyACM0.
    port: str
    baudrate: int = 115200

    # If the Teensy hasn't reported a motor's position in longer than this, get_observation()
    # logs a warning (the stale value is still returned -- there's currently no fresher source).
    stale_after_s: float = 3.0

    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "cam_1": RealSenseCameraConfig(
                serial_number_or_name="825312072171",
                fps=15,
                width=640,
                height=480,
                color_mode=ColorMode.RGB,
                use_depth=False,
                rotation=Cv2Rotation.NO_ROTATION,
            )
        }
    )
