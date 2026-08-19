from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.configs import ColorMode, Cv2Rotation
from lerobot.cameras.realsense import RealSenseCameraConfig
from lerobot.robots import RobotConfig

# Forte is a bilateral master/slave (leader/follower) rig: two physically-identical arms, a
# human-driven MASTER and a motorized SLAVE that mirrors it. Ground truth for all of this is the
# `teleop-bi-p-t` branch of the `teensy-forte` repo (teensy/teensy.ino), which already runs the
# full bilateral position-torque control loop (500 Hz, master position -> slave goal position,
# slave contact torque -> master haptic feedback) directly on the Teensy, independent of any host
# PC. See SMOLVLA_GUIDE.md section 1 for the full architecture writeup.
#
# The Teensy exposes 2 independent CAN buses; each carries one master/slave joint pair:
#   CAN1: master 1 (shoulder_yaw)   <-> slave 11 (shoulder_yaw)
#         master 2 (shoulder_roll)  <-> slave 12 (shoulder_roll)
#   CAN2: master 3 (shoulder_pitch) <-> slave 13 (shoulder_pitch)
#         master 4 (elbow_pitch)    <-> slave 14 (elbow_pitch)
#
# joint name -> (CAN bus, slave CAN id, master CAN id, motor:link external gear ratio)
# Gear ratios are carried over from teensy-dash/src/main.cpp (a separate, non-bilateral firmware
# for the same physical slave arm) -- NOT re-verified against this bilateral rig, and assumed
# identical on the master arm since it's stated to be the same arm design. Verify before trusting
# absolute link-space angles for anything safety-critical.
JOINTS: dict[str, tuple[str, int, int, float]] = {
    "shoulder_yaw": ("can1", 11, 1, 4.8077),
    "shoulder_roll": ("can1", 12, 2, 1.0),
    "shoulder_pitch": ("can2", 13, 3, 3.180),
    "elbow_pitch": ("can2", 14, 4, 1.0),
}

CAN_BUSES: tuple[str, ...] = ("can1", "can2")


@RobotConfig.register_subclass("forte_arm")
@dataclass
class ForteArmConfig(RobotConfig):
    # One CAN interface per Teensy CAN bus this host is tapped into, e.g. "/dev/ttyUSB0" for an
    # slcan adapter or "can0" for socketcan. Both are required -- shoulder_pitch/elbow_pitch live
    # on can2, shoulder_yaw/shoulder_roll on can1 (see JOINTS above).
    port_can1: str
    port_can2: str

    can_interface: str = "auto"
    use_can_fd: bool = False
    can_bitrate: int = 1_000_000
    can_data_bitrate: int | None = None

    # Robstride motor type shared by all 4 slave joints (affects MIT position/velocity/torque
    # scaling). NOT verified against the motors' actual nameplate model yet.
    motor_type: str = "o0"

    # If True (default), this Robot directly commands the slave motors (torque enable + MIT
    # position control) -- use this for standalone operation, e.g. a trained policy driving the
    # arm with no human/Teensy in the loop.
    #
    # If False, the Teensy's own bilateral firmware already owns actuation of the slave motors
    # (human is driving the master arm, Teensy is running its 500 Hz position-follow + haptic
    # feedback loop). In this mode ForteArm becomes read-only: connect() never enables torque or
    # touches motor config, and send_action() is a no-op (it returns the -- possibly
    # limit-clipped -- action for logging, but never writes to the bus). Use this for
    # `lerobot-record` during bilateral teleoperation, paired with `forte_arm_master` as the
    # teleoperator. Writing to the slave bus while the Teensy also controls it would fight the
    # Teensy's control loop.
    owns_actuation: bool = True

    position_kp: float = 10.0
    position_kd: float = 0.5

    # Per-motor absolute limits in link-space degrees, e.g. {"elbow_pitch": (-10, 120)}.
    # None means unrestricted -- set this before unattended teleoperation/recording/eval.
    # Only enforced when owns_actuation=True (the Teensy has its own separate raw-radian and
    # virtual-wall limits -- see RAW_LIMIT_MIN/MAX and K_WALL in teensy.ino).
    joint_limits: dict[str, tuple[float, float]] | None = None
    # Caps how far a single send_action() step may move a joint from its current position.
    # Only enforced when owns_actuation=True.
    max_relative_target: float | dict[str, float] | None = None

    disable_torque_on_disconnect: bool = True

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
