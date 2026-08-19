import logging
import time
from functools import cached_property
from typing import Any

from lerobot.cameras import make_cameras_from_configs
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.robstride.robstride import RobstrideMotorsBus
from lerobot.robots import Robot
from lerobot.robots.utils import ensure_safe_goal_position
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .config import CAN_BUSES, JOINTS, ForteArmConfig

logger = logging.getLogger(__name__)

_PORT_BY_BUS_FIELD = {"can1": "port_can1", "can2": "port_can2"}


class ForteArm(Robot):
    """
    Forte arm slave (follower). Only the 4 currently-motorized joints (shoulder_yaw,
    shoulder_pitch, shoulder_roll, elbow_pitch) are exposed here; lower_arm_roll, wrist_pitch and
    the gripper aren't wired up yet.

    Motors are Robstride CAN servos in MIT mode, split across 2 physical CAN buses (see JOINTS in
    config.py). RobstrideMotorsBus reports/accepts positions in degrees at the motor shaft, so
    positions are converted to/from link-space degrees using each joint's external gear ratio.

    A separate Teensy (teensy-forte, `teleop-bi-p-t` branch) already runs a full bilateral
    master/slave teleoperation loop against these same motors, independent of this class. See
    `config.owns_actuation` and SMOLVLA_GUIDE.md for how the two coexist.
    """

    config_class = ForteArmConfig
    name = "forte_arm"

    def __init__(self, config: ForteArmConfig):
        super().__init__(config)
        self.config = config

        joints_by_bus: dict[str, dict[str, Motor]] = {bus: {} for bus in CAN_BUSES}
        self._gear_ratio: dict[str, float] = {}
        self._bus_of_joint: dict[str, str] = {}
        for joint_name, (bus, slave_id, _master_id, gear_ratio) in JOINTS.items():
            joints_by_bus[bus][joint_name] = Motor(
                id=slave_id,
                model="robstride",
                norm_mode=MotorNormMode.DEGREES,
                motor_type_str=config.motor_type,
            )
            self._gear_ratio[joint_name] = gear_ratio
            self._bus_of_joint[joint_name] = bus

        self.buses: dict[str, RobstrideMotorsBus] = {
            bus: RobstrideMotorsBus(
                port=getattr(self.config, _PORT_BY_BUS_FIELD[bus]),
                motors=motors,
                calibration=self.calibration,
                can_interface=self.config.can_interface,
                use_can_fd=self.config.use_can_fd,
                bitrate=self.config.can_bitrate,
                data_bitrate=self.config.can_data_bitrate,
            )
            for bus, motors in joints_by_bus.items()
        }

        self.cameras = make_cameras_from_configs(config.cameras)

    def _bus_for(self, joint_name: str) -> RobstrideMotorsBus:
        return self.buses[self._bus_of_joint[joint_name]]

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{joint}.pos": float for joint in JOINTS}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return all(bus.is_connected for bus in self.buses.values()) and all(
            cam.is_connected for cam in self.cameras.values()
        )

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        logger.info(f"Connecting {self} (can1={self.config.port_can1}, can2={self.config.port_can2})...")
        for bus in self.buses.values():
            bus.connect()

        if self.config.owns_actuation:
            if not self.is_calibrated and calibrate:
                self.calibrate()
        else:
            logger.info(
                f"{self} owns_actuation=False: skipping torque enable / calibration -- "
                "the Teensy bilateral firmware owns the slave motors."
            )

        for cam in self.cameras.values():
            cam.connect()

        if self.config.owns_actuation:
            self.configure()

        logger.info(f"{self} connected.")

    @check_if_not_connected
    def disconnect(self) -> None:
        disable_torque = self.config.disable_torque_on_disconnect and self.config.owns_actuation
        for bus in self.buses.values():
            bus.disconnect(disable_torque)
        for cam in self.cameras.values():
            cam.disconnect()
        logger.info(f"{self} disconnected.")

    @property
    def is_calibrated(self) -> bool:
        return all(bus.is_calibrated for bus in self.buses.values())

    def calibrate(self) -> None:
        # Robstride motors report absolute shaft position directly (no incremental homing needed),
        # so "calibration" here is just a trivial per-motor range record to satisfy the Robot
        # interface, not a real offset computation. (The Teensy computes its own master/slave
        # offset separately and independently, on 'e' / enable.)
        logger.info(f"Calibrating {self}: recording a [-180, 180] deg range per motor.")
        for bus in self.buses.values():
            for motor_name, motor in bus.motors.items():
                self.calibration[motor_name] = MotorCalibration(
                    id=motor.id,
                    drive_mode=0,
                    homing_offset=0,
                    range_min=-180,
                    range_max=180,
                )
            bus.write_calibration(self.calibration)
        self._save_calibration()

    def configure(self) -> None:
        if not self.config.owns_actuation:
            return
        for bus in self.buses.values():
            with bus.torque_disabled():
                bus.configure_motors()
            bus.sync_write("Kp", dict.fromkeys(bus.motors, self.config.position_kp))
            bus.sync_write("Kd", dict.fromkeys(bus.motors, self.config.position_kd))

    def _read_link_positions(self) -> dict[str, float]:
        link_pos: dict[str, float] = {}
        for bus in self.buses.values():
            motor_positions = bus.sync_read("Present_Position")
            link_pos.update({motor: pos / self._gear_ratio[motor] for motor, pos in motor_positions.items()})
        return link_pos

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        start = time.perf_counter()

        obs_dict: dict[str, Any] = {f"{joint}.pos": pos for joint, pos in self._read_link_positions().items()}

        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} get_observation took: {dt_ms:.1f}ms")
        return obs_dict

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}

        if self.config.joint_limits is not None:
            for joint_name, position in goal_pos.items():
                if joint_name in self.config.joint_limits:
                    min_limit, max_limit = self.config.joint_limits[joint_name]
                    goal_pos[joint_name] = max(min_limit, min(max_limit, position))

        if not self.config.owns_actuation:
            # The Teensy bilateral firmware already drives these motors from the master arm.
            # Writing our own Goal_Position here would fight its 500 Hz control loop, so we only
            # report back what the (possibly limit-clipped) action was, for logging.
            return {f"{joint}.pos": val for joint, val in goal_pos.items()}

        if self.config.max_relative_target is not None:
            present_pos = self._read_link_positions()
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        for bus in self.buses.values():
            motor_goal_pos = {
                joint: pos * self._gear_ratio[joint] for joint, pos in goal_pos.items() if joint in bus.motors
            }
            if motor_goal_pos:
                bus.sync_write("Goal_Position", motor_goal_pos)

        return {f"{joint}.pos": val for joint, val in goal_pos.items()}
