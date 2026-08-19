import logging
from dataclasses import dataclass

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.robstride.robstride import RobstrideMotorsBus
from lerobot.teleoperators import Teleoperator, TeleoperatorConfig
from lerobot.types import RobotAction
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .config import CAN_BUSES, JOINTS

logger = logging.getLogger(__name__)

_PORT_BY_BUS_FIELD = {"can1": "port_can1", "can2": "port_can2"}


@TeleoperatorConfig.register_subclass("forte_arm_master")
@dataclass
class ForteArmMasterTeleopConfig(TeleoperatorConfig):
    """
    Reads the Forte rig's human-driven MASTER arm. Read-only by design: the Teensy
    (teensy-forte, `teleop-bi-p-t` branch) already owns torque/haptic feedback control of the
    master motors as part of its bilateral loop -- this class must never write to them.
    """

    # One CAN interface per Teensy CAN bus this host is tapped into. Same physical buses the
    # ForteArm slave robot connects to (see config.JOINTS) -- master and slave motors for a given
    # pair share a bus.
    port_can1: str
    port_can2: str

    can_interface: str = "auto"
    use_can_fd: bool = False
    can_bitrate: int = 1_000_000
    can_data_bitrate: int | None = None

    # Assumed identical to the slave's motor type -- NOT verified.
    motor_type: str = "o0"


class ForteArmMasterTeleop(Teleoperator):
    config_class = ForteArmMasterTeleopConfig
    name = "forte_arm_master"

    def __init__(self, config: ForteArmMasterTeleopConfig):
        super().__init__(config)
        self.config = config

        joints_by_bus: dict[str, dict[str, Motor]] = {bus: {} for bus in CAN_BUSES}
        self._gear_ratio: dict[str, float] = {}
        self._bus_of_joint: dict[str, str] = {}
        for joint_name, (bus, _slave_id, master_id, gear_ratio) in JOINTS.items():
            joints_by_bus[bus][joint_name] = Motor(
                id=master_id,
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

    @property
    def action_features(self) -> dict:
        return {f"{joint}.pos": float for joint in JOINTS}

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return all(bus.is_connected for bus in self.buses.values())

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        logger.info(
            f"Connecting {self} (can1={self.config.port_can1}, can2={self.config.port_can2}) "
            "-- read-only, never enables torque on the master motors."
        )
        for bus in self.buses.values():
            bus.connect()

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        action: RobotAction = {}
        for bus in self.buses.values():
            motor_positions = bus.sync_read("Present_Position")
            action.update(
                {f"{motor}.pos": pos / self._gear_ratio[motor] for motor, pos in motor_positions.items()}
            )
        return action

    def send_feedback(self, feedback: dict) -> None:
        # Haptic feedback to the master is already driven by the Teensy's own bilateral loop.
        pass

    @check_if_not_connected
    def disconnect(self) -> None:
        for bus in self.buses.values():
            bus.disconnect(disable_torque=False)
        logger.info(f"{self} disconnected.")
