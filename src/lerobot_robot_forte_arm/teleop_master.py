import logging
from dataclasses import dataclass

from lerobot.teleoperators import Teleoperator, TeleoperatorConfig
from lerobot.types import RobotAction
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .config import JOINTS
from .teensy_link import TeensyLink

logger = logging.getLogger(__name__)


@TeleoperatorConfig.register_subclass("forte_arm_master")
@dataclass
class ForteArmMasterTeleopConfig(TeleoperatorConfig):
    """
    Reads the Forte rig's human-driven MASTER arm over the same Teensy serial port the paired
    `ForteArm` robot uses. Pass the identical `port` (and `baudrate`) as that robot's config --
    `TeensyLink` shares one underlying serial connection per port, so both end up talking to the
    same open connection instead of fighting over exclusive access to it.
    """

    port: str
    baudrate: int = 115200
    stale_after_s: float = 3.0


class ForteArmMasterTeleop(Teleoperator):
    """
    Read-only by design: the Teensy already owns torque/haptic-feedback control of the master
    motors as part of its bilateral loop. This class never writes anything to the bus (there's no
    serial command for it to use even if it wanted to -- see ForteArm's docstring).

    `action_features` values are raw motor-shaft degrees (whatever the Teensy's CAN feedback
    reports), not gear-adjusted joint/link degrees -- JOINTS' external gear ratios are recorded as
    hardware documentation only and are not applied anywhere in this pipeline. See ForteArm's
    docstring for why, and for why `self._baseline_deg` is a fixed 0.0 rather than dynamically
    captured -- on `teleop-bi-c`, `'c'` (`calibrate_zero()`) makes the firmware itself report
    zero-relative positions, so nothing needs subtracting host-side.
    """

    config_class = ForteArmMasterTeleopConfig
    name = "forte_arm_master"

    def __init__(self, config: ForteArmMasterTeleopConfig):
        super().__init__(config)
        self.config = config
        self.link = TeensyLink.get(config.port, config.baudrate)
        self._master_id = {joint: master_id for joint, (_slave_id, master_id, _ratio) in JOINTS.items()}
        self._connected = False
        self._baseline_deg: dict[int, float] = {}

    @property
    def action_features(self) -> dict:
        return {f"{joint}.pos": float for joint in JOINTS}

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._connected

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        logger.info(f"Connecting {self} to Teensy on {self.config.port} (read-only)...")
        self.link.connect()
        self._baseline_deg = dict.fromkeys(self._master_id.values(), 0.0)
        self._connected = True
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def calibrate_zero(self) -> None:
        """Send 'c' (`teleop-bi-c` branch only): logging-only zero -- see ForteArm's docstring/
        `calibrate_zero()`. Shares `TeensyLink` with `ForteArm`, so calling it from either object
        has the same effect; only needs calling once per session, not from both."""
        self.link.calibrate()

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        positions = self.link.get_positions_deg()
        action: RobotAction = {}
        for joint, master_id in self._master_id.items():
            if master_id not in positions:
                logger.warning(f"{self}: no data yet from master motor {master_id} ({joint}).")
                continue
            age = self.link.age_s(master_id)
            if age is not None and age > self.config.stale_after_s:
                logger.warning(f"{self}: {joint} position is {age:.1f}s old (Teensy not reporting?).")
            # delta from this session's connect()-time baseline, not raw absolute -- see class docstring
            action[f"{joint}.pos"] = positions[master_id] - self._baseline_deg[master_id]
        return action

    def send_feedback(self, feedback: dict) -> None:
        # Haptic feedback to the master is already driven by the Teensy's own bilateral loop.
        pass

    @check_if_not_connected
    def disconnect(self) -> None:
        self.link.disconnect()
        self._connected = False
        logger.info(f"{self} disconnected.")
