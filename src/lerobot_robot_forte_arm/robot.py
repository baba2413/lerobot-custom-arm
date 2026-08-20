import logging
import time
from functools import cached_property
from typing import Any

from lerobot.cameras import make_cameras_from_configs
from lerobot.robots import Robot
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .config import JOINTS, ForteArmConfig
from .teensy_link import TeensyLink, wait_for_positions

logger = logging.getLogger(__name__)


class ForteArm(Robot):
    """
    Forte arm slave (follower), observed over the Teensy's single serial port.

    IMPORTANT: this class cannot move the arm. The current teensy-forte firmware (`teleop-bi` /
    `teleop-bi-p-t` branches) only accepts `'e'`/`'d'` (enable/disable) over serial -- there is no
    serial command to set a goal position. All actuation of the slave happens inside the Teensy's
    own bilateral control loop, driven by the master arm. `send_action()` is therefore a
    logging-only no-op: it satisfies `lerobot-record`'s interface (which always calls it every
    frame) without writing anything, since there's nothing it could write to.

    Standalone closed-loop policy control (a trained policy directly commanding the slave with no
    human on the master) is NOT possible with the firmware as it stands today -- that needs a new
    goal-position serial command added to teensy.ino first. See SMOLVLA_GUIDE.md.

    `observation_features`'/`action_features` `.pos` values are raw motor-shaft degrees (whatever
    the Teensy's CAN feedback reports), not gear-adjusted joint/link degrees. JOINTS' external gear
    ratios describe the real hardware but are deliberately not applied here: this pipeline only
    ever needs to record what a motor did and later reproduce it on the same motor, and a trained
    policy doesn't care whether that number is "physically real" degrees, only that recording and
    eval agree -- so there's no reason to add a unit conversion whose only real effect would be
    another way for a train/eval mismatch to sneak in (e.g. via SMOLVLA_GUIDE.md's still-unverified
    assumption that master and slave gear ratios even match).

    Values are also **delta from this session's connect()-time baseline**, not the motor's raw
    absolute reading -- the Robstride protocol's own `UNCALIBRATED` fault bit implies the absolute
    reference isn't guaranteed stable across power cycles, so a dataset recorded across multiple
    sessions in raw absolute terms could have the same `.pos` number mean a different physical
    angle in different episodes. Delta-from-baseline is robust to that by construction: it only
    requires the motor's rotation-to-radians *scale* to be stable (safe to assume), not its
    absolute zero. See `wait_for_positions()` in teensy_link.py. This only works if the arm is
    physically posed the same way at the start of every session before connect() -- delta cancels
    an absolute-reference shift, not a genuinely different starting pose.
    """

    config_class = ForteArmConfig
    name = "forte_arm"

    def __init__(self, config: ForteArmConfig):
        super().__init__(config)
        self.config = config
        self.link = TeensyLink.get(config.port, config.baudrate)
        self._slave_id = {joint: slave_id for joint, (slave_id, _master_id, _ratio) in JOINTS.items()}
        self.cameras = make_cameras_from_configs(config.cameras)
        self._connected = False
        self._baseline_deg: dict[int, float] = {}

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
        return self._connected and all(cam.is_connected for cam in self.cameras.values())

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        logger.info(f"Connecting {self} to Teensy on {self.config.port}...")
        self.link.connect()
        for cam in self.cameras.values():
            cam.connect()
        self._baseline_deg = wait_for_positions(self.link, self._slave_id.values())
        self._connected = True
        logger.info(f"{self} connected (read-only -- see class docstring). Baseline: {self._baseline_deg}")

    @check_if_not_connected
    def disconnect(self) -> None:
        self.link.disconnect()
        for cam in self.cameras.values():
            cam.disconnect()
        self._connected = False
        logger.info(f"{self} disconnected.")

    @property
    def is_calibrated(self) -> bool:
        # Not applicable: Robstride motors report absolute position directly, and the Teensy
        # computes its own master/slave offset independently of anything on the host.
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def enable(self) -> None:
        """Send 'e' to the Teensy: enable both arms and (re)compute the master/slave offset. Only
        call this once both arms are physically posed to match -- see RUNBOOK.md Phase 3."""
        self.link.enable()

    def disable(self) -> None:
        """Send 'd' to the Teensy: disable both arms."""
        self.link.disable()

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        start = time.perf_counter()

        positions = self.link.get_positions_deg()
        obs_dict: dict[str, Any] = {}
        for joint, slave_id in self._slave_id.items():
            if slave_id not in positions:
                logger.warning(f"{self}: no data yet from slave motor {slave_id} ({joint}).")
                continue
            age = self.link.age_s(slave_id)
            if age is not None and age > self.config.stale_after_s:
                logger.warning(f"{self}: {joint} position is {age:.1f}s old (Teensy not reporting?).")
            # delta from this session's connect()-time baseline, not raw absolute -- see class docstring
            obs_dict[f"{joint}.pos"] = positions[slave_id] - self._baseline_deg[slave_id]

        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} get_observation took: {dt_ms:.1f}ms")
        return obs_dict

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        # No-op: see class docstring. Only echoes back what "would have" been sent, for logging.
        return {key: val for key, val in action.items() if key.endswith(".pos")}
