import logging
import time
from functools import cached_property
from typing import Any

from lerobot.cameras import make_cameras_from_configs
from lerobot.robots import Robot
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .config import JOINTS, ForteArmConfig
from .teensy_link import TeensyLink

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

    `self._baseline_deg` is fixed at a plain **0.0** for every motor -- not dynamically captured
    (that's what an earlier version of this class did via `wait_for_positions()`; see
    `teensy_link.py`'s history for why that got reverted here). Recorded `.pos` is therefore
    whatever the Teensy's status line reports, unmodified. On `teleop-bi-c` that's already
    zero-relative -- send `'c'` (see `calibrate_zero()`) once, and the firmware itself subtracts
    each motor's `'c'`-time position before printing (logging-only on that branch, see its
    `teensy.ino` header -- does not touch the bilateral offset or control loop at all). On
    `teleop-bi` / `teleop-bi-p-t` (no `'c'` support), `'c'` is just an ignored byte and you'll get
    raw absolute values with no warning, since the status line's format is identical either way
    and there's no way to tell from here which firmware is actually flashed.
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
        self._baseline_deg = dict.fromkeys(self._slave_id.values(), 0.0)
        self._connected = True
        logger.info(f"{self} connected (read-only -- see class docstring).")

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

    def calibrate_zero(self) -> None:
        """Send 'c' (`teleop-bi-c` branch only): logging-only zero -- the status line reports
        relative-to-this-pose from then on, which is what makes recorded `.pos` values delta
        instead of raw absolute (see class docstring). Does not enable, disable, or move anything.
        A no-op byte if `teleop-bi`/`teleop-bi-p-t` is flashed instead."""
        self.link.calibrate()

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
