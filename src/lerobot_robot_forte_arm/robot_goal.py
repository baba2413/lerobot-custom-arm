import logging
import time
from functools import cached_property
from typing import Any

from lerobot.cameras import make_cameras_from_configs
from lerobot.robots import Robot
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .config import JOINTS, ForteArmGoalConfig
from .teensy_link import TeensyGoalLink, wait_for_positions

logger = logging.getLogger(__name__)


class ForteArmGoal(Robot):
    """
    Forte arm slave, driven by the standalone GOAL-following firmware (`goal` branch of
    teensy-forte -- single arm, no master, no bilateral teleop; see that branch's teensy.ino
    header comment). Point this at that firmware only -- it speaks a different wire protocol
    than `teleop-bi-p-t`, which `ForteArm` talks to.

    Unlike `ForteArm.send_action()` (a no-op, since the bilateral firmware has no goal-position
    command -- see that class's docstring), `send_action()` here actually commands the arm via
    `TeensyGoalLink.send_goal()` -- over Ethernet UDP, not serial (see that class's docstring for
    the hybrid transport split). This is what Phase 9 eval / trained-policy control needs.

    The firmware auto-disables if it doesn't see a new goal packet within GOAL_TIMEOUT_MS (500ms
    in teensy.ino), so `send_action()` must be called at least that often -- normal for a policy
    control loop, but a `lerobot-record`/`lerobot-eval` pause (e.g. a breakpoint) will stop the
    arm mid-motion rather than leave it holding a stale command.

    `observation_features`/`action_features` `.pos` values are raw motor-shaft degrees, matching
    `ForteArm`/`ForteArmMasterTeleop` exactly (no gear-ratio conversion anywhere in this pipeline)
    -- required so a policy trained on `ForteArm`'s recordings sees the same units here at eval
    time. See ForteArm's docstring for why gear ratios aren't applied at all, and for why values
    are also delta-from-this-session's-baseline rather than the motor's raw absolute reading
    (`get_observation()` subtracts it; `send_action()` adds it back before calling
    `TeensyGoalLink.send_goal()`, which still speaks absolute raw radians over UDP unchanged --
    delta-vs-absolute is a host-side representation choice, not a wire-protocol one). This only
    works if the arm is posed the same way at connect() as it was for the recordings a policy was
    trained on -- match `ForteArm`'s starting pose, not wherever `goal` firmware's `'c'`
    calibration (a separate, firmware-side concept for the safety clamp) happens to be centered.
    """

    config_class = ForteArmGoalConfig
    name = "forte_arm_goal"

    def __init__(self, config: ForteArmGoalConfig):
        super().__init__(config)
        self.config = config
        self.link = TeensyGoalLink(config.port, config.baudrate)
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
        logger.info(f"Connecting {self} to Teensy on {self.config.port} (GOAL firmware)...")
        self.link.connect()
        for cam in self.cameras.values():
            cam.connect()
        self._baseline_deg = wait_for_positions(self.link, self._slave_id.values())
        self._connected = True
        logger.info(f"{self} connected. Baseline: {self._baseline_deg}")

    @check_if_not_connected
    def disconnect(self) -> None:
        self.link.disable()  # stop the arm before dropping the connection
        self.link.disconnect()
        for cam in self.cameras.values():
            cam.disconnect()
        self._connected = False
        logger.info(f"{self} disconnected.")

    @property
    def is_calibrated(self) -> bool:
        # Not applicable: Robstride motors report absolute position directly. Note this is the
        # standard LeRobot Robot ABC lifecycle hook, unrelated to the firmware's own 'c'
        # zero-calibration -- see calibrate_zero() for that.
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def calibrate_zero(self) -> None:
        """Send 'c' (serial): capture the arm's current pose as each motor's software zero.
        Needed for the firmware's per-joint limit clamp to be enforced -- see RUNBOOK.md Phase 9.
        Not the same as calibrate() above (that's LeRobot's own ABC hook, a no-op here)."""
        self.link.calibrate()

    def enable(self) -> None:
        """Enter GOAL mode holding the arm's current position -- sends that position as the first
        UDP goal packet (see TeensyGoalLink.enable()'s docstring for why; the firmware's UDP
        protocol has no "hold current position" concept, unlike the old serial 'g'). Not required
        before send_action() -- the first send_action() call enters GOAL mode itself -- but useful
        to arm the motors deliberately (e.g. right after posing the arm) before streaming starts.
        Requires get_observation() to have run at least once first, so the link has a known
        position to send."""
        self.link.enable()

    def disable(self) -> None:
        """Send 'd' (serial): stop and disable."""
        self.link.disable()

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        start = time.perf_counter()

        positions = self.link.get_positions_deg()
        obs_dict: dict[str, Any] = {}
        for joint, slave_id in self._slave_id.items():
            if slave_id not in positions:
                logger.warning(f"{self}: no data yet from motor {slave_id} ({joint}).")
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
        # action[f"{joint}.pos"] is delta-from-baseline (no gear conversion either, see class
        # docstring) -- add this session's baseline back to get the absolute raw motor degrees
        # TeensyGoalLink.send_goal() actually sends over UDP.
        positions_deg = {
            self._slave_id[joint]: self._baseline_deg[self._slave_id[joint]] + action[f"{joint}.pos"]
            for joint in JOINTS
            if f"{joint}.pos" in action
        }
        self.link.send_goal(positions_deg)
        return {key: val for key, val in action.items() if key.endswith(".pos")}
