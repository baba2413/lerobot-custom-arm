import logging
import math
import re
import socket
import threading
import time
from typing import ClassVar, Iterable, Protocol

import serial

logger = logging.getLogger(__name__)


class _PositionSource(Protocol):
    def get_positions_deg(self) -> dict[int, float]: ...


def wait_for_positions(
    link: _PositionSource, motor_ids: Iterable[int], timeout_s: float = 3.0
) -> dict[int, float]:
    """
    Block until `link.get_positions_deg()` has a reading for every id in `motor_ids`, then return
    that snapshot. Works with either `TeensyLink` or `TeensyGoalLink` (both share the
    `get_positions_deg()` shape) via duck typing rather than a shared base class.

    Used by `ForteArmGoal` to establish a per-session position baseline at connect time -- see its
    docstring for why `.pos` is reported delta-from-this-baseline rather than the motor's raw
    absolute reading: the Robstride protocol's own `UNCALIBRATED` fault bit implies the absolute
    reference isn't guaranteed stable across power cycles, so commanding delta-from-session-start
    is robust to that drift by construction, whether or not it actually turns out to matter on
    this hardware. `ForteArm`/`ForteArmMasterTeleop` (bilateral) don't call this anymore -- see
    their docstrings: on `teleop-bi-c` the equivalent zeroing happens firmware-side via `'c'`
    instead, so their own baseline is now fixed at a plain 0.0 rather than dynamically captured.

    Raises TimeoutError if not all ids report within `timeout_s` -- fails loudly rather than
    silently baselining against a partial/wrong snapshot.
    """
    motor_ids = list(motor_ids)
    deadline = time.monotonic() + timeout_s
    positions: dict[int, float] = {}
    while time.monotonic() < deadline:
        positions = link.get_positions_deg()
        if all(m in positions for m in motor_ids):
            return {m: positions[m] for m in motor_ids}
        time.sleep(0.05)
    missing = [m for m in motor_ids if m not in positions]
    raise TimeoutError(
        f"Timed out after {timeout_s}s waiting for initial position feedback for motor id(s) "
        f"{missing} -- is the Teensy powered and running the right firmware?"
    )

# Matches the common prefix of teensy-forte's periodic status line, present on both the
# `teleop-bi` and `teleop-bi-p-t` branches of teensy.ino, e.g.:
#   [CAN1] Master 1 Pos: 0.123 rad | Slave 11 Pos: 0.125 rad | Offset: 0.002 (ENABLED)
#   [CAN2] Master 3 Pos: -0.456 rad | Slave 13 Pos: -0.450 rad | Offset: 0.006 (ENABLED) | SLV Trq: 0.05 Nm | MST FB: -0.02 Nm
# Deliberately doesn't anchor the whole line or require the trailing torque fields, so it matches
# either branch's format.
_STATUS_RE = re.compile(
    r"Master\s+(?P<master_id>\d+)\s+Pos:\s*(?P<master_pos>-?[\d.]+)\s*rad\s*\|\s*"
    r"Slave\s+(?P<slave_id>\d+)\s+Pos:\s*(?P<slave_pos>-?[\d.]+)\s*rad"
)


class TeensyLink:
    """
    Shared serial connection to the Teensy running teensy-forte's bilateral firmware
    (`teleop-bi` / `teleop-bi-p-t` / `teleop-bi-c` branches, `teensy/teensy.ino`). `teleop-bi-c`
    is `teleop-bi-p-t` plus a purely-additive `'c'` command (see `calibrate()`) -- its status line
    keeps the exact same single-number format, so `_STATUS_RE` matches all three branches
    unchanged; `calibrate()` is simply a no-op byte on branches that don't understand `'c'`.

    One physical Teensy exposes
    exactly one USB-serial port, but LeRobot instantiates the slave robot (`ForteArm`) and the
    master teleoperator (`ForteArmMasterTeleop`) independently — both need to read from that same
    port. This class is a per-port singleton with reference-counted connect/disconnect, so both
    callers share one underlying `serial.Serial` instead of fighting over exclusive access to it.
    Construct instances via `TeensyLink.get(port)`, never `TeensyLink(port)` directly, or you'll
    bypass the sharing and get two competing connections.

    A background thread continuously drains the Teensy's periodic
    `[CAN1] Master <id> Pos: <rad> | Slave <id> Pos: <rad> | ...` status lines and caches the
    latest position per motor CAN id, in degrees. `get_positions_deg()` is non-blocking and always
    returns whatever was last parsed (i.e. it's a "latest known", not a fresh synchronous read).

    Known limitation: this firmware only prints status roughly once a second (`LOG_PERIOD` in
    teensy.ino). That's far too slow for real LeRobot dataset recording at typical fps
    (10-30 Hz) — most observations between prints will be an exact repeat of the last one. See
    SMOLVLA_GUIDE.md for the firmware change needed to fix this properly (a faster/on-demand
    report mode); this class works with what the firmware exposes today.

    This class also never writes anything except the single-byte `'e'`/`'d'` enable/disable
    commands — there is no serial command in the current firmware to set a goal position, so
    nothing here can command the arm to move.
    """

    _links: ClassVar[dict[str, "TeensyLink"]] = {}

    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self._serial: serial.Serial | None = None
        self._ref_count = 0
        self._lock = threading.Lock()
        self._positions_deg: dict[int, float] = {}
        self._last_update: dict[int, float] = {}
        self._reader_thread: threading.Thread | None = None
        self._stop_reader = threading.Event()

    @classmethod
    def get(cls, port: str, baudrate: int = 115200) -> "TeensyLink":
        link = cls._links.get(port)
        if link is None:
            link = cls(port, baudrate)
            cls._links[port] = link
        elif link.baudrate != baudrate:
            raise ValueError(
                f"TeensyLink for port '{port}' already exists with baudrate={link.baudrate}, "
                f"got a conflicting baudrate={baudrate}. Both the ForteArm robot and the "
                f"ForteArmMasterTeleop must be configured with the same baudrate."
            )
        return link

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def connect(self) -> None:
        with self._lock:
            if self._serial is None:
                self._serial = serial.Serial(self.port, self.baudrate, timeout=0.2)
                self._stop_reader.clear()
                self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
                self._reader_thread.start()
                logger.info(f"TeensyLink connected on {self.port} @ {self.baudrate}.")
            self._ref_count += 1

    def disconnect(self) -> None:
        with self._lock:
            if self._ref_count == 0:
                return
            self._ref_count -= 1
            should_close = self._ref_count == 0 and self._serial is not None
            if should_close:
                self._stop_reader.set()
        # Join outside the lock -- the reader thread doesn't take it while blocked in readline().
        if should_close:
            if self._reader_thread is not None:
                self._reader_thread.join(timeout=1.0)
                self._reader_thread = None
            if self._serial is not None:
                self._serial.close()
                self._serial = None
            logger.info(f"TeensyLink disconnected from {self.port}.")

    def enable(self) -> None:
        """Send 'e': Teensy enables both arms and (re)computes the master/slave position offset.
        Only send this once both arms are physically posed to match -- see RUNBOOK.md Phase 3."""
        self._write(b"e")

    def disable(self) -> None:
        """Send 'd': Teensy disables both arms."""
        self._write(b"d")

    def calibrate(self) -> None:
        """Send 'c' (`teleop-bi-c` branch only -- `teleop-bi-p-t` doesn't have this command and
        will just ignore the byte). Logging-only zero: captures each motor's current raw position
        so the status line reports relative-to-this-pose from then on. Does not enable, disable,
        or move anything, and does not touch the bilateral offset ('e') or control loop at all."""
        self._write(b"c")

    def _write(self, data: bytes) -> None:
        if self._serial is None:
            raise ConnectionError(f"TeensyLink({self.port}) is not connected.")
        self._serial.write(data)

    def get_positions_deg(self) -> dict[int, float]:
        """Latest known position per CAN motor id, in degrees. Non-blocking; may be stale -- see
        age_s()."""
        with self._lock:
            return dict(self._positions_deg)

    def age_s(self, motor_id: int) -> float | None:
        """Seconds since the last update for this motor id, or None if never received."""
        with self._lock:
            last = self._last_update.get(motor_id)
        return None if last is None else time.monotonic() - last

    def _reader_loop(self) -> None:
        assert self._serial is not None
        while not self._stop_reader.is_set():
            try:
                raw = self._serial.readline()
            except serial.SerialException:
                break
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore")
            match = _STATUS_RE.search(line)
            if not match:
                continue
            now = time.monotonic()
            master_id = int(match["master_id"])
            slave_id = int(match["slave_id"])
            master_deg = math.degrees(float(match["master_pos"]))
            slave_deg = math.degrees(float(match["slave_pos"]))
            with self._lock:
                self._positions_deg[master_id] = master_deg
                self._positions_deg[slave_id] = slave_deg
                self._last_update[master_id] = now
                self._last_update[slave_id] = now


# Matches the standalone GOAL-following firmware's status line (`goal` branch of teensy-forte),
# e.g.:
#   [CAN1] Slave 11 Pos: 0.123 (0.123) rad | Target: 0.125 (0.125) rad (GOAL / CALIB_OK) | Trq: 0.05 Nm
# Pos/Target are printed as "radian_after_calibration (original_radian)" -- this only needs the
# raw (parenthesized) value, since TeensyGoalLink works in raw motor-shaft units throughout (see
# send_goal()'s docstring). Only motor_id/pos are captured -- Target and the mode/calib flags
# aren't consumed anywhere in this class, and capturing them added fragility for no benefit: an
# earlier version of this regex tried to also capture the mode word and silently stopped matching
# at all the moment the status line grew a second "/ CALIB_OK" flag, since `\w+` can't span the
# space and slash -- caught while making this change, not by anything noticing it break.
# This firmware has no master arm, so this is deliberately a different regex from _STATUS_RE
# rather than stretching that one (or having the firmware print fake master data) to fit.
_GOAL_STATUS_RE = re.compile(
    r"Slave\s+(?P<motor_id>\d+)\s+Pos:\s*-?[\d.]+\s*\(\s*(?P<pos>-?[\d.]+)\s*\)\s*rad"
)

# Wire order the GOAL firmware expects in a UDP goal packet ("<yaw>,<pitch>,<roll>,<elbow>"):
# kinematic order, NOT CAN-bus wiring order -- teensy.ino's SLV_IDS_CAN1={11,12}/
# SLV_IDS_CAN2={13,14} groups yaw+roll together and pitch+elbow together, but
# handleGoalPacket() unswaps 12/13 so the wire payload reads yaw, pitch, roll, elbow. Kept as
# its own constant rather than importing JOINTS -- like TeensyLink, this module stays
# gear-ratio- and joint-name-agnostic; that mapping is a robot.py-layer concern.
GOAL_MOTOR_ORDER: tuple[int, int, int, int] = (11, 13, 12, 14)  # yaw, pitch, roll, elbow


class TeensyGoalLink:
    """
    Connection to the Teensy running the standalone GOAL-following firmware (`goal` branch of
    teensy-forte, single slave arm, no master/bilateral logic -- see that branch's teensy.ino
    header comment). Deliberately a separate class from `TeensyLink`, not a variant of it: the
    wire protocol differs entirely, and since this firmware has no master arm there is only ever
    one Python-side consumer during eval, so this class owns its connections directly instead of
    reference-counting like `TeensyLink` has to.

    Hybrid transport, matching the firmware's split (referenced from teensy-forte's isaacsim-udp
    branch, which solved the same "one channel, two jobs" problem we hit when goal-streaming
    briefly lived on serial -- see git history on the `goal` branch for what that cost):
      - USB serial: `calibrate()` ('c') / `disable()` ('d'), single-char, human-supervised. Also
        the source of `get_positions_deg()`'s telemetry (the firmware's periodic status line).
      - Ethernet UDP: `send_goal()`, the continuous goal-position stream. Fire-and-forget
        datagrams to (UDP_HOST, UDP_PORT) -- no connection/handshake/delivery guarantee, matching
        the firmware's stateless, watchdog-protected design (GOAL_TIMEOUT_MS = 500ms in
        teensy.ino): a dropped packet just means the next one takes over.

    Like `TeensyLink`, this class works entirely in raw motor-shaft units (radians for commands,
    degrees for `get_positions_deg()`, matching that class's convention) and knows nothing about
    gear ratios or joint names -- callers convert via `config.JOINTS` before calling `send_goal()`.
    """

    UDP_HOST = "192.168.1.15"  # matches teensy.ino's static IP (staticIP, direct-cable setup)
    UDP_PORT = 5005

    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self._serial: serial.Serial | None = None
        self._udp_sock: socket.socket | None = None
        self._lock = threading.Lock()
        self._positions_deg: dict[int, float] = {}
        self._last_update: dict[int, float] = {}
        self._reader_thread: threading.Thread | None = None
        self._stop_reader = threading.Event()

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open and self._udp_sock is not None

    def connect(self) -> None:
        if self._serial is not None:
            return
        self._serial = serial.Serial(self.port, self.baudrate, timeout=0.2)
        self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._stop_reader.clear()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()
        logger.info(
            f"TeensyGoalLink connected: serial {self.port} @ {self.baudrate}, "
            f"UDP -> {self.UDP_HOST}:{self.UDP_PORT}."
        )

    def disconnect(self) -> None:
        if self._serial is None:
            return
        self._stop_reader.set()
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.0)
            self._reader_thread = None
        self._serial.close()
        self._serial = None
        if self._udp_sock is not None:
            self._udp_sock.close()
            self._udp_sock = None
        logger.info(f"TeensyGoalLink disconnected from {self.port}.")

    def calibrate(self) -> None:
        """Send 'c' over serial: capture the arm's current pose as each motor's software zero
        (offset only, no physical move). Needed for the firmware's per-joint limit clamp to be
        enforced -- see RUNBOOK.md Phase 9."""
        self._write_serial(b"c")

    def disable(self) -> None:
        """Send 'd' over serial: stop and disable. Also clears calibration on the firmware side --
        call calibrate() again before the next send_goal() if you want the limit clamp back."""
        self._write_serial(b"d")

    def enable(self) -> None:
        """Enter GOAL mode holding the arm's current position -- the UDP-era equivalent of the old
        bare serial 'g'. There's no "hold current position" concept in the firmware's UDP protocol
        (every packet must carry 4 real values), so this reads the last known position from
        get_positions_deg() and sends that straight back as the first goal. Requires at least one
        status line to have arrived first (i.e. call this after connect(), not immediately at it)."""
        positions = self.get_positions_deg()
        missing = [m for m in GOAL_MOTOR_ORDER if m not in positions]
        if missing:
            raise RuntimeError(
                f"TeensyGoalLink({self.port}): no known position yet for motor id(s) {missing} -- "
                "wait for at least one status line (printed ~1x/second) before calling enable()."
            )
        self.send_goal(positions)

    def send_goal(self, positions_deg: dict[int, float]) -> None:
        """
        Send one goal-position UDP packet. `positions_deg` must have exactly the four keys in
        GOAL_MOTOR_ORDER (raw motor-shaft degrees, not gear-adjusted -- same convention as
        get_positions_deg()). Also enters GOAL mode if not already in it (see teensy.ino's
        handleGoalPacket()), so calling this alone is enough to both arm and start moving the arm.

        Call this at your control-loop rate (e.g. once per policy inference step) -- the firmware
        auto-disables if it doesn't see a new goal packet within GOAL_TIMEOUT_MS (500ms). UDP is
        fire-and-forget: this does not block and does not confirm delivery.
        """
        if self._udp_sock is None:
            raise ConnectionError(f"TeensyGoalLink({self.port}) is not connected.")
        values_rad = [math.radians(positions_deg[motor_id]) for motor_id in GOAL_MOTOR_ORDER]
        payload = ",".join(f"{v:.4f}" for v in values_rad)
        self._udp_sock.sendto(payload.encode("ascii"), (self.UDP_HOST, self.UDP_PORT))

    def _write_serial(self, data: bytes) -> None:
        if self._serial is None:
            raise ConnectionError(f"TeensyGoalLink({self.port}) is not connected.")
        self._serial.write(data)

    def get_positions_deg(self) -> dict[int, float]:
        """Latest known position per CAN motor id, in degrees. Non-blocking; may be stale -- see
        age_s(). Sourced from the firmware's periodic serial status line, independent of the UDP
        goal stream."""
        with self._lock:
            return dict(self._positions_deg)

    def age_s(self, motor_id: int) -> float | None:
        """Seconds since the last update for this motor id, or None if never received."""
        with self._lock:
            last = self._last_update.get(motor_id)
        return None if last is None else time.monotonic() - last

    def _reader_loop(self) -> None:
        assert self._serial is not None
        while not self._stop_reader.is_set():
            try:
                raw = self._serial.readline()
            except serial.SerialException:
                break
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore")
            match = _GOAL_STATUS_RE.search(line)
            if not match:
                continue
            now = time.monotonic()
            motor_id = int(match["motor_id"])
            pos_deg = math.degrees(float(match["pos"]))
            with self._lock:
                self._positions_deg[motor_id] = pos_deg
                self._last_update[motor_id] = now
