import logging
import math
import re
import threading
import time
from typing import ClassVar

import serial

logger = logging.getLogger(__name__)

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
    (`teleop-bi` / `teleop-bi-p-t` branches, `teensy/teensy.ino`). One physical Teensy exposes
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
