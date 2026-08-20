#!/usr/bin/env python
"""
Manual bench test for the `goal` firmware's per-joint limit clamp (teensy-forte `goal` branch,
teensy.ino's JOINT_LIMIT_MIN/MAX_CAN1/CAN2 + the 'c' zero-calibration).

Slowly ramps ONE motor's target in ONE direction (see MOTOR_ID/DIRECTION below), holding the
other three motors fixed at their last-known position, until the firmware's own
"[CANx JOINT LIMIT] ... clamped to [lo, hi]" line is seen for that motor -- confirming the clamp
is actually engaging on real hardware, not just in the firmware source. Once seen, keeps ramping a
bit further (OVERSHOOT_RAD) past the reported bound, then stops and disables.

Does NOT hardcode the limit values here -- it reads them off the firmware's own log line, so this
stays correct even if JOINT_LIMIT_MIN/MAX_CAN1/CAN2 change. If the arm isn't calibrated ('c'), no
per-joint clamp is active at all (only the wide +-12.4 rad protocol limit) -- MAX_RAMP_RAD below is
a hard stop for exactly that case, so this script can't ramp indefinitely into the arm's physical
range if you forgot to calibrate first.

Transport matches the firmware's hybrid split: goal-position targets go out over Ethernet UDP
(matching TeensyGoalLink.send_goal(), see teensy_link.py), but this script reads the serial port
directly rather than going through TeensyGoalLink -- that class's reader thread only extracts
position telemetry and silently discards the WARN/CALIB/JOINT LIMIT diagnostic lines this script
specifically needs to see. Serial is otherwise only used for 'd' (disable) at the end -- 'c'
(calibrate) is not sent by this script; do that yourself first if you want the clamp active (see
RUNBOOK.md Phase 9).

Usage:
    uv run python src/lerobot_robot_forte_arm/goal_limit_bench.py --port /dev/ttyACM0
"""

import argparse
import re
import socket
import sys
import threading
import time

import serial

from lerobot_robot_forte_arm.teensy_link import GOAL_MOTOR_ORDER, TeensyGoalLink

# ---------------------------------------------------------------------------
# Edit these between runs.
# ---------------------------------------------------------------------------
MOTOR_ID = 11  # which slave motor to ramp: 11=shoulder_yaw, 12=shoulder_roll, 13=shoulder_pitch, 14=elbow_pitch
DIRECTION = -1  # +1 or -1 -- which way to move MOTOR_ID
STEP_RAD = 0.01  # how much to change MOTOR_ID's target per tick
STEP_PERIOD_S = 0.05  # seconds between ticks -- keep well under teensy.ino's GOAL_TIMEOUT_MS (500ms)
OVERSHOOT_RAD = 0.1  # once clamped, keep ramping this far past the reported bound before stopping
MAX_RAMP_RAD = 3.0  # hard stop: give up if we've moved this far from the start with no clamp seen
# ---------------------------------------------------------------------------

# Firmware prints "Pos: radian_after_calibration (original_radian) rad" -- capture the raw
# (parenthesized) value specifically, since send_goal() operates in raw motor-shaft radians.
_STATUS_RE = re.compile(r"Slave\s+(\d+)\s+Pos:\s*-?[\d.]+\s*\(\s*(-?[\d.]+)\s*\)\s*rad")
_CLAMP_RE = re.compile(
    r"\[CAN[12] JOINT LIMIT\] Motor (\d+) target (-?[\d.]+) rad clamped to \[(-?[\d.]+), (-?[\d.]+)\]"
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True, help="Teensy serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    args = parser.parse_args()

    if MOTOR_ID not in GOAL_MOTOR_ORDER:
        sys.exit(f"MOTOR_ID={MOTOR_ID} must be one of {GOAL_MOTOR_ORDER}")
    if DIRECTION not in (1, -1):
        sys.exit(f"DIRECTION={DIRECTION} must be 1 or -1")

    ser = serial.Serial(args.port, args.baudrate, timeout=0.2)
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"Connected: serial {args.port} @ {args.baudrate}, "
          f"UDP -> {TeensyGoalLink.UDP_HOST}:{TeensyGoalLink.UDP_PORT}.\n")

    positions: dict[int, float] = {}
    clamp_bounds: tuple[float, float] | None = None
    lock = threading.Lock()
    stop_reader = threading.Event()

    def reader_loop() -> None:
        nonlocal clamp_bounds
        while not stop_reader.is_set():
            try:
                raw = ser.readline()
            except serial.SerialException:
                break
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore")
            print(line, end="")

            for match in _STATUS_RE.finditer(line):
                motor_id, pos = int(match[1]), float(match[2])
                with lock:
                    positions[motor_id] = pos

            clamp_match = _CLAMP_RE.search(line)
            if clamp_match and int(clamp_match[1]) == MOTOR_ID:
                with lock:
                    clamp_bounds = (float(clamp_match[3]), float(clamp_match[4]))

    reader = threading.Thread(target=reader_loop, daemon=True)
    reader.start()

    def send_goal(positions_by_id: dict[int, float]) -> None:
        values = [positions_by_id[m] for m in GOAL_MOTOR_ORDER]
        payload = ",".join(f"{v:.4f}" for v in values)
        udp_sock.sendto(payload.encode("ascii"), (TeensyGoalLink.UDP_HOST, TeensyGoalLink.UDP_PORT))

    print(f"Waiting to learn current positions for all 4 motors {GOAL_MOTOR_ORDER} "
          f"(needs at least one status line, printed ~1x/second)...")
    wait_start = time.monotonic()
    while time.monotonic() - wait_start < 5.0:
        with lock:
            if all(m in positions for m in GOAL_MOTOR_ORDER):
                break
        time.sleep(0.1)
    else:
        stop_reader.set()
        ser.close()
        udp_sock.close()
        sys.exit("\nTimed out waiting for status lines from all 4 motors. Is the Teensy powered, "
                  "running the `goal` firmware, and is --port correct?")

    with lock:
        held = dict(positions)
    start_pos = held[MOTOR_ID]
    print(f"\nStarting positions: {held}")
    print(f"Ramping motor {MOTOR_ID} from {start_pos:.4f} rad, direction={DIRECTION:+d}, "
          f"step={STEP_RAD} rad every {STEP_PERIOD_S}s. Holding the other 3 motors fixed.\n")

    target = start_pos
    overshoot_target: float | None = None

    try:
        while True:
            target += DIRECTION * STEP_RAD
            held[MOTOR_ID] = target
            send_goal(held)

            with lock:
                bounds = clamp_bounds

            if bounds is not None and overshoot_target is None:
                lo, hi = bounds
                bound = hi if DIRECTION > 0 else lo
                overshoot_target = bound + DIRECTION * OVERSHOOT_RAD
                print(f"\n>>> Clamp confirmed: firmware reports bound=[{lo:.4f}, {hi:.4f}]. "
                      f"Continuing to {overshoot_target:.4f} rad to confirm it holds, then stopping.\n")

            if overshoot_target is not None:
                reached = target >= overshoot_target if DIRECTION > 0 else target <= overshoot_target
                if reached:
                    print(f"\n>>> Reached {target:.4f} rad, past the clamp bound as requested. Stopping.")
                    break

            if abs(target - start_pos) >= MAX_RAMP_RAD:
                print(f"\n>>> MAX_RAMP_RAD ({MAX_RAMP_RAD} rad) reached with no clamp seen for motor "
                      f"{MOTOR_ID}. Is the arm calibrated ('c')? Stopping without further movement.")
                break

            time.sleep(STEP_PERIOD_S)
    except KeyboardInterrupt:
        print("\n>>> Interrupted.")
    finally:
        print("Sending 'd' (serial) to disable...")
        ser.write(b"d")
        time.sleep(0.5)
        stop_reader.set()
        ser.close()
        udp_sock.close()


if __name__ == "__main__":
    main()
