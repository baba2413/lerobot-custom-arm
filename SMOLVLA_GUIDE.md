# Forte Arm × LeRobot SmolVLA — End-to-End Guide

Everything needed to go from "serial link works" to "SmolVLA policy running on the real Forte
arm," using the `lerobot_robot_forte_arm` package in this repo. Written against the current state
of the code as of this session — re-check file contents if a lot of time has passed before you
follow this.

---

## 0. Where things live (workspace2 map)

| Folder                     | Role                                                                                             |
| --------------------------- | -------------------------------------------------------------------------------------------------|
| `lerobot/`                  | The LeRobot library itself (cloned source, not our code). Source of truth for all APIs used here. |
| `lerobot_robot_forte_arm/`  | **This project.** LeRobot integration: `config.py`, `teensy_link.py` (`TeensyLink` for bilateral, `TeensyGoalLink` for goal-following), `robot.py` (bilateral slave, `ForteArm`), `teleop_master.py` (bilateral master), `robot_goal.py` (goal-following slave, `ForteArmGoal`), `smoke_test.py`. |
| `teensy-forte/`              | Teensy firmware, one branch per job (see §1a) — **not layers meant to be combined**. `teleop-bi-p-t` is the bilateral teleop/recording firmware (ground truth for CAN ids §2). `goal` is the standalone single-arm goal-following firmware for eval (§9/§12). `teleop-bi` is an older bilateral variant without haptic torque feedback, superseded by `teleop-bi-p-t`. |
| `teensy-dash/`               | A separate, older single-arm (non-bilateral) Teensy firmware for the same physical slave arm. Its `GEAR_RATIO` numbers are reused in `config.py` (best available source) — its CAN id scheme doesn't apply to the bilateral rig. |
| `isaacsim_script/`           | IsaacSim IK solver + UDP sender, a third, independent way to drive the slave arm (not used by the master/slave rig or by `lerobot_robot_forte_arm`). |
| `urdf/`                      | Forte arm URDF + meshes, used by the IsaacSim IK script. |
| `my_raw_datasets/`           | Local LeRobot-format datasets (currently just the `pusht` example, not Forte data). |

---

## 1. Architecture: one Teensy, one serial port, everything through it

Forte is **two physically-identical arms**: a human-driven **master** (leader) and a motorized
**slave** (follower) that mirrors it. This already works today, entirely on a **single Teensy**
board — ground truth is `teensy-forte`'s **`teleop-bi`** branch (`teensy/teensy.ino`, not the
default branch checked out in this repo — `git show teleop-bi:teensy/teensy.ino` to read it, or
`git checkout teleop-bi` to flash it).

```
   [human hand]
        |
        v
  MASTER arm (Robstride motors, ids 1/2/3/4)
        ^  |
        |  v   <-- both wired to the Teensy's onboard CAN1 + CAN2 peripherals -->
        |  |       (which physical pair lands on which of the Teensy's two CAN buses
        |  v        doesn't matter to anything below -- see note at the end of this section)
        |  |
  SLAVE arm (Robstride motors, ids 11/12/13/14 = master id + 10)
        |
        v
   [task / gripper / camera]

            Teensy (one board, two onboard CAN peripherals CAN1/CAN2, one USB-serial port)
            --------------------------------------------------------------------------
            Internally, ~300 Hz: slave_goal = master_pos + offset (offset fixed at 'e'-enable)
            Externally, over the SINGLE serial port:
              host -> Teensy:  'e' (enable + compute offset), 'd' (disable)
              Teensy -> host:  periodic text status lines, ~1x/second:
                "[CAN1] Master 1 Pos: 0.123 rad | Slave 11 Pos: 0.125 rad | Offset: 0.002 (ENABLED)"
```

**This was the load-bearing correction to an earlier draft of this guide:** an earlier version of
`lerobot_robot_forte_arm` assumed the host PC had its own CAN adapter(s) physically tapped onto
the Teensy's CAN1/CAN2 wiring, and talked to the motors directly over `python-can`. That hardware
doesn't exist in this setup. **The Teensy's USB-serial port is the only link between the host and
the arms, full stop.** `lerobot_robot_forte_arm` now reads the Teensy's text status lines and
sends single-character `'e'`/`'d'` commands over that one port — nothing else, and nothing CAN- or
python-can-related is used anywhere in this package anymore.

**Two direct consequences (of the `teleop-bi-p-t` firmware specifically):**

1. **`ForteArm` cannot move the slave arm.** The `teleop-bi-p-t` serial protocol has no "set goal
   position" command — only `'e'`/`'d'`. All actual motor control happens inside the Teensy's own
   firmware, driven by the master arm. `ForteArm.send_action()` is therefore a no-op that only
   echoes back what it was given, purely so `lerobot-record`'s loop (which always calls it) has
   something to log. Standalone policy control needs a *different* firmware — see §1a — not a
   change to this one.
2. **The status line only updates ~once a second** (`LOG_PERIOD` in teensy.ino). LeRobot datasets
   want observations at 10-30 Hz; between Teensy prints, every read from `lerobot_robot_forte_arm`
   is an exact repeat of the last value. This works for the pipeline mechanically, but the
   resulting dataset has effectively ~1 Hz of real signal wrapped in a higher-fps repeat pattern.
   Still open — see §14 item 1.

Both the slave robot (`ForteArm`) and the master teleoperator (`ForteArmMasterTeleop`) read from
the *same* Teensy over the *same* physical port. Since LeRobot instantiates them independently and
a serial port can't be opened twice, `teensy_link.TeensyLink` is a per-port singleton with
reference-counted connect/disconnect — as long as you pass the identical `--robot.port` and
`--teleop.port`, both classes end up sharing one real `serial.Serial` connection instead of
fighting over it.

*(Aside, for anyone re-reading teensy.ino: CAN1 and CAN2 are the Teensy's own two onboard CAN
peripherals — one board, not two — and which master/slave pair is wired to which of the two
doesn't matter here, since the host never touches CAN directly at all anymore, only the merged
text stream over the single serial port.)*

---

## 1a. The `goal` firmware: a separate, single-arm branch for eval

Standalone policy control (§9/§12) needed a serial command the bilateral firmware never had, so
rather than bolt one onto `teleop-bi-p-t`, it's a **separate standalone firmware** on the `goal`
branch of `teensy-forte` — no master arm, no bilateral logic, no haptic feedback, just the 4 slave
motors driven toward host-streamed targets. Consistent with how every branch in this repo is a
self-contained snapshot for one job, not a layer meant to be combined with another (this was a
correction mid-session — an earlier pass at this branch mistakenly extended `teleop-bi-p-t` with a
bolted-on mode instead of starting a standalone build).

Serial protocol (host → Teensy):
```
g\n                              -> enter GOAL mode, holding the arm's current position
g <yaw> <pitch> <roll> <elbow>\n -> enter/refresh GOAL mode, set the 4 joint targets
                                     (raw motor radians; kinematic order -- motor ids 11,13,12,14 --
                                     NOT CAN-wiring order, which would be 11,12,13,14)
d                                -> disable (zero-torque, stop)
```
A 150 ms watchdog (`GOAL_TIMEOUT_MS`) auto-disables the arm if the host stops sending `'g'` lines —
the only thing that keeps the arm moving/enabled is a steady stream of goal commands, not a single
"start" call.

Host side: `teensy_link.TeensyGoalLink` (a separate class from `TeensyLink`, not a variant of it —
different wire protocol, and since there's no master arm competing for the port, no ref-counted
sharing needed) exposes `send_goal()`, `enable()`, `disable()`, `get_positions_deg()`. `robot_goal.
ForteArmGoal` (`--robot.type=forte_arm_goal`) wraps that as a full `Robot`, and unlike `ForteArm`,
its `send_action()` actually moves the arm.

**Status as of this session: written, not yet flashed or bench-tested on real hardware.** Before
trusting it with a policy, bench-test manually over `screen`/minicom: bare `g` holds current pose
with no jump, `g <values>` moves the correct physical joint per axis (one at a time — this is the
first real-hardware check of the yaw/pitch/roll/elbow ↔ motor-id mapping), `d` stops immediately,
and letting `'g'` lines lapse trips the watchdog within ~150 ms.

---

## 2. Current hardware scope

Of the arm's 7 joints, **4 are motorized and wired today** on both master and slave:

| Joint            | Slave CAN id | Master CAN id | Motor:link gear ratio |
| ---------------- | -------------: | --------------: | ----------------------: |
| `shoulder_yaw`    | 11             | 1                | 4.8077                  |
| `shoulder_roll`   | 12             | 2                | 1.0                     |
| `shoulder_pitch`  | 13             | 3                | 3.180                   |
| `elbow_pitch`     | 14             | 4                | 1.0                     |
| `lower_arm_roll`  | —              | —                | not wired               |
| `wrist_pitch`     | —              | —                | not wired               |
| gripper           | —              | —                | not wired               |

The `(slave = master + 10)` id pairing comes straight from `teleop-bi-p-t`'s `MST_IDS_CAN*`/
`SLV_IDS_CAN*` arrays.

**Gear ratios are recorded here as hardware documentation only — nothing in this package applies
them.** They used to (an earlier version of `ForteArm.get_observation()` /
`ForteArmMasterTeleop.get_action()` divided by gear ratio to report "link-space" degrees), but
that was removed: the record→train→replay/eval loop never needed physically-real units — a policy
trained end-to-end doesn't care whether a number is "real" degrees, only that recording and eval
agree — and the conversion was actively risky, since it rested on the unverified assumption that
master and slave gear ratios even match (§14 used to list this as an open item; it's now moot,
since nothing depends on it). The whole pipeline works in **raw motor-shaft degrees** now — what
the Teensy's CAN feedback reports directly, with no conversion in either direction. If you ever
need real physical joint angles again (e.g. for kinematics, or human-authored safety limits — see
§14), that's a good place to reach for this table; just don't thread it back through the
recording/eval path itself.

`ForteArm`, `ForteArmMasterTeleop`, and `ForteArmGoal` all expose exactly these 4 joints as
`{joint}.pos` in raw motor-shaft degrees. Camera: one Intel RealSense (`cam_1`, serial
`825312072171`, 640×480 @ 15 fps, RGB), attached to the slave side (it watches the task).

**Values are also delta from that session's connect()-time baseline, not the motor's raw absolute
reading.** The Robstride CAN protocol has an `UNCALIBRATED` fault bit (decoded in
`printFaultBits()` on every branch) — its existence implies the motor's absolute position
reference isn't guaranteed stable across power cycles, so a dataset recorded across multiple
power-on sessions in raw *absolute* terms could have the same `.pos` number silently mean a
different physical angle in different episodes. `wait_for_positions()` in `teensy_link.py`
captures each motor's first reading at `connect()` as a per-session baseline; `get_observation()`/
`get_action()` report `raw - baseline`, and `ForteArmGoal.send_action()` adds the baseline back
before calling `TeensyGoalLink.send_goal()` (which still speaks absolute raw radians over UDP,
unchanged — delta-vs-absolute is a host-side representation choice, not a wire-protocol one, so
neither firmware branch needed to change for this). This is robust to the encoder's absolute
reference drifting, by construction, whether or not it actually turns out to on this hardware — it
only requires the motor's rotation-to-degrees *scale* to be stable, not its zero point.

**This only works if the arm is physically posed the same way at the start of every session**
(recording *and* eval) before `connect()` — delta cancels an absolute-reference shift between
sessions, not a genuinely different starting pose. Pose consistency is still a manual/process
requirement, not something the code can verify.

When the other 3 joints get motors: add them to `JOINTS` in `config.py`, and to the Teensy's
`MST_IDS_CAN*`/`SLV_IDS_CAN*` arrays.

---

## 3. What `lerobot_robot_forte_arm` can and can't do today

| Capability | Works today? | Notes |
| ---------- | :-----------: | ----- |
| Read slave joint positions (`ForteArm.get_observation()`, `teleop-bi-p-t` firmware) | Yes | Rate-limited to the Teensy's ~1 Hz status print (§1, §14 item 1). |
| Read master joint positions (`ForteArmMasterTeleop.get_action()`) | Yes | Same rate limit. |
| Send `'e'`/`'d'` to the Teensy (`ForteArm.enable()`/`.disable()`) | Yes | Convenience wrapper; you can also just type into the Teensy's serial console directly. |
| Record a teleoperated dataset (`lerobot-record` with `forte_arm` + `forte_arm_master`) | Yes, mechanically | Data quality bottlenecked by the ~1 Hz update rate until §14 item 1 is done. |
| Command the slave to a goal position from the host (`ForteArmGoal.send_action()`, `goal` firmware) | **Written, unverified** | Code complete (§1a); the `goal` firmware has not been flashed or bench-tested on real hardware yet. |
| Standalone policy control / `lerobot-record --policy.path=...` on `forte_arm_goal` | **Written, unverified** | Depends on the above bench test, then wiring a trained policy through `lerobot-record`/`lerobot-eval`. |
| `lerobot-replay` | **Should work on `forte_arm_goal`, unverified** | Not possible on `forte_arm` (no goal-position command in that firmware). Untested on `forte_arm_goal` — same caveats as above. |

---

## 4. Safety notes

- `ForteArm`/`ForteArmMasterTeleop` (`teleop-bi-p-t` firmware) never write anything except
  `'e'`/`'d'` — there is no way for a bug in this package to command an unsafe motor position on
  that firmware, because there is no position-command channel at all. The Teensy's own limits
  (`RAW_LIMIT_MIN/MAX`, ±12.4 rad) are the only safety net in play there.
- **This is no longer true for `ForteArmGoal` (`goal` firmware) — a bug here genuinely can command
  the arm to move.** Two safety nets exist today: the same `RAW_LIMIT_MIN/MAX` protocol-level clamp
  (drops any single motor's command outside ±12.4 rad rather than sending it), and the 150 ms
  `'g'`-line watchdog (§1a) that auto-disables if the host stops streaming. **Neither of these is a
  real per-joint range-of-motion limit** — ±12.4 rad is the wide Robstride protocol bound, not this
  arm's actual safe range. Setting real per-joint limits is still open (§14) and should happen
  before letting a policy — not a human — drive this firmware.
- `TeensyLink.enable()` (or typing `'e'` directly) makes the Teensy compute a new master/slave
  offset from whatever pose the arms are in *at that moment* — pose them to match first (see
  RUNBOOK.md Phase 3). A mismatched pose becomes a wrong offset for the rest of the session.
- Keep the Teensy's serial console (or a way to send `'d'`) accessible any time the arms are
  enabled — it's still the fastest kill switch available. On `goal` firmware specifically, a
  second serial console contends with whatever Python process holds the port (see RUNBOOK.md
  Phase 9) — `'d'` from a live console is the reliable stop, not a second concurrent reader.

---

## 5. Environment setup

```bash
cd /home/daros/workspace2/lerobot_robot_forte_arm
uv sync
```

Requires Python ≥3.12 (matches `lerobot`'s own requirement). `pyproject.toml` depends on the local
`lerobot` checkout with the `intelrealsense`, `hardware`, `viz`, and `dataset` extras:
- `intelrealsense` — the RealSense camera SDK bindings.
- `hardware` — bundles `pyserial-dep` (the `serial` package `teensy_link.py` uses), `pynput-dep`
  (keyboard control during `lerobot-record`), and `deepdiff-dep`.
- `viz` — `rerun-sdk`, needed for `--display_data=true` on `lerobot-teleoperate`/`lerobot-record`.
- `dataset` — `datasets` and friends, needed by `lerobot-record` to actually write episodes.

(Earlier drafts also pulled in the `robstride` extra for direct CAN access via `python-can`;
that's gone now — nothing in this package touches CAN or python-can anymore.)

To push datasets/policies to the Hub later:
```bash
uv run hf auth login
```

---

## 6. Step 1 — Hardware smoke test (never sends `'e'`, nothing can move)

```bash
uv run forte-arm-smoke-test --port /dev/ttyACM0
```

Opens the Teensy's serial port, waits ~3s to catch at least one status-print cycle, and prints
whatever master/slave positions it received (raw motor-shaft degrees — see §2 on why this pipeline
doesn't convert to link-space). Confirm all 4 joints show up for both master and slave — if some
are missing, check the Teensy is powered and running `teleop-bi-p-t`, and that `--port` is right
(`ls /dev/ttyACM* /dev/ttyUSB*`).

---

## 7. Step 2 — Bring up bilateral teleoperation

Pose both arms to match, then send `'e'` (via the Teensy's own serial console, or
`ForteArm(...).enable()` from a Python shell) — see RUNBOOK.md Phase 3 for the literal steps.
Verify by hand that the slave mirrors the master correctly before doing anything else.

---

## 8. Step 3 — Sanity-check teleoperation through LeRobot (no recording)

```bash
uv run lerobot-teleoperate \
  --robot.type=forte_arm --robot.port=/dev/ttyACM0 --robot.id=forte_v1 \
  --teleop.type=forte_arm_master --teleop.port=/dev/ttyACM0 --teleop.id=master1 \
  --display_data=true
```

`--robot.port` and `--teleop.port` must be the **same** device path — that's what makes
`TeensyLink` share the one real serial connection between the two objects instead of erroring on a
second open. Confirm the displayed positions track the master as you move it by hand.

---

## 9. Step 4 — Record a dataset

```bash
HF_USER=$(NO_COLOR=1 uv run hf auth whoami | awk -F': *' 'NR==1 {print $2}')

uv run lerobot-record \
  --robot.type=forte_arm --robot.port=/dev/ttyACM0 --robot.id=forte_v1 \
  --teleop.type=forte_arm_master --teleop.port=/dev/ttyACM0 --teleop.id=master1 \
  --dataset.repo_id=${HF_USER}/forte_<task_name> \
  --dataset.single_task="<one sentence, action-phrased task description>" \
  --dataset.num_episodes=50 \
  --dataset.fps=15 \
  --dataset.episode_time_s=30 \
  --dataset.reset_time_s=10 \
  --display_data=true
```

`--dataset.fps=15` matches the camera config, but note §1/§14 item 1: the joint data itself only
refreshes ~1x/second regardless of `fps` — this doesn't error or crash anything, it just means the
recorded action/state trajectory is coarser than the video. Fine for validating the full pipeline;
do §14 item 1's firmware fix before collecting data you intend to actually train a good policy on.

Data lands at `~/.cache/huggingface/lerobot/${HF_USER}/forte_<task_name>/` (and is pushed to the
Hub unless you pass `--dataset.push_to_hub=false`) — customizable per-run via `--dataset.root=...`,
or globally via the `HF_LEROBOT_HOME`/`HF_HOME` env vars, if you'd rather it land somewhere other
than the default HF cache. **If `$HF_USER` is unset/empty**, `${HF_USER}/forte_<task_name>` expands
to a leading-slash string (e.g. `/forte_<task_name>`), which `pathlib` then treats as an *absolute*
path when joined against the cache root — `lerobot-record` will try to create a directory at
filesystem `/` and die with `PermissionError`. Always confirm `echo $HF_USER` prints something
before running this.

---

## 10. Step 5 — Visualize

```
https://huggingface.co/spaces/lerobot/visualize_dataset  ->  paste ${HF_USER}/forte_<task_name>
```

`lerobot-replay` is **not usable on `forte_arm`** (the `teleop-bi-p-t` firmware has no
goal-position command — see §1). It should work on `forte_arm_goal`/`goal` firmware in principle
(§1a, §3) — untested as of this session.

---

## 11. Step 6 — Train SmolVLA

```bash
uv run lerobot-train \
  --dataset.repo_id=${HF_USER}/forte_<task_name> \
  --policy.type=smolvla \
  --policy.device=cuda \
  --output_dir=outputs/train/smolvla_forte_<task_name> \
  --job_name=smolvla_forte_<task_name> \
  --batch_size=4 \
  --wandb.enable=true \
  --policy.repo_id=${HF_USER}/smolvla_forte_<task_name>
```

Guidance (from `lerobot/AGENT_GUIDE.md` §6–7, unchanged for Forte): 12–16 GB VRAM runs comfortably
at these defaults; aim for 5–10 epochs first; unfreeze the vision encoder
(`--policy.freeze_vision_encoder=false --policy.train_expert_only=false`) once the basic pipeline
works, for a real accuracy gain at the cost of more VRAM/step time.

---

## 12. Step 7 — Evaluate on the real arm

No longer blocked on firmware capability — `goal`/`ForteArmGoal` (§1a) exists — but **not yet
verified on real hardware**, so treat everything below as a draft to validate carefully, not a
routine you can run unattended.

**Before any of this:** bench-test the `goal` firmware manually (§1a's checklist; see
RUNBOOK.md Phase 9 for the literal steps) and decide real per-joint safety limits (§4, §14) — don't
skip straight to a policy driving the arm.

```bash
uv run lerobot-rollout \
  --robot.type=forte_arm_goal --robot.port=/dev/ttyACM0 --robot.id=forte_v1 \
  --policy.path=outputs/train/smolvla_forte_<task_name>/checkpoints/last/pretrained_model \
  --fps=15 --display_data=true
```
(`lerobot-eval` is for gym-style simulation environments, not a bare real robot — don't reach for
it here despite the name. `lerobot-rollout` is the real-hardware equivalent; no `--teleop.*` flags
needed, since `ForteArmGoal` has no paired teleoperator. If you want the eval episodes themselves
saved as a dataset instead, `lerobot-record --policy.path=... --robot.type=forte_arm_goal ...`
with `--teleop` simply omitted works too.)

Two things to keep in mind that don't apply to the bilateral firmware:
- The `goal` firmware's 150 ms watchdog (§1a) means `send_action()` must be called that often to
  keep the arm moving — a policy inference stall (slow model, host under load) shows up as the arm
  stopping mid-motion, not as a queued/delayed command.
- `forte_arm_goal`'s `.pos` units match `forte_arm`'s exactly (raw motor-shaft degrees, §2) by
  design — a policy trained on `forte_arm` recordings should need no unit translation to run here.

---

## 13. Troubleshooting log

| Symptom | Cause | Fix |
| ------- | ----- | --- |
| `SerialException: could not open port` | Wrong `--port`, Teensy not plugged in, or something else already has the port open. | `ls /dev/ttyACM* /dev/ttyUSB*`; close any other serial monitor (Arduino IDE, `screen`, etc.) using the same port. Note the port can renumber (`/dev/ttyACM0` → `/dev/ttyACM1`) if the Teensy re-enumerates — re-check with `ls` rather than assuming it's stable. |
| `TeensyLink for port '...' already exists with baudrate=... got a conflicting baudrate=...` | `--robot.baudrate` and `--teleop.baudrate` don't match for the same `--port`. | Use the same baudrate (115200 default) on both. |
| `DeviceAlreadyConnectedError: ForteArmMasterTeleop is already connected` on `lerobot-record` (but not `lerobot-teleoperate`) | Historical bug, fixed — `lerobot-record` connects `robot` before `teleop`; `ForteArmMasterTeleop.is_connected` used to read the *shared* `TeensyLink`'s state instead of its own, so it looked "already connected" the moment `ForteArm.connect()` opened the link. | Already fixed in `robot.py`/`teleop_master.py` (each tracks its own `_connected` flag now). If you see this again, that fix regressed. |
| Smoke test shows some joints missing | Teensy not running `teleop-bi-p-t`, not yet printed its first status cycle, or a motor fault. | Re-run with a longer `--wait-s`; check the Teensy's own serial output directly for fault messages. |
| Positions look frozen / repeat exactly | Expected — the Teensy only prints ~1x/second (§1). Not a bug. | See §14 item 1. |
| Recorded dataset's action/state looks "steppy" (holds a value for several frames, jumps) | Same cause as above, showing up in the data. | See §14 item 1 before recording data meant for real training. |
| `ImportError: 'rerun-sdk' is required but not installed` on `--display_data=true` | Missing the `viz` extra (§5). | `uv sync` after confirming `pyproject.toml` includes `lerobot[...,viz,...]`. |
| `ImportError: 'datasets' is required but not installed` on `lerobot-record` | Missing the `dataset` extra (§5). | Same fix, add `dataset` to the extras list. |
| `ModuleNotFoundError: No module named 'pynput'` (usually appears as a second exception masking a real error) | Missing the `hardware` extra's `pynput-dep` (§5) — needed for keyboard control during recording. | Add `hardware` (not just `pyserial-dep`) to the extras list. |
| `PermissionError: [Errno 13] Permission denied: '/forte_<task>_<timestamp>'` on `lerobot-record` | `$HF_USER` was empty when `--dataset.repo_id=${HF_USER}/forte_<task>` was expanded, so the repo id became `/forte_<task>` (leading slash) — `pathlib` then treats it as absolute when joined against the cache root, and `lerobot-record` tries to `mkdir` at filesystem `/`. | `echo $HF_USER` before running; see §9 for the full explanation. |
| `lerobot-find-cameras` (no argument) fails on RealSense with `ioctl(VIDIOC_QBUF): Bad file descriptor` / `read failed` spam | The RealSense exposes several `/dev/videoN` nodes (one per sub-stream); the bare form also scans for OpenCV/UVC cameras and tries to reopen those same nodes, colliding with librealsense's exclusive hold on them. | Scope the scan: `lerobot-find-cameras realsense`. |

---

## 14. What's still open (in priority order)

1. **[Hardware] Bench-test the `goal` firmware.** Written this session (§1a), never flashed or run
   against real motors. Do this before anything below that assumes it works: flash `goal`, then
   manually verify over `screen`/minicom — bare `g` holds current pose with no jump, `g <values>`
   moves the correct physical joint per axis (test yaw/pitch/roll/elbow individually), `d` stops
   immediately, and the 150 ms watchdog auto-disables when `'g'` lines stop arriving.
2. **[Decision] Set real per-joint safety limits for `goal` firmware.** The only bound today is the
   Robstride protocol's wide `RAW_LIMIT_MIN/MAX` (±12.4 rad) — not this arm's actual safe range of
   motion. Needed before a policy (not a human) drives the arm via `ForteArmGoal` (§4, §12).
3. **[Firmware] Add a fast/on-demand position report to `teleop-bi-p-t`.** The current ~1 Hz
   `Serial.printf` status line is a monitoring afterthought, not designed for a control loop
   consuming it at 10-30 Hz. The straightforward fix: print the same data (or a terser CSV form)
   every control-loop tick, or add a request/response command the host can poll on demand. This is
   the highest-leverage remaining data-quality fix — everything downstream of it (steppy datasets,
   §1) depends on it. Not done in this session because it means editing and reflashing physical
   hardware firmware, which needs to happen deliberately with someone at the arm. (Note: this is
   scoped to `teleop-bi-p-t` specifically, for teleop/recording data quality — it has no bearing on
   `goal` firmware, which doesn't do bilateral status reporting at all.)
4. **[Wiring] Connect a trained policy through `lerobot-eval`/`lerobot-rollout`/`lerobot-record`
   end to end on `forte_arm_goal`** (§12) once (1) and (2) are done — not yet exercised even in
   principle, only the `Robot`-interface plumbing (`ForteArmGoal.send_action()`) exists so far.
5. Wire up `lower_arm_roll`, `wrist_pitch`, gripper on both arms once motorized, extending
   `JOINTS` in `config.py` and the Teensy's `MST_IDS_CAN*`/`SLV_IDS_CAN*` arrays (on whichever
   firmware branch(es) still need it).
6. First real dataset + SmolVLA training run once (3) is done.

**No longer open:** the old item 3 here ("confirm master/slave gear ratios match") is moot — the
pipeline no longer applies gear ratios anywhere (§2), so whether they matched was never actually
load-bearing for anything once removed.
