# Forte SmolVLA Runbook — A to Z

A strict, linear checklist: every physical action and every command, in the order to do them, for
what's achievable **today** with `lerobot_robot_forte_arm` — recording a teleoperated dataset.
Background/rationale lives in `SMOLVLA_GUIDE.md` — read that once first if anything here is
unclear, especially §3 ("what can and can't do today") and §14 (why training beyond this point
needs a firmware change first). This file is the "just tell me what to do" version of what's
currently possible.

Legend: **ACTION** = something you physically do, not a command. **RUN** = paste this into a
terminal, verbatim except placeholders in `<angle brackets>`. **CHECK** = stop and verify before
continuing. **DECISION** = pick one of the listed options for your setup.

Placeholders used throughout — these are **not** shell variables to `export` once. Type the actual
value in by hand, every single time you see one of these in a command below:
- `<TEENSY_PORT>` — the Teensy's serial device path, e.g. `/dev/ttyACM0` (found in Step 7). Used
  **only** for the minicom console you keep open the whole session (Step 11) — LeRobot itself never
  touches serial any more (see Phase 3/4/6: `forte_arm`/`forte_arm_master` read the Teensy over UDP
  telemetry instead, `teleop-bi-c` firmware). This can renumber (`ttyACM0` → `ttyACM1`) if the
  Teensy re-enumerates, so re-check with Step 7's `ls` rather than trusting a value from earlier in
  the session.
- `<HF_USER>` — your Hugging Face username, e.g. `jsmith` (found in Step 6).
- `<TASK_NAME>` — a short slug for the task you're teaching, e.g. `pick_cube` (chosen in Step 20).
  **This is not a category label — it's the literal, exact name of one specific dataset.** It must
  be the *same exact string* in `lerobot-record` (Step 21, where it's chosen) and later in
  `lerobot-train` (Step 26) and `lerobot-rollout`/`lerobot-record --policy.path=...` (Phase 9) —
  `lerobot-train` doesn't know what `pick_cube` "means," it just needs `<HF_USER>/forte_<TASK_NAME>`
  to resolve to an actual dataset that exists. If you append a timestamp at record time to avoid
  collisions between attempts (e.g. `movegear_20260820_230138` instead of plain `movegear`), that
  timestamp is part of `<TASK_NAME>` from then on — check `ls
  ~/.cache/huggingface/lerobot/<HF_USER>/` (Phase 6b) for the exact string if you're not sure which
  recording you're pointing at.

(An earlier version of this runbook had you `export HF_USER=...`/`export TEENSY_PORT=...` once and
reuse the shell variable everywhere below. That's gone — the auto-detection command
(`hf auth whoami` piped through `awk`) silently produced the wrong value often enough that it
wasn't worth the convenience. Typing the literal value is slower but never wrong.)

---

## Phase 0 — Physical rig setup

**Step 1 — ACTION: Mount both arms.**
Bolt the master and slave arm bases down securely. They must not shift or wobble during
recording — any base movement invalidates the fixed camera framing and the Teensy's position
offset. Leave enough clearance that the human operator can comfortably reach the master's full
range of motion without their arm colliding with the slave or the camera stand.

**Step 2 — ACTION: Mount the camera.**
Fix the RealSense (serial `825312072171`) on a stand next to the slave arm, **not** on the arm
itself (this config is a fixed external view, not a wrist cam). Angle it downward, roughly
30–45° off horizontal, aimed at the workspace in front of the slave's end effector — adjust so
that:
- The slave gripper/end-effector is in frame across its full working range of motion.
- The task object's start and end positions are both in frame.
- No part of the master arm or the operator's hand is in frame (the policy should only ever see
  what the slave "knows about").
Tighten the mount. Nudging it after recording starts invalidates the dataset's camera geometry —
treat the mount as fixed for the entire data-collection campaign.

**Step 3 — ACTION: Lighting.**
Set up diffuse, consistent lighting over the workspace. Avoid a single hard light source that
casts moving shadows as the arm moves.

**Step 4 — ACTION: Power on the motors, connect the Teensy.**
Power on the motor supply for both arms (double-check voltage matches your Robstride motor
variant before switching on). Plug the Teensy into the host PC via USB. Do not send `'e'` yet.
Note: unlike an earlier draft of this runbook, **no separate CAN adapter is needed on the host** —
the Teensy's own USB-serial connection is the only link required (see `SMOLVLA_GUIDE.md` §1).

**Which firmware for which phase:** Phases 0–8 (below) need `teensy-forte`'s **`teleop-bi-p-t`**
branch flashed (bilateral teleop, `'e'`/`'d'`). Phase 9 (evaluation) needs a **different**,
standalone firmware — the **`goal`** branch (single arm, `'c'`/`'d'` over serial + goal positions
over Ethernet UDP, no bilateral) — flashed instead; see Phase 9 for when to switch. Don't mix them
up: they speak entirely different protocols.

**Step 4b — ACTION: Pick and mark a physical "home" pose.**
Every `.pos` value this pipeline records or sends is **delta from wherever the arm was posed the
moment you connected**, not an absolute reading (see `SMOLVLA_GUIDE.md` §2 for why) — the Robstride
motors' own absolute position reference isn't guaranteed stable across power cycles. This means: if
you pose the arm differently at the start of two different sessions, `0.5` in one session's data
and `0.5` in the other's don't mean the same physical angle. Pick one physical pose (e.g. "both
arms hanging straight down," or against a mechanical stop) and **always** return the arm to it —
by eye, or with a physical marker/fixture — right before connecting, for every recording session
*and* every eval session for a given task. This is a manual discipline the code can't verify for
you.

---

## Phase 1 — Software environment (one-time)

**Step 5 — RUN: sync the environment.**
```bash
cd /home/daros/workspace2/lerobot_robot_forte_arm
uv sync
```
`pyproject.toml`'s `lerobot[...]` extras list already covers everything this runbook needs
(camera, viz, dataset recording, and SmolVLA training). If a later step still fails with
`ImportError: '<package>' is required but not installed` anyway, that extra fell out of
`pyproject.toml` somehow — add it back to the `lerobot[...]` list and re-run `uv sync`; see
`SMOLVLA_GUIDE.md` §13 for the extra-name-to-package mapping we've hit so far.

**Step 6 — DECISION: Push datasets to the Hugging Face Hub, or stay fully local?**
- **Push to Hub (recommended — easier to visualize/share):**
  ```bash
  uv run hf auth login
  uv run hf auth whoami
  ```
  Note down the username `hf auth whoami` prints — that's your `<HF_USER>` for every command below
  that has one. Type it in literally each time; don't rely on a shell variable (see the placeholder
  note above).
- **Fully local:** skip login; add `--dataset.push_to_hub=false` to the `lerobot-record` command
  in Step 21; use `local` (literally) wherever `<HF_USER>` appears below.

**Step 7 — ACTION: Find the Teensy's serial port.**
```bash
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```
(Unplug/replug the Teensy if you're unsure which one it is.) Note the path — that's your
`<TEENSY_PORT>` for Step 11's minicom command. If it ever stops connecting later in the session,
re-run this `ls` rather than assuming the path is still the same.

---

## Phase 2 — Hardware smoke test (never sends `'e'`, nothing can move)

**Step 8 — RUN:**
```bash
uv run forte-arm-smoke-test
```
Listens on the default UDP telemetry port (5006) — pass `--udp-port` only if you changed
`TELEMETRY_UDP_PORT` in teensy.ino.

**Step 9 — CHECK:**
All 4 joints show a slave *and* a master position (8 numbers total). If some are missing:
- Re-run with `--wait-s 6` in case you just caught it between status prints.
- Confirm the Teensy is powered, running `teensy-forte`'s `teleop-bi-c` firmware, and the Ethernet
  cable is connected — open its serial console directly (step 11) and watch for the
  `Ethernet up: ...` line at boot plus ongoing status/fault lines.

Camera line should print `frame shape (480, 640, 3)`.

---

## Phase 3 — Bring up bilateral teleoperation

**Step 10 — ACTION: Pose both arms to match.**
Physically move the master and slave arms (by hand, torque off) into the same pose — e.g. both
arms hanging straight down, or both at a marked "home" position. The Teensy computes its
master↔slave position offset from whatever pose they're in at the moment you enable, so a
mismatch here becomes a permanent offset error for this session.

**Step 11 — ACTION: Open the Teensy's serial console and leave it open.**
```bash
minicom -D <TEENSY_PORT> -b 115200
```
(`screen <TEENSY_PORT> 115200` also works, but `minicom` is what the rest of this runbook assumes.)
Unlike earlier versions of this workflow, **this console now stays open for the entire session** —
through Phase 4, all of Phase 6's recording, every episode's reset window, all the way to Step 34.
LeRobot's `forte_arm`/`forte_arm_master` never touch the serial port at all (they read the
Teensy's UDP telemetry instead — `teleop-bi-c`'s firmware sends the identical status line over
both channels), so there's nothing left to fight over.

**Step 12 — ACTION: Enable.**
Type `e` and press Enter.

**Step 13 — CHECK:**
Output should show each pair's offset calculated, ending with
`All Motors Enabled & Offset Calculated.` If any pair reports `Offset FAILED`, type `d`, fix the
reported issue (missing feedback / motor fault / invalid position), and repeat from step 10.

**Step 14 — ACTION: Verify mirroring by hand.**
Gently move the master arm through a small range on each joint one at a time. Confirm the slave
mirrors it — correct direction, no jitter, no grinding/stalling sound, no fault messages.

**Step 15 — ACTION (safety): Know your kill switch.**
Confirm you can type `d` in the still-open minicom console instantly if something looks wrong.
Keep that terminal tab visible/accessible for the rest of this session — you'll type `e`/`c`/`d`
into it directly whenever needed, including mid-recording during Phase 6's reset windows (see
Step 22).

---

## Phase 4 — Teleop sanity check through LeRobot (no recording yet)

**Step 16 — RUN** (Teensy still `'e'`-enabled from Phase 3, minicom console from Step 11 still
open in another tab):
```bash
uv run lerobot-teleoperate \
  --robot.type=forte_arm --robot.id=forte_v1 \
  --teleop.type=forte_arm_master --teleop.id=master1 \
  --display_data=true
```
No `--robot.port`/`--teleop.port` any more — both default to UDP telemetry port 5006, which is
what makes both objects share the one UDP socket instead of erroring on a second bind. Only pass
`--robot.udp_port=`/`--teleop.udp_port=` (matching, and matching `TELEMETRY_UDP_PORT` in
teensy.ino) if you changed the firmware's port.

**Step 17 — CHECK:**
Move the master by hand. In the display window, confirm the plotted joint positions move
together (accounting for the fact that updates only arrive ~1x/second — see
`SMOLVLA_GUIDE.md` §1) and the camera feed shows the workspace correctly. Ctrl+C to stop.

---

## Phase 5 — Practice before recording

**Step 18 — ACTION: Place the task object.**
Put the object you'll manipulate at a fixed, marked position within the camera's view.

**Step 19 — ACTION: Practice 5–10 runs, unrecorded.**
With the Teensy still enabled, perform the task via the master arm 5–10 times without recording.
Build one deliberate, repeatable strategy (same grasp point, same approach angle, same timing).

---

## Phase 6 — Record the dataset

**Step 20 — ACTION: Name the task.**
Pick `<TASK_NAME>` (e.g. `pick_cube`) and a one-sentence, action-phrased description.

**Step 21 — RUN** (minicom console from Step 11 still open in another tab — leave it open for the
whole recording session, do not close it between episodes):
```bash
uv run lerobot-record \
  --robot.type=forte_arm --robot.id=forte_v1 \
  --teleop.type=forte_arm_master --teleop.id=master1 \
  --dataset.repo_id=<HF_USER>/forte_<TASK_NAME> \
  --dataset.single_task="<one sentence task description>" \
  --dataset.num_episodes=50 \
  --dataset.fps=15 \
  --dataset.episode_time_s=30 \
  --dataset.reset_time_s=10 \
  --display_data=true
```

**Step 22 — ACTION: Perform the task, once per episode.**
For each episode: perform the practiced strategy via the master arm. Between episodes, during the
`reset_time_s` window, physically put the object back at its marked start position — if you need
to re-`'c'` or cycle `'d'`/`'e'` to fix a drifted offset, type it directly into the still-open
minicom console; there is no serial port to fight over any more, so this no longer requires
closing anything. Keyboard controls (in the LeRobot window): **→** accept and move to next
episode, **←** discard and redo, **ESC** finish early.

**Step 23 — CHECK: Where the data landed.**
```
~/.cache/huggingface/lerobot/<HF_USER>/forte_<TASK_NAME>/
├── data/       # per-episode action/state parquet files
├── videos/     # per-episode, per-camera mp4 files — already organized for you
└── meta/       # info.json, episode index, task list, stats
```
`--dataset.repo_id` is the one name that propagates everywhere (local folder name, Hub repo name).
You don't need to manually move or collect anything.

**Step 24 — CHECK: Expect "steppy" data.**
The joint trajectories in this dataset will visibly hold a value for several frames then jump —
that's the Teensy's ~1 Hz status print rate showing up in the data (`SMOLVLA_GUIDE.md` §1,
§14 item 3), not a bug in the recording. Fine for validating the pipeline end-to-end; treat it as
a known limitation to fix (firmware side) before collecting data for a policy you actually care
about.

**Note on what `action`/`observation.state` actually are:**
- `observation.state` comes from `ForteArm.get_observation()` — the **slave** arm's joint positions.
- `action` comes from `ForteArmMasterTeleop.get_action()` — the **master** arm's joint positions
  (what the human operator does). `ForteArm.send_action(action)` is also called every frame (LeRobot
  always calls it), but it's a no-op that just echoes the action back unchanged — `teleop-bi-p-t`
  has no way to actually command the slave, so it can't modify what gets recorded.

**Note on units:** recorded `.pos` values are motor-shaft **radians** (the firmware's own native
unit, whatever the Teensy's own CAN feedback reports) — this pipeline does not convert to degrees
or to link/joint-space angles anywhere. See `SMOLVLA_GUIDE.md` §2 if you're comparing these numbers
against the physical arm's real range of motion.

They're also **zero-relative**, not the motor's raw absolute reading — but on `teleop-bi-c`, that
zeroing happens firmware-side, not in `ForteArm`/`ForteArmMasterTeleop`: `self._baseline_rad` is a
fixed `0.0` for every motor in both classes, so `.pos` is exactly what the Teensy's status line
reports. The zero point is whatever pose you were in when you last sent `'c'` at the Teensy (Step
12/Phase 3) — see `SMOLVLA_GUIDE.md` §2 for why this matters at all (the Robstride `UNCALIBRATED`
fault bit implies the absolute position reference isn't stable across power cycles). Pose the arm
the same way and re-send `'c'` at the start of every session (Step 4b's home pose), or episodes
recorded in different sessions won't line up. Camera frames (`cam_1`) are unaffected — only the
float `.pos` features go through this.

---

## Phase 6b — Managing your recorded datasets (rename, delete, inspect)

Every `lerobot-record` run creates one dataset — a local folder, and (unless you passed
`--dataset.push_to_hub=false`, Step 6) a matching repo on the Hub. Nothing about this is automatic
housekeeping: cancelled/crashed recordings, throwaway test runs, and typos in `<TASK_NAME>` all
leave folders behind, and only you know which ones are worth keeping.

**List what you have (local):**
```bash
ls -la ~/.cache/huggingface/lerobot/<HF_USER>/
du -sh ~/.cache/huggingface/lerobot/<HF_USER>/*/     # sizes -- a near-empty one is usually a
                                                        # crashed/aborted run with 0 real episodes
```

**Check whether a given dataset actually has data in it**, without opening the visualizer:
```bash
cat ~/.cache/huggingface/lerobot/<HF_USER>/forte_<TASK_NAME>/meta/info.json | grep total_episodes
```
`"total_episodes": 0` means the run never got past the first episode (e.g. the `KeyError` we hit
early on before Ethernet/telemetry was wired up correctly) — safe to delete without a second look.

**Rename (local only):**
```bash
mv ~/.cache/huggingface/lerobot/<HF_USER>/forte_<OLD_NAME> \
   ~/.cache/huggingface/lerobot/<HF_USER>/forte_<NEW_NAME>
```
Safe — nothing inside `meta/info.json` stores its own folder name or repo id, so a plain `mv` is
enough for local use (e.g. pointing `--dataset.root=` at it, or Step 25's visualizer). It does
**not** rename anything already pushed to the Hub — see below for that.

**Delete (local):**
```bash
rm -rf ~/.cache/huggingface/lerobot/<HF_USER>/forte_<TASK_NAME>
```
Irreversible, no trash/undo — double-check the path before running. Useful right after Step 23 if
an episode count or a quick `du -sh` tells you the run is junk, before it's worth the time to open
the visualizer at all.

**Managing the Hub copy** (only relevant if you pushed, i.e. didn't pass
`--dataset.push_to_hub=false`) — a local `mv`/`rm` never touches the Hub; these are separate:
```bash
# List your dataset repos on the Hub
uv run hf repos list --type dataset

# Rename/move a pushed dataset (e.g. fixing a typo, or moving to an org namespace)
uv run hf repos move <HF_USER>/forte_<OLD_NAME> <HF_USER>/forte_<NEW_NAME> --type dataset

# Delete a pushed dataset -- irreversible, prompts for confirmation unless you pass -y
uv run hf repos delete <HF_USER>/forte_<TASK_NAME> --type dataset
```
If you renamed both sides and want them to match again, do the local `mv` and the Hub `hf repos
move` with the same `<NEW_NAME>` — nothing enforces they stay in sync, that's on you to keep
consistent (or just always delete+re-record instead of renaming, if that's simpler for your
workflow).

---

## Phase 7 — Visualize

**Step 25 — ACTION:**
If pushed to the Hub, open https://huggingface.co/spaces/lerobot/visualize_dataset and paste
`<HF_USER>/forte_<TASK_NAME>`. Look through several episodes for anything beyond the expected
steppiness: missing/blurry camera frames, the object not at its marked start position, etc.

**Note:** `lerobot-replay` does not work on `forte_arm` — see `SMOLVLA_GUIDE.md` §3/§1. The
`teleop-bi-p-t` firmware this dataset was recorded against has no way for the host to command the
slave to a position, which replay needs. It should work via `forte_arm_goal`/`goal` firmware
instead (§1a, §12) — not yet verified as of this writing.

---

## Phase 8 — Train SmolVLA

**`<TASK_NAME>` here must be the exact dataset you're training on** — the same literal string you
passed to `--dataset.repo_id` in Step 21, timestamp suffix and all if you used one. If you're not
sure which of your recordings you want, check Phase 6b first (`ls`, `total_episodes`) rather than
guessing.

**Step 26 — RUN:**
```bash
uv run lerobot-train \
  --dataset.repo_id=<HF_USER>/forte_<TASK_NAME> \
  --policy.type=smolvla \
  --policy.device=cuda \
  --output_dir=outputs/train/smolvla_forte_<TASK_NAME> \
  --job_name=smolvla_forte_<TASK_NAME> \
  --batch_size=4 \
  --wandb.enable=true \
  --policy.repo_id=<HF_USER>/smolvla_forte_<TASK_NAME>
```

**Step 27 — ACTION: Watch the first few hundred steps.**
Confirm loss is decreasing and there's no immediate crash. OOM → lower `--batch_size` to 2 or 1.

**Step 28 — ACTION: Let it train** 5–10 epochs over the dataset as a first pass (see
`lerobot/AGENT_GUIDE.md` §7 for the steps↔epochs math). Checkpoints land in
`outputs/train/smolvla_forte_<TASK_NAME>/checkpoints/`.

**If Step 26 fails immediately with `FileExistsError: Output directory ... already exists`:** a
previous attempt at this same `<TASK_NAME>` already created `--output_dir` (even a crash right
after startup creates the directory and starts a wandb run before failing — no real checkpoints in
it). Check first: `ls outputs/train/smolvla_forte_<TASK_NAME>/checkpoints/` — if that's empty/
missing, the old attempt never got anywhere and it's safe to `rm -rf
outputs/train/smolvla_forte_<TASK_NAME>` and re-run Step 26. If there *are* checkpoints you want to
keep, don't delete — either pass `--resume=true` to continue that run, or pick a different
`--output_dir`/`--job_name` for this attempt instead.

---

## Phase 9 — Evaluation on the real arm

The firmware capability this used to be blocked on now exists — a standalone goal-following
firmware, `teensy-forte`'s **`goal`** branch, single arm only (no master, no bilateral logic), plus
a matching `Robot` class (`ForteArmGoal`, `--robot.type=forte_arm_goal`). Protocol is hybrid, not
serial-only: `'c'`/`'d'` (calibrate/disable) still go over USB serial, but the continuous
goal-position stream goes over **Ethernet UDP** to the Teensy (static IP `192.168.1.15:5005`,
direct cable, no gateway/router — referenced from `teensy-forte`'s `isaacsim-udp` branch). Per-joint
safety limits (`JOINT_LIMIT_MIN/MAX_CAN1/CAN2` in `teensy.ino`) are configured with real values, not
placeholders. **Still not run end-to-end with a trained policy as of this writing** — treat this as
a careful first real run, not a routine.

**Step 29 — ACTION: Flash the `goal` firmware, connect Ethernet.**
```bash
cd /home/daros/workspace2/teensy-forte
git checkout goal
# flash teensy/teensy.ino to the Teensy (Arduino IDE / Teensyduino, or your usual flashing method)
```
This **replaces** `teleop-bi-p-t` on the Teensy — you cannot teleoperate or record more bilateral
data until you reflash back afterward. Confirm you're done with Phases 3–8 for this session first.
Also connect a direct Ethernet cable between the host and the Teensy — `'c'`/`'d'` work over USB
alone, but nothing will move without the Ethernet link up (host static IP on the `192.168.1.0/24`
subnet, e.g. `192.168.1.10`).

**Step 30 — ACTION: Pose the arm at your marked home position (Step 4b), then calibrate.**
```bash
screen <TEENSY_PORT> 115200
```
Type `c`. Confirm each motor reports `zero set: 0.000 (<raw>) rad` and
`Calibration complete. Per-joint limits now active relative to this pose.` If any motor reports
`no CAN feedback yet -- FAILED`, don't proceed — that motor isn't communicating.

**Step 31 — ACTION: Bench-test the UDP goal stream before trusting a policy with it.**
```bash
uv run forte-arm-goal-limit-bench --port <TEENSY_PORT>
```
Edit `MOTOR_ID`/`DIRECTION` at the top of `goal_limit_bench.py` to test one axis at a time (motor
ids 11, 13, 12, 14 = yaw, pitch, roll, elbow — **not** ascending CAN-wiring order). It ramps that
one motor slowly, holding the other three at their current position, watching for the firmware's
own `[CANx JOINT LIMIT] ... clamped to [lo, hi]` line — confirming both that the correct physical
joint moves in the direction you intended, and that the real per-joint clamp actually engages on
hardware, not just in source. Repeat for all 4 axes before trusting a combined/policy-driven
command. This script needs `--port` open for serial (status/log lines, `'d'` at the end) *and* the
Ethernet link up for the actual UDP ramp — if Ethernet isn't connected it'll read status fine but
nothing will move.

**Step 32 — CHECK:** all four axes moved the correct joint in the correct direction, and the clamp
fired at a sensible bound for each. If any axis is wrong, stop — that's a wiring/protocol mismatch
to resolve (see `teensy.ino` and `teensy_link.py`'s `GOAL_MOTOR_ORDER`) before anything below.

**Step 33 — RUN: rollout a trained policy.**
```bash
uv run lerobot-rollout \
  --robot.type=forte_arm_goal --robot.port=<TEENSY_PORT> --robot.id=forte_v1 \
  --policy.path=outputs/train/smolvla_forte_<TASK_NAME>/checkpoints/last/pretrained_model \
  --fps=15 --display_data=true
```
No `--teleop.*` flags — `ForteArmGoal` has no paired teleoperator. (`lerobot-eval` is for
gym-style simulation environments despite the name — don't use it for a bare real robot.) To save
the eval episodes as a dataset instead, use `lerobot-record --policy.path=... --robot.type=forte_arm_goal ...`
with `--teleop` simply omitted. `connect()` waits for the arm's current position before it returns
and uses it as the session's delta baseline (Step 4b) — make sure the arm is already at your marked
home pose *before* this command starts, same as recording.

**Step 34 — ACTION (safety): keep `'d'` reachable.**
Same rule as Phase 3's kill switch, but note two things specific to this firmware:
- A second serial console will contend with whatever Python process holds `--port` right now (see
  `SMOLVLA_GUIDE.md` §4) — Ctrl+C on the running `lerobot-rollout`/`lerobot-record` process is the
  practical stop, not a second concurrent reader.
- `'d'` briefly ignores any UDP goal packet for 300ms after it's sent (`DISABLE_IGNORE_MS` in
  teensy.ino), specifically so a straggler packet already in flight can't silently undo the
  disable and re-enable the motors uncalibrated. You may see a
  `"GOAL packet ignored (just disabled...)"` line right after disabling — that's this working as
  intended, not an error.

---

## Phase 10 — Iterate on what you can control today

**Step 35 — DECISION:**
- **Dataset quality concerns beyond the expected steppiness** (camera framing, object placement
  consistency, operator technique) → fix the physical setup (Phase 0) or practice more (Phase 5)
  before recording again.
- **Want a real (non-steppy) dataset** → `SMOLVLA_GUIDE.md` §14 item 3 (fast/on-demand position
  report on `teleop-bi-p-t`). Still the highest-leverage data-quality fix, untouched this session.
- **Want to actually run policies on the arm** → done in principle (Phase 9) — per-joint limits are
  configured and the disable race is fixed, but nothing has been run end-to-end with a trained
  policy yet. The per-axis bench test (Step 31) is the thing to actually do next, not skip.
