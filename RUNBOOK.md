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

Placeholders used throughout — set these once and reuse them:
- `<TEENSY_PORT>` — the Teensy's serial device path (found in step 8). There is exactly **one**
  port for the whole rig — both arms are read through this single connection.
- `<HF_USER>` — your Hugging Face username (set in step 10).
- `<TASK_NAME>` — a short slug for the task you're teaching, e.g. `pick_cube` (chosen in step 22).

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
standalone firmware — the **`goal`** branch (single arm, `'g'`/`'d'`, no bilateral) — flashed
instead; see Phase 9 for when to switch. Don't mix them up: they speak different serial protocols.

---

## Phase 1 — Software environment (one-time)

**Step 5 — RUN: sync the environment.**
```bash
cd /home/daros/workspace2/lerobot_robot_forte_arm
uv sync
```

**Step 6 — DECISION: Push datasets to the Hugging Face Hub, or stay fully local?**
- **Push to Hub (recommended — easier to visualize/share):**
  ```bash
  uv run hf auth login
  export HF_USER=$(NO_COLOR=1 uv run hf auth whoami | awk -F': *' 'NR==1 {print $2}')
  ```
- **Fully local:** skip login; add `--dataset.push_to_hub=false` to the `lerobot-record` command
  in step 23; set `export HF_USER=local` as a stand-in for naming.

**Step 7 — ACTION: Find the Teensy's serial port.**
```bash
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```
(Unplug/replug the Teensy if you're unsure which one it is.) Set it for the rest of this session:
```bash
export TEENSY_PORT=/dev/ttyACM0   # replace with your actual path
```

---

## Phase 2 — Hardware smoke test (never sends `'e'`, nothing can move)

**Step 8 — RUN:**
```bash
uv run forte-arm-smoke-test --port $TEENSY_PORT
```

**Step 9 — CHECK:**
All 4 joints show a slave *and* a master position (8 numbers total). If some are missing:
- Re-run with `--wait-s 6` in case you just caught it between status prints.
- Confirm the Teensy is powered and running `teensy-forte`'s `teleop-bi-p-t` firmware — open its
  serial console directly (step 12) and watch for status/fault lines.

Camera line should print `frame shape (480, 640, 3)`.

---

## Phase 3 — Bring up bilateral teleoperation

**Step 10 — ACTION: Pose both arms to match.**
Physically move the master and slave arms (by hand, torque off) into the same pose — e.g. both
arms hanging straight down, or both at a marked "home" position. The Teensy computes its
master↔slave position offset from whatever pose they're in at the moment you enable, so a
mismatch here becomes a permanent offset error for this session.

**Step 11 — ACTION: Open the Teensy's serial console.**
```bash
screen $TEENSY_PORT 115200
```
(any serial tool works — Arduino IDE's Serial Monitor, `minicom`, etc.)

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
Confirm you can type `d` in the serial console instantly if something looks wrong. Keep it
visible/accessible (in a separate terminal tab) for the rest of this session — you'll need the
Teensy's port for LeRobot commands too, so either share the console non-exclusively or be ready to
switch tabs quickly.

---

## Phase 4 — Teleop sanity check through LeRobot (no recording yet)

**Step 16 — RUN** (Teensy still `'e'`-enabled from Phase 3):
```bash
uv run lerobot-teleoperate \
  --robot.type=forte_arm --robot.port=$TEENSY_PORT --robot.id=forte_v1 \
  --teleop.type=forte_arm_master --teleop.port=$TEENSY_PORT --teleop.id=master1 \
  --display_data=true
```
`--robot.port` and `--teleop.port` must be the identical path — that's what makes both objects
share the one real connection to the Teensy instead of erroring on a second open.

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

**Step 21 — RUN:**
```bash
uv run lerobot-record \
  --robot.type=forte_arm --robot.port=$TEENSY_PORT --robot.id=forte_v1 \
  --teleop.type=forte_arm_master --teleop.port=$TEENSY_PORT --teleop.id=master1 \
  --dataset.repo_id=${HF_USER}/forte_<TASK_NAME> \
  --dataset.single_task="<one sentence task description>" \
  --dataset.num_episodes=50 \
  --dataset.fps=15 \
  --dataset.episode_time_s=30 \
  --dataset.reset_time_s=10 \
  --display_data=true
```

**Step 22 — ACTION: Perform the task, once per episode.**
For each episode: perform the practiced strategy via the master arm. Between episodes, during the
`reset_time_s` window, physically put the object back at its marked start position. Keyboard
controls: **→** accept and move to next episode, **←** discard and redo, **ESC** finish early.

**Step 23 — CHECK: Where the data landed.**
```
~/.cache/huggingface/lerobot/${HF_USER}/forte_<TASK_NAME>/
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

**Note on units:** recorded `.pos` values are raw motor-shaft degrees (whatever the Teensy's own
CAN feedback reports) — this pipeline does not convert to link/joint-space degrees anywhere. See
`SMOLVLA_GUIDE.md` §2 if you're comparing these numbers against the physical arm's real range of
motion.

---

## Phase 7 — Visualize

**Step 25 — ACTION:**
If pushed to the Hub, open https://huggingface.co/spaces/lerobot/visualize_dataset and paste
`${HF_USER}/forte_<TASK_NAME>`. Look through several episodes for anything beyond the expected
steppiness: missing/blurry camera frames, the object not at its marked start position, etc.

**Note:** `lerobot-replay` does not work on `forte_arm` — see `SMOLVLA_GUIDE.md` §3/§1. The
`teleop-bi-p-t` firmware this dataset was recorded against has no way for the host to command the
slave to a position, which replay needs. It should work via `forte_arm_goal`/`goal` firmware
instead (§1a, §12) — not yet verified as of this writing.

---

## Phase 8 — Train SmolVLA

**Step 26 — RUN:**
```bash
uv run lerobot-train \
  --dataset.repo_id=${HF_USER}/forte_<TASK_NAME> \
  --policy.type=smolvla \
  --policy.device=cuda \
  --output_dir=outputs/train/smolvla_forte_<TASK_NAME> \
  --job_name=smolvla_forte_<TASK_NAME> \
  --batch_size=4 \
  --wandb.enable=true \
  --policy.repo_id=${HF_USER}/smolvla_forte_<TASK_NAME>
```

**Step 27 — ACTION: Watch the first few hundred steps.**
Confirm loss is decreasing and there's no immediate crash. OOM → lower `--batch_size` to 2 or 1.

**Step 28 — ACTION: Let it train** 5–10 epochs over the dataset as a first pass (see
`lerobot/AGENT_GUIDE.md` §7 for the steps↔epochs math). Checkpoints land in
`outputs/train/smolvla_forte_<TASK_NAME>/checkpoints/`.

---

## Phase 9 — Evaluation on the real arm

The firmware capability this used to be blocked on now exists — a standalone goal-following
firmware, `teensy-forte`'s **`goal`** branch, single arm only (no master, no bilateral logic), plus
a matching `Robot` class (`ForteArmGoal`, `--robot.type=forte_arm_goal`). **It has not been flashed
or run against real motors yet.** Treat every step below as a first-time bring-up, not a routine —
go slowly, and don't skip the bench test to get to the policy faster.

**Step 29 — ACTION: Flash the `goal` firmware.**
```bash
cd /home/daros/workspace2/teensy-forte
git checkout goal
# flash teensy/teensy.ino to the Teensy (Arduino IDE / Teensyduino, or your usual flashing method)
```
(As of this writing the `goal` branch's firmware commits are local-only, not pushed to `origin` —
if you're flashing from a different machine than where they were written, `git push` first.)
This **replaces** `teleop-bi-p-t` on the Teensy — you cannot teleoperate or record more bilateral
data until you reflash back to `teleop-bi-p-t` afterward. Confirm you're done with Phases 3–8 for
this session before doing this.

**Step 30 — ACTION: Bench-test `'g'`/`'d'` manually before touching Python.**
```bash
screen $TEENSY_PORT 115200
```
- Type `g` and press Enter (bare). Confirm the arm holds its current pose — no jump, no motion.
- Type `d`. Confirm it stops/disables cleanly.
- Type `g` followed by 4 space-separated raw-radian targets, **one axis at a time** (i.e. change
  only one of the 4 numbers a little from the arm's current position, leave the other 3 matching
  its current pose) — confirm the physical joint that moves is the one you intended. Order is
  `<yaw> <pitch> <roll> <elbow>` (motor ids 11, 13, 12, 14 — **not** ascending CAN-wiring order).
  This is the first real-hardware check of that mapping, so check all 4 axes individually before
  trusting a combined command.
- Stop sending `g` lines and confirm the arm auto-stops within ~150ms (the watchdog) without you
  typing `d`.

**Step 31 — CHECK:** all four of the above behaved as expected. If any axis moved the wrong joint
or the wrong direction, stop — that's a wiring/protocol mismatch to resolve (see
`teensy-forte`'s `goal` branch `teensy.ino` and `teensy_link.py`'s `GOAL_MOTOR_ORDER`) before
anything below.

**Step 32 — DECISION: Set real per-joint safety limits.**
The only bound in the firmware today is the Robstride protocol's wide `RAW_LIMIT_MIN/MAX`
(±12.4 rad) — not this arm's actual safe range of motion. Decide and implement real per-joint
limits (firmware-side clamp, or a host-side pre-send clamp in `ForteArmGoal.send_action()`) before
letting a policy — not a human at the bench — drive the arm. Not yet decided or implemented.

**Step 33 — RUN: rollout a trained policy.**
```bash
uv run lerobot-rollout \
  --robot.type=forte_arm_goal --robot.port=$TEENSY_PORT --robot.id=forte_v1 \
  --policy.path=outputs/train/smolvla_forte_<TASK_NAME>/checkpoints/last/pretrained_model \
  --fps=15 --display_data=true
```
No `--teleop.*` flags — `ForteArmGoal` has no paired teleoperator. (`lerobot-eval` is for
gym-style simulation environments despite the name — don't use it for a bare real robot.) To save
the eval episodes as a dataset instead, use `lerobot-record --policy.path=... --robot.type=forte_arm_goal ...`
with `--teleop` simply omitted.

**Step 34 — ACTION (safety): keep `'d'` reachable.**
Same rule as Phase 3's kill switch — but note a second serial console will contend with whatever
Python process holds the port right now (see `SMOLVLA_GUIDE.md` §4). Ctrl+C on the running
`lerobot-rollout`/`lerobot-record` process is the practical stop, not a second concurrent reader.

---

## Phase 10 — Iterate on what you can control today

**Step 35 — DECISION:**
- **Dataset quality concerns beyond the expected steppiness** (camera framing, object placement
  consistency, operator technique) → fix the physical setup (Phase 0) or practice more (Phase 5)
  before recording again.
- **Want a real (non-steppy) dataset** → `SMOLVLA_GUIDE.md` §14 item 3 (fast/on-demand position
  report on `teleop-bi-p-t`). Still the highest-leverage data-quality fix, untouched this session.
- **Want to actually run policies on the arm** → done in principle (Phase 9) — what's left is the
  bench test (Step 30) and safety limits (Step 32), both still open.
