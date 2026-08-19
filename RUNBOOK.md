# Forte SmolVLA Runbook — A to Z

A strict, linear checklist: every physical action and every command, in the order to do them,
from empty desk to a trained SmolVLA policy running on the slave arm. Background/rationale lives
in `SMOLVLA_GUIDE.md` — read that once first if anything here is unclear; this file is the
"just tell me what to do" version.

Legend: **ACTION** = something you physically do, not a command. **RUN** = paste this into a
terminal, verbatim except placeholders in `<angle brackets>`. **CHECK** = stop and verify before
continuing. **DECISION** = pick one of the listed options for your setup.

Placeholders used throughout — set these once and reuse them:
- `<PORT_CAN1>`, `<PORT_CAN2>` — your two CAN adapter device paths (found in step 8).
- `<HF_USER>` — your Hugging Face username (set in step 6).
- `<TASK_NAME>` — a short slug for the task you're teaching, e.g. `pick_cube` (chosen in step 24).

---

## Phase 0 — Physical rig setup

**Step 1 — ACTION: Mount both arms.**
Bolt the master and slave arm bases down securely. They must not shift or wobble during
recording — any base movement invalidates the fixed camera framing and the Teensy's position
offset. Leave enough clearance that the human operator can comfortably reach the master's full
range of motion without their arm colliding with the slave or the camera stand.

**Step 2 — ACTION: Check CAN wiring topology.**
Confirm (visually trace the wiring, or ask whoever wired it) which physical bus is which:
- **CAN1** carries `shoulder_yaw` + `shoulder_roll` — master motor ids 1/2, slave ids 11/12.
- **CAN2** carries `shoulder_pitch` + `elbow_pitch` — master motor ids 3/4, slave ids 13/14.

Each bus should be terminated with a 120 Ω resistor at both physical ends (standard CAN bus
practice) if not already — untermin­ated buses cause intermittent comms errors under load.

**Step 3 — DECISION: How does the host PC tap into CAN1/CAN2?**
- If you have **two USB-CAN adapters**: plug one onto the CAN1 wiring, one onto CAN2. This is the
  configuration everything below assumes.
- If you have **one adapter**: it can only see one bus unless your wiring bridges CAN1 and CAN2
  together (uncommon, since they're electrically separate Teensy peripherals) — check with
  whoever wired it. If you truly only have one bus reachable, you'll only be able to
  observe/record 2 of the 4 joints; the rest of this runbook assumes both buses are reachable.

**Step 4 — ACTION: Plug in the CAN adapter(s).**
Connect them to the host PC via USB. Do not power-cycle the Teensy or motors yet.

**Step 5 — ACTION: Mount the camera.**
Fix the RealSense (serial `825312072171`) on a stand next to the slave arm, **not** on the arm
itself (this config is a fixed external view, not a wrist cam). Angle it downward, roughly
30–45° off horizontal, aimed at the workspace in front of the slave's end effector — adjust so
that:
- The slave gripper/end-effector is in frame across its full working range of motion.
- The task object's start and end positions are both in frame.
- No part of the master arm or the operator's hand is in frame (the policy should only ever see
  what the slave "knows about").
Tighten the mount. Nudging it even slightly after recording starts invalidates the dataset's
camera geometry — treat the mount as fixed for the entire data-collection campaign.

**Step 6 — ACTION: Lighting.**
Set up diffuse, consistent lighting over the workspace. Avoid a single hard light source that
casts moving shadows as the arm moves — that's a common cause of policies that "see" a shadow as
part of the task.

**Step 7 — ACTION: Power on.**
Power on the motor supply for both arms (double-check voltage matches your Robstride motor
variant before switching on). Power/connect the Teensy via USB. Do not send `'e'` yet.

---

## Phase 1 — Software environment (one-time)

**Step 8 — ACTION: Find your CAN adapter device paths.**
```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```
Note which path is CAN1's adapter and which is CAN2's (unplug one at a time and re-run if unsure
which is which). Set them for the rest of this session:
```bash
export PORT_CAN1=/dev/ttyUSB0   # replace with your actual path
export PORT_CAN2=/dev/ttyUSB1   # replace with your actual path
```

**Step 9 — RUN: sync the environment.**
```bash
cd /home/daros/workspace2/lerobot_robot_forte_arm
uv sync
```

**Step 10 — DECISION: Push datasets/policies to the Hugging Face Hub, or stay fully local?**
- **Push to Hub (recommended — easier to visualize/share/resume):**
  ```bash
  uv run hf auth login
  export HF_USER=$(NO_COLOR=1 uv run hf auth whoami | awk -F': *' 'NR==1 {print $2}')
  ```
- **Fully local:** skip login; add `--dataset.push_to_hub=false` to every `lerobot-record` command
  below, and set `export HF_USER=local` as a stand-in for naming.

---

## Phase 2 — Hardware smoke test (torque off, nothing can move)

**Step 11 — RUN:**
```bash
uv run forte-arm-smoke-test --port-can1 $PORT_CAN1 --port-can2 $PORT_CAN2
```

**Step 12 — CHECK:**
- All 8 motors (4 master + 4 slave) responded — no `ConnectionError`, no missing ids.
- Printed link-space degrees look physically plausible for wherever the arms currently are.
- Camera line prints `frame shape (480, 640, 3)`.

If anything fails here, stop and fix it before continuing — do not proceed to enabling torque
with an unverified CAN link.

---

## Phase 3 — Bring up bilateral teleoperation

**Step 13 — ACTION: Pose both arms to match.**
Physically move the master and slave arms (by hand, torque off) into the same pose — e.g. both
arms hanging straight down, or both at a marked "home" position. The Teensy computes its
master↔slave position offset from whatever pose they're in at the moment you enable, so a
mismatch here becomes a permanent offset error for this session.

**Step 14 — ACTION: Open the Teensy's serial console.**
```bash
# example using screen; use whatever serial tool you have (Arduino IDE Serial Monitor also works)
screen <TEENSY_SERIAL_PORT> 115200
```

**Step 15 — ACTION: Enable.**
Type `e` and press Enter in the serial console.

**Step 16 — CHECK:**
Serial output should show each pair's offset being calculated and end with
`All Motors Enabled & Offset Calculated.` If any pair reports `Offset FAILED` (missing feedback,
motor fault, or invalid position), type `d` to disable, fix the reported issue, and repeat from
step 13.

**Step 17 — ACTION: Verify mirroring by hand.**
Gently move the master arm through a small range on each joint one at a time. Confirm the slave
mirrors it — correct direction, no jitter, no grinding/stalling sound, no fault messages in the
serial log.

**Step 18 — ACTION (safety): Know your kill switch.**
Confirm you can type `d` in the serial console (or cut motor power) instantly if something looks
wrong. Keep the serial console visible/accessible for the rest of this session.

---

## Phase 4 — LeRobot calibration (one-time per `--robot.id`)

**Step 19 — RUN:**
```bash
uv run lerobot-calibrate \
  --robot.type=forte_arm \
  --robot.port_can1=$PORT_CAN1 --robot.port_can2=$PORT_CAN2 \
  --robot.id=forte_v1
```
This just records a trivial per-motor range so the `Robot` interface is satisfied — it does not
move anything or interact with the Teensy's own offset. Safe to run with the Teensy currently
enabled from Phase 3.

---

## Phase 5 — Teleop sanity check through LeRobot (no recording yet)

**Step 20 — RUN** (with the Teensy still `'e'`-enabled from Phase 3):
```bash
uv run lerobot-teleoperate \
  --robot.type=forte_arm --robot.port_can1=$PORT_CAN1 --robot.port_can2=$PORT_CAN2 \
  --robot.id=forte_v1 --robot.owns_actuation=false \
  --teleop.type=forte_arm_master --teleop.port_can1=$PORT_CAN1 --teleop.port_can2=$PORT_CAN2 \
  --display_data=true
```

**Step 21 — CHECK:**
Move the master by hand. In the display window, confirm the plotted joint positions move
together and the camera feed shows the workspace correctly. Ctrl+C to stop when satisfied.

---

## Phase 6 — Practice before recording

**Step 22 — ACTION: Place the task object.**
Put the object you'll manipulate at a fixed, marked position within the camera's view (e.g. tape
an outline on the table). Decide exactly what "done" looks like for one episode.

**Step 23 — ACTION: Practice 5–10 runs, unrecorded.**
With the Teensy still enabled, perform the task via the master arm 5–10 times without recording.
Build one deliberate, repeatable strategy (same grasp point, same approach angle, same timing).
Hesitant or inconsistent practice teaches the eventual policy to be hesitant and inconsistent too.

---

## Phase 7 — Record the dataset

**Step 24 — ACTION: Name the task.**
Pick `<TASK_NAME>` (e.g. `pick_cube`) and a one-sentence, action-phrased description (e.g. "Pick
up the red cube and place it in the bin"). Use both consistently below.

**Step 25 — RUN:**
```bash
uv run lerobot-record \
  --robot.type=forte_arm --robot.port_can1=$PORT_CAN1 --robot.port_can2=$PORT_CAN2 \
  --robot.id=forte_v1 --robot.owns_actuation=false \
  --teleop.type=forte_arm_master --teleop.port_can1=$PORT_CAN1 --teleop.port_can2=$PORT_CAN2 \
  --dataset.repo_id=${HF_USER}/forte_<TASK_NAME> \
  --dataset.single_task="<one sentence task description>" \
  --dataset.num_episodes=50 \
  --dataset.fps=15 \
  --dataset.episode_time_s=30 \
  --dataset.reset_time_s=10 \
  --display_data=true
```
`--robot.owns_actuation=false` is mandatory here — see `SMOLVLA_GUIDE.md` §3 for why (the Teensy,
not the host, is driving the slave; the host only logs).

**Step 26 — ACTION: Perform the task, once per episode.**
For each of the 50 episodes: perform the practiced strategy via the master arm. Between episodes,
during the `reset_time_s` window, physically put the object back at its marked start position.
Keyboard controls while recording: **→** accept episode and move to next, **←** discard and redo
the current episode, **ESC** finish early and finalize the dataset.

**Step 27 — CHECK: Where the data landed.**
If `--dataset.push_to_hub=false` (or the upload is still catching up), the raw episode files are
already on disk at:
```
~/.cache/huggingface/lerobot/${HF_USER}/forte_<TASK_NAME>/
├── data/       # per-episode action/state parquet files
├── videos/     # per-episode, per-camera mp4 files (this is "gather all video files" — LeRobot
│               # already does this for you; you don't need to manually collect them anywhere)
└── meta/       # info.json, episode index, task list, stats
```
You do not need to manually move or rename anything — `--dataset.repo_id` is the one name that
propagates everywhere (local folder name, Hub repo name, and what you pass to every subsequent
command in this runbook as `${HF_USER}/forte_<TASK_NAME>`). If you're scripting anything custom
against the raw files later, that folder path is the one to point it at.

---

## Phase 8 — Visualize and replay before training

**Step 28 — ACTION: Visualize.**
If pushed to the Hub, open https://huggingface.co/spaces/lerobot/visualize_dataset and paste
`${HF_USER}/forte_<TASK_NAME>`. Look through several episodes for: missing/blurry camera frames,
joints hitting limits, the object not actually at its marked start position, anything that
doesn't match what you intended to teach.

**Step 29 — ACTION: Stop the Teensy before replay.**
Type `d` in the Teensy's serial console (from Phase 3) to disable both arms. Replay drives the
slave directly from the host and will fight the Teensy's control loop if it's still active.

**Step 30 — RUN:**
```bash
uv run lerobot-replay \
  --robot.type=forte_arm --robot.port_can1=$PORT_CAN1 --robot.port_can2=$PORT_CAN2 \
  --robot.id=forte_v1 \
  --dataset.repo_id=${HF_USER}/forte_<TASK_NAME> --dataset.episode=0
```

**Step 31 — CHECK:** the slave physically reproduces episode 0's motion smoothly, without
faulting or hitting anything. Repeat for a couple more episode indices if you have any doubt about
data quality.

---

## Phase 9 — Train SmolVLA

**Step 32 — RUN:**
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
(Drop `--wandb.enable=true` if you haven't set up a Weights & Biases account; training still
prints loss to the console.)

**Step 33 — ACTION: Watch the first few hundred steps.**
Confirm loss is actually decreasing and there's no immediate crash (shape mismatch, OOM). OOM →
lower `--batch_size` to 2 or 1; still OOM at 1 → your GPU is too small for SmolVLA at these
image dimensions, see `lerobot/AGENT_GUIDE.md` §6 for alternatives.

**Step 34 — ACTION: Let it train.**
Aim for 5–10 epochs over the dataset first (see `SMOLVLA_GUIDE.md` §11 for the steps↔epochs math
for your actual episode count). Checkpoints land in
`outputs/train/smolvla_forte_<TASK_NAME>/checkpoints/`.

**Step 35 — DECISION: Good enough, or push further?**
If the loss curve has clearly plateaued, stop here and move to evaluation (Phase 10). If it's
still dropping and you're under ~10 epochs, let it keep training.

---

## Phase 10 — Evaluate on the real arm

**Step 36 — ACTION: Take the Teensy and master arm fully out of the loop.**
Confirm the Teensy is disabled (`'d'` sent, from step 29) — for this phase the trained policy
replaces the human/master entirely, and the host must be the sole controller of the slave.

**Step 37 — ACTION: Set real safety limits.**
Work out the slave's actual safe joint range by hand (torque off) and note min/max degrees per
joint — an untrained/undertrained policy can command large, sudden jumps and you want hard limits
in place before it's in control.

**Step 38 — RUN:**
```bash
uv run lerobot-record \
  --robot.type=forte_arm --robot.port_can1=$PORT_CAN1 --robot.port_can2=$PORT_CAN2 \
  --robot.id=forte_v1 \
  --robot.joint_limits="{shoulder_yaw: [<min>,<max>], shoulder_pitch: [<min>,<max>], shoulder_roll: [<min>,<max>], elbow_pitch: [<min>,<max>]}" \
  --robot.max_relative_target=5.0 \
  --dataset.repo_id=${HF_USER}/eval_forte_<TASK_NAME> \
  --dataset.single_task="<same task description as training>" \
  --dataset.num_episodes=10 \
  --dataset.fps=15 \
  --policy.path=${HF_USER}/smolvla_forte_<TASK_NAME>
```
Replace every `<min>,<max>` with the values from step 37. Note this run does **not** pass
`--robot.owns_actuation=false` — here the host is meant to drive the arm directly.

**Step 39 — ACTION: Supervise every episode.**
Stand ready at the kill switch / motor power for all 10 episodes. Reset the object to its start
position between episodes exactly as you did during data collection (step 26).

**Step 40 — ACTION: Score it.**
Count successes out of 10. Compare against how reliably you could do the task by teleoperation —
that's your baseline, not 100%.

---

## Phase 11 — Iterate

**Step 41 — DECISION: What to do with the result.**
- **Fails at one specific point in the task** → go back to Phase 6/7 and record 10–20 more
  demonstrations targeting exactly that failure point. Re-run Phase 9 training from the existing
  checkpoint's dataset plus the new episodes.
- **Ignores the object / seems not to see it** → camera framing or lighting problem (Phase 0,
  steps 5–6), not a training problem — fix the physical setup before recording more data.
- **Loss was still dropping and success rate is mediocre** → train longer (back to step 34) before
  collecting more data.
- **Basic pipeline works, want more accuracy** → retrain with the vision encoder unfrozen:
  ```bash
  uv run lerobot-train ... --policy.type=smolvla \
    --policy.freeze_vision_encoder=false --policy.train_expert_only=false
  ```
  Costs more VRAM and time per step — try it once a frozen-encoder run already works.
