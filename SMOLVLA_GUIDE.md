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
| `lerobot_robot_forte_arm/`  | **This project.** LeRobot integration: `config.py`, `teensy_link.py` (shared serial link), `robot.py` (slave), `teleop_master.py` (master), `smoke_test.py`. |
| `teensy-forte/`              | Teensy firmware. **`teleop-bi` branch, `teensy/teensy.ino`** is the currently-used, working bilateral rig and the ground truth for CAN ids and serial protocol (§1). `teleop-bi-p-t` is a variant that adds haptic torque feedback — same ids, same serial format, not currently the primary branch. |
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

**Two direct consequences:**

1. **`ForteArm` cannot move the slave arm.** The Teensy's serial protocol has no "set goal
   position" command — only `'e'`/`'d'`. All actual motor control happens inside the Teensy's own
   firmware, driven by the master arm. `ForteArm.send_action()` is therefore a no-op that only
   echoes back what it was given, purely so `lerobot-record`'s loop (which always calls it) has
   something to log. **Standalone policy control (§9/§12, a trained policy driving the arm with no
   human on the master) is not possible until the firmware gains a goal-position serial command —
   see §14.**
2. **The status line only updates ~once a second** (`LOG_PERIOD` in teensy.ino). LeRobot datasets
   want observations at 10-30 Hz; between Teensy prints, every read from `lerobot_robot_forte_arm`
   is an exact repeat of the last value. This works for the pipeline mechanically, but the
   resulting dataset has effectively ~1 Hz of real signal wrapped in a higher-fps repeat pattern.
   See §14 for the firmware fix.

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

The `(slave = master + 10)` id pairing comes straight from `teleop-bi`'s `MST_IDS_CAN*`/
`SLV_IDS_CAN*` arrays. Gear ratios come from `teensy-dash/src/main.cpp` (the separate single-arm
firmware) — **assumed** identical on the master arm since it's stated to be the same physical arm
design, but not independently re-verified. The Teensy's own bilateral control never applies a
gear ratio; it works entirely in raw motor-shaft radians (offset-calibrated per pair at
`'e'`-time), so this assumption only affects the link-space degrees `lerobot_robot_forte_arm`
reports — not the Teensy's own control.

`ForteArm` and `ForteArmMasterTeleop` both expose exactly these 4 joints as `{joint}.pos` in
**link-space degrees**. Camera: one Intel RealSense (`cam_1`, serial `825312072171`, 640×480 @
15 fps, RGB), attached to the slave side (it watches the task).

When the other 3 joints get motors: add them to `JOINTS` in `config.py`, and to the Teensy's
`MST_IDS_CAN*`/`SLV_IDS_CAN*` arrays.

---

## 3. What `lerobot_robot_forte_arm` can and can't do today

| Capability | Works today? | Notes |
| ---------- | :-----------: | ----- |
| Read slave joint positions (`ForteArm.get_observation()`) | Yes | Rate-limited to the Teensy's ~1 Hz status print (§1, §14). |
| Read master joint positions (`ForteArmMasterTeleop.get_action()`) | Yes | Same rate limit. |
| Send `'e'`/`'d'` to the Teensy (`ForteArm.enable()`/`.disable()`) | Yes | Convenience wrapper; you can also just type into the Teensy's serial console directly. |
| Record a teleoperated dataset (`lerobot-record` with `forte_arm` + `forte_arm_master`) | Yes, mechanically | Data quality bottlenecked by the ~1 Hz update rate until §14 is done. |
| Command the slave to a goal position from the host | **No** | No serial command for it exists yet in teensy.ino. |
| Standalone policy control / `lerobot-record --policy.path=...` on `forte_arm` | **No** | Depends on the above. |
| `lerobot-replay` | **No** | Also depends on goal-position control. |

---

## 4. Safety notes

- `ForteArm`/`ForteArmMasterTeleop` never write anything except `'e'`/`'d'` — there is currently no
  way for a bug in this package to command an unsafe motor position, because there is no
  position-command channel at all. The Teensy firmware's own limits (`RAW_LIMIT_MIN/MAX`) are the
  only safety net in play right now.
- `TeensyLink.enable()` (or typing `'e'` directly) makes the Teensy compute a new master/slave
  offset from whatever pose the arms are in *at that moment* — pose them to match first (see
  RUNBOOK.md Phase 3). A mismatched pose becomes a wrong offset for the rest of the session.
- Keep the Teensy's serial console (or a way to send `'d'`) accessible any time the arms are
  enabled — it's still the fastest kill switch available.

---

## 5. Environment setup

```bash
cd /home/daros/workspace2/lerobot_robot_forte_arm
uv sync
```

Requires Python ≥3.12 (matches `lerobot`'s own requirement). `pyproject.toml` depends on the local
`lerobot` checkout with the `intelrealsense` and `pyserial-dep` extras — `pyserial-dep` is what
gives us the `serial` package `teensy_link.py` uses to talk to the Teensy. (Earlier drafts also
pulled in the `robstride` extra for direct CAN access via `python-can`; that's gone now — nothing
in this package touches CAN or python-can anymore.)

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
whatever master/slave positions it received (link-space and raw motor-space degrees). Confirm all
4 joints show up for both master and slave — if some are missing, check the Teensy is powered and
running `teleop-bi`/`teleop-bi-p-t`, and that `--port` is right (`ls /dev/ttyACM* /dev/ttyUSB*`).

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

`--dataset.fps=15` matches the camera config, but note §1/§14: the joint data itself only refreshes
~1x/second regardless of `fps` — this doesn't error or crash anything, it just means the recorded
action/state trajectory is coarser than the video. Fine for validating the full pipeline; do §14's
firmware fix before collecting data you intend to actually train a good policy on.

Data lands at `~/.cache/huggingface/lerobot/${HF_USER}/forte_<task_name>/` (and is pushed to the
Hub unless you pass `--dataset.push_to_hub=false`).

---

## 10. Step 5 — Visualize

```
https://huggingface.co/spaces/lerobot/visualize_dataset  ->  paste ${HF_USER}/forte_<task_name>
```

`lerobot-replay` is **not usable yet** — see §3/§14, it needs the goal-position serial command
that doesn't exist in the firmware today.

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

**Not possible with the current firmware** — see §3. A trained policy has no way to command the
slave arm; there is no serial command for it. This needs §14's firmware addition first.

---

## 13. Troubleshooting log

| Symptom | Cause | Fix |
| ------- | ----- | --- |
| `SerialException: could not open port` | Wrong `--port`, Teensy not plugged in, or something else already has the port open. | `ls /dev/ttyACM* /dev/ttyUSB*`; close any other serial monitor (Arduino IDE, `screen`, etc.) using the same port. |
| `TeensyLink for port '...' already exists with baudrate=... got a conflicting baudrate=...` | `--robot.baudrate` and `--teleop.baudrate` don't match for the same `--port`. | Use the same baudrate (115200 default) on both. |
| Smoke test shows some joints missing | Teensy not running `teleop-bi`/`teleop-bi-p-t`, not yet printed its first status cycle, or a motor fault. | Re-run with a longer `--wait-s`; check the Teensy's own serial output directly for fault messages. |
| Positions look frozen / repeat exactly | Expected — the Teensy only prints ~1x/second (§1). Not a bug. | See §14. |
| Recorded dataset's action/state looks "steppy" (holds a value for several frames, jumps) | Same cause as above, showing up in the data. | See §14 before recording data meant for real training. |

---

## 14. What's still open (in priority order)

1. **[Firmware] Add a fast/on-demand position report to teensy.ino.** The current ~1 Hz
   `Serial.printf` status line is a monitoring afterthought, not designed for a control loop
   consuming it at 10-30 Hz. The straightforward fix: print the same data (or a terser CSV form)
   every control-loop tick, or add a request/response command the host can poll on demand. This is
   the single highest-value next step — everything downstream (dataset quality, eventually
   replay/eval) depends on it. Not done in this session because it means editing and reflashing
   physical hardware firmware, which needs to happen deliberately with someone at the arm.
2. **[Firmware] Add a goal-position serial command.** Needed before `lerobot-replay` or any
   standalone/trained-policy control of the slave arm can work at all — right now the Teensy
   simply has no serial command for it.
3. Confirm the master arm actually shares the slave's gear ratios (assumed, not verified).
4. Wire up `lower_arm_roll`, `wrist_pitch`, gripper on both arms once motorized, extending
   `JOINTS` in `config.py` and the Teensy's `MST_IDS_CAN*`/`SLV_IDS_CAN*` arrays.
5. First real dataset + SmolVLA training run once (1) is done.
