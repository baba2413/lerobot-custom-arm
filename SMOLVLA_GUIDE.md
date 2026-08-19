# Forte Arm × LeRobot SmolVLA — End-to-End Guide

Everything needed to go from "CAN wiring works" to "SmolVLA policy running on the real Forte
arm," using the `lerobot_robot_forte_arm` package in this repo. Written against the current state
of the code as of this session — re-check file contents if a lot of time has passed before you
follow this.

---

## 0. Where things live (workspace2 map)

| Folder                     | Role                                                                                             |
| --------------------------- | -------------------------------------------------------------------------------------------------|
| `lerobot/`                  | The LeRobot library itself (cloned source, not our code). Source of truth for all APIs used here. |
| `lerobot_robot_forte_arm/`  | **This project.** LeRobot integration: `config.py`, `robot.py` (slave), `teleop_master.py` (master), `smoke_test.py`. |
| `teensy-forte/`              | Teensy firmware. **`teleop-bi-p-t` branch, `teensy/teensy.ino`** is the ground truth for the real bilateral master/slave rig: CAN ids, gear ratios, control loop. See §1. |
| `teensy-dash/`               | A separate, older single-arm (non-bilateral) Teensy firmware for the same physical slave arm. Its `GEAR_RATIO` numbers are reused in `config.py` (best available source), but its CAN id scheme does **not** apply to the bilateral rig — don't mix the two up. |
| `isaacsim_script/`           | IsaacSim IK solver + UDP sender, a third, independent way to drive the slave arm (not used by the master/slave rig or by `lerobot_robot_forte_arm`). |
| `urdf/`                      | Forte arm URDF + meshes, used by the IsaacSim IK script. |
| `my_raw_datasets/`           | Local LeRobot-format datasets (currently just the `pusht` example, not Forte data). |

---

## 1. Architecture: the bilateral master/slave rig

Forte is **two physically-identical arms**: a human-driven **master** (leader) and a motorized
**slave** (follower) that mirrors it. This already works today, entirely on a Teensy — ground
truth is `teensy-forte`'s **`teleop-bi-p-t`** branch (`teensy/teensy.ino`, not the default branch
checked out in this repo — `git show teleop-bi-p-t:teensy/teensy.ino` to read it, or
`git checkout teleop-bi-p-t` to flash it).

```
   [human hand]
        |
        v
  MASTER arm (Robstride motors, torque = haptic feedback)
        ^  |
        |  v  CAN1: master ids {1,2}  <-> slave ids {11,12}   (shoulder_yaw, shoulder_roll)
        |  v  CAN2: master ids {3,4}  <-> slave ids {13,14}   (shoulder_pitch, elbow_pitch)
        |  |
        |  v
  SLAVE arm (Robstride motors, torque = position-follow)
        |
        v
   [task / gripper / camera]

  Teensy runs a 500 Hz loop entirely on its own:
    slave_goal_position = master_position + offset      (offset fixed at 'e'-enable time)
    master_feedback_torque = f(slave_contact_torque)     (haptic reflection, clamped/rate-limited)
```

Key facts, all read directly out of `teensy.ino` on `teleop-bi-p-t`:

- **Two independent CAN buses** (Teensy `CAN1`/`CAN2`), each carrying one master/slave joint pair.
- **CAN ids**: master = {1, 2, 3, 4}, slave = **{11, 12, 13, 14}** (slave = master id + 10). This
  supersedes anything said elsewhere (including earlier drafts of this guide / `config.py`) that
  used 1–4 for the slave — that was read off `teensy-dash`'s separate, non-bilateral firmware and
  was wrong for this rig.
- **`HOST_ID = 253`**, MIT mode, 1 Mbps CAN, control loop at 500 Hz (`CONTROL_PERIOD_US = 3333`,
  i.e. ~300 Hz effective — comment says 500 Hz, actual constant gives ~300 Hz, firmware detail, not
  ours to fix here).
- Serial `'e'` enables both arms and computes the per-pair position offset (arm must be roughly in
  a matching pose master vs. slave at that moment); `'d'` disables both.
- Master receives **torque commands** (haptic feedback, `MAX_SAFE_TORQUE = 2.0 Nm`, rate-limited);
  slave receives **position commands** (`SLV_KP = 24.0`, `SLV_KD = 0.2`) plus a virtual-wall
  repulsion (`K_WALL`) if it approaches the raw ±12.4 rad protocol limit.

**Implication for `lerobot_robot_forte_arm`:** the Teensy is a third, independent controller that
already owns the CAN buses during teleoperation. `lerobot_robot_forte_arm` does **not**
reimplement master-follows-slave logic — it only *observes* both arms over the same CAN buses
(read-only) while the Teensy does the actual control, and logs `(observation, action)` pairs for
`lerobot-record`. This is what `ForteArmConfig.owns_actuation=False` and the `forte_arm_master`
teleoperator are for (§3, §6).

The **only** time `lerobot_robot_forte_arm` should actually drive the slave motors itself
(`owns_actuation=True`, the default) is standalone operation with **no human and no Teensy loop
in the picture** — i.e. running a trained policy directly against the slave arm (§9). Never run
the Teensy's bilateral firmware and `owns_actuation=True` against the same motors at the same
time — both would be writing `Goal_Position` to the slave and fighting each other.

**Open question, not verified from code:** whether the host's CAN adapter(s) can physically see
both `CAN1` and `CAN2` traffic — that depends on how your USB-CAN adapter(s) are wired onto the
Teensy's two CAN buses. If you only have one adapter tapped onto one bus, you'll only be able to
read 2 of the 4 joints. `ForteArmConfig`/`ForteArmMasterTeleopConfig` take **two** ports
(`port_can1`, `port_can2`) for exactly this reason — pass the same device path for both only if
you've confirmed they're actually one shared bus.

---

## 2. Current hardware scope

Of the arm's 7 joints, **4 are motorized and wired today** on both master and slave:

| Joint            | CAN bus | Slave CAN id | Master CAN id | Motor:link gear ratio |
| ---------------- | ------- | ------------: | -------------: | ----------------------: |
| `shoulder_yaw`    | can1    | 11            | 1              | 4.8077                  |
| `shoulder_roll`   | can1    | 12            | 2              | 1.0                     |
| `shoulder_pitch`  | can2    | 13            | 3              | 3.180                   |
| `elbow_pitch`     | can2    | 14            | 4              | 1.0                     |
| `lower_arm_roll`  | —       | —             | —              | not wired               |
| `wrist_pitch`     | —       | —             | —              | not wired               |
| gripper           | —       | —             | —              | not wired               |

Gear ratios come from `teensy-dash/src/main.cpp` (the separate single-arm firmware) — **assumed**
identical on the master arm since it's stated to be the same physical arm design, but not
independently re-verified. The bilateral Teensy firmware itself never applies a gear ratio; it
works entirely in raw motor-shaft radians (offset-calibrated per pair at `'e'`-time), so this
assumption doesn't affect the Teensy's own control — only the link-space degrees
`lerobot_robot_forte_arm` reports for the dataset/policy.

`ForteArm` (slave robot) and `ForteArmMasterTeleop` (master teleoperator) both expose exactly
these 4 joints as `{joint}.pos` in **link-space degrees**. Camera: one Intel RealSense (`cam_1`,
serial `825312072171`, 640×480 @ 15 fps, RGB), attached to the slave side (it watches the task).

When the other 3 joints get motors: add them to `JOINTS` in `config.py`, and to the Teensy's
`MST_IDS_CAN*`/`SLV_IDS_CAN*` arrays (which CAN bus they land on is a wiring decision made there).

---

## 3. Two operating modes — don't mix them up

| Mode | `owns_actuation` | Who drives the slave | Use for |
| ---- | :---------------: | --------------------- | ------- |
| **Teleoperated recording** | `False` | Teensy (bilateral firmware, human on master) | `lerobot-record` with `--teleop.type=forte_arm_master` |
| **Standalone policy control** | `True` (default) | This host, via `ForteArm.send_action()` | `lerobot-eval`-style rollout, no Teensy loop running, no master involved |

In teleoperated-recording mode, `ForteArm.connect()` never enables torque or touches motor config
(the Teensy already owns that), and `ForteArm.send_action()` is a no-op that only returns the
(possibly limit-clipped) action for logging — it never writes to the bus. This matters because
`lerobot-record`'s internal loop *always* calls `robot.send_action(teleop.get_action())` every
frame regardless of mode; without this no-op, the host would be writing `Goal_Position` to the
slave at the dataset fps on top of the Teensy's own 500 Hz control, fighting it.

---

## 4. Safety notes (read before connecting torque)

- `ForteArmConfig.motor_type` / `ForteArmMasterTeleopConfig.motor_type` default to `"o0"`,
  carried over from an earlier test script — **not verified against the physical motors'
  nameplate model.** Wrong type scales the MIT position/velocity/torque limits incorrectly. This
  only matters in standalone mode (`owns_actuation=True`) — in teleoperated-recording mode we
  never write MIT commands, so it only affects position/velocity unit conversion, not torque
  safety.
- `ForteArmConfig.joint_limits` and `max_relative_target` default to `None` (unrestricted), and
  are **only enforced in standalone mode**. Set real values before unattended policy rollout:
  ```python
  joint_limits={"shoulder_yaw": (-90, 90), "shoulder_pitch": (-90, 10), ...}
  max_relative_target=5.0   # degrees per send_action() call
  ```
  During teleoperated recording, safety is entirely the Teensy firmware's job
  (`RAW_LIMIT_MIN/MAX`, `K_WALL`, `MAX_SAFE_TORQUE` in `teensy.ino`).
- `robot.buses[...].connect()` alone (as used by `smoke_test.py`) never enables torque — safe to
  run any time, Teensy running or not. Torque is enabled inside `ForteArm.connect()` →
  `configure()`, and only when `owns_actuation=True`.
- In standalone mode, the first `robot.connect()` on new gains/motor-type is the moment to have a
  hand on the e-stop / power switch.
- `disable_torque_on_disconnect` only has an effect when `owns_actuation=True` — in teleoperated
  mode we never enabled torque ourselves, so disconnecting never disables it either (that would
  cut the Teensy's control out from under the operator).

---

## 5. Environment setup

Already done once in this session, documented here for repeatability.

```bash
cd /home/daros/workspace2/lerobot_robot_forte_arm
uv sync
```

Requires Python ≥3.12 (matches `lerobot`'s own requirement). `pyproject.toml` depends on the local
`lerobot` checkout with the `robstride`, `intelrealsense`, and `pyserial-dep` extras (the last one
is required for the `slcan` CAN backend if you're using serial-to-CAN adapters).

To push datasets/policies to the Hub later:
```bash
uv run hf auth login
```

---

## 6. Step 1 — Hardware smoke test (no torque, can't move)

```bash
uv run forte-arm-smoke-test --port-can1 /dev/ttyUSB0 --port-can2 /dev/ttyUSB1
# or, on socketcan:
uv run forte-arm-smoke-test --port-can1 can0 --port-can2 can1 --can-interface socketcan
```

Safe whether or not the Teensy's bilateral firmware is currently running — this only issues
read-only CAN queries. It connects both buses, reads present position on **all 4 master motors
and all 4 slave motors** (prints link-space and raw motor-space degrees side by side), then grabs
one RealSense frame. Confirm:
- All 8 motors respond (no `ConnectionError` / missing ids) — if only 4 respond, you're likely
  only tapped onto one of the two CAN buses (§1's open question).
- Master and slave link-space degrees for each joint are close to each other if the Teensy is
  currently enabled and the operator isn't actively moving the master (they track each other by
  design).
- The camera frame shape is `(480, 640, 3)`.

Only move on once this passes cleanly.

---

## 7. Step 2 — Calibration

Robstride motors report absolute shaft position directly — no incremental-encoder homing needed.
`ForteArm.calibrate()` records a trivial `[-180, 180]` degree range per motor to satisfy the
`Robot` interface's contract, and saves it under
`~/.cache/huggingface/lerobot/calibration/robots/forte_arm/<id>.json`. This is unrelated to (and
does not interfere with) the Teensy's own master/slave position offset, which it computes itself
on `'e'`-enable and never persists to the host.

```bash
uv run lerobot-calibrate --robot.type=forte_arm --robot.port_can1=/dev/ttyUSB0 --robot.port_can2=/dev/ttyUSB1 --robot.id=forte_v1
```

Only needed once per `--robot.id`. Skip this before teleoperated recording if you'd rather not
touch the motors at all before the Teensy takes over — `owns_actuation=False` runs will still work
without a calibration file (calibration is a no-op there).

---

## 8. Step 3 — Sanity-check teleoperation (no recording)

Turn on the Teensy's bilateral firmware first (power up, send `'e'` over its serial console — see
`teleop-bi-p-t`'s `serialEvent()` — and confirm the master mirrors correctly to the slave by hand).
Then, on the host, just observe both sides through LeRobot without writing anything:

```bash
uv run lerobot-teleoperate \
  --robot.type=forte_arm --robot.port_can1=/dev/ttyUSB0 --robot.port_can2=/dev/ttyUSB1 \
  --robot.id=forte_v1 --robot.owns_actuation=false \
  --teleop.type=forte_arm_master --teleop.port_can1=/dev/ttyUSB0 --teleop.port_can2=/dev/ttyUSB1 \
  --display_data=true
```

Confirm the displayed master and slave positions move together as you move the master by hand,
and the camera feed looks right, before recording anything for real.

---

## 9. Step 4 — Record a dataset

```bash
HF_USER=$(NO_COLOR=1 uv run hf auth whoami | awk -F': *' 'NR==1 {print $2}')

uv run lerobot-record \
  --robot.type=forte_arm --robot.port_can1=/dev/ttyUSB0 --robot.port_can2=/dev/ttyUSB1 \
  --robot.id=forte_v1 --robot.owns_actuation=false \
  --teleop.type=forte_arm_master --teleop.port_can1=/dev/ttyUSB0 --teleop.port_can2=/dev/ttyUSB1 \
  --dataset.repo_id=${HF_USER}/forte_<task_name> \
  --dataset.single_task="<one sentence, action-phrased task description>" \
  --dataset.num_episodes=50 \
  --dataset.fps=15 \
  --dataset.episode_time_s=30 \
  --dataset.reset_time_s=10 \
  --display_data=true
```

Notes specific to this setup:
- **`--robot.owns_actuation=false` is not optional here** — see §3. Forgetting it means the host
  starts writing `Goal_Position` to the slave on top of the Teensy's own control loop.
- `--dataset.fps=15` — matches the RealSense camera config in `config.py` (`fps=15`). Don't push
  this higher than the camera's configured fps or you'll get dropped/duplicated frames. It's
  independent of the Teensy's own 500 Hz internal control rate — the dataset just samples
  positions at 15 Hz.
- Controls during recording: **→** next episode, **←** redo, **ESC** finish & upload.
- Start with **50 episodes** of a single, constrained task (fixed object position, fixed camera,
  one operator) — see `lerobot/AGENT_GUIDE.md` §5 for the full data-quality playbook. It applies
  unchanged to Forte.

---

## 10. Step 5 — Visualize and replay before training

Always inspect the data before spending GPU time on it.

```bash
# After upload, paste ${HF_USER}/forte_<task_name> into:
# https://huggingface.co/spaces/lerobot/visualize_dataset
```

`lerobot-replay` drives the robot directly from logged actions and needs `owns_actuation=True`
(there's no master to mirror during replay) — **make sure the Teensy's bilateral firmware is
stopped / slave motors disabled before running this**, or replay and the Teensy will fight over
the same motors exactly like §3 warns about.

```bash
uv run lerobot-replay \
  --robot.type=forte_arm --robot.port_can1=/dev/ttyUSB0 --robot.port_can2=/dev/ttyUSB1 \
  --robot.id=forte_v1 \
  --dataset.repo_id=${HF_USER}/forte_<task_name> --dataset.episode=0
```

Look for: missing/blurry camera frames, joints hitting limits, inconsistent object placement
across episodes.

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

Guidance (from `lerobot/AGENT_GUIDE.md` §6–7, unchanged for Forte):
- **VRAM budget:** SmolVLA needs ~4 GB at batch 1 with SGD; AdamW (LeRobot's default optimizer)
  uses noticeably more. 12–16 GB GPUs run comfortably at the defaults above.
- **Epochs, not raw steps:** aim for 5–10 epochs over the dataset first. With 50 episodes × 30 s @
  15 fps ≈ 22,500 frames, that's roughly `22500/4 × 5 ≈ 28k` to `56k` steps — scale
  `--steps`, `--policy.scheduler_decay_steps`, and `--save_freq` together (see AGENT_GUIDE §7.5).
- **Unfreeze the vision encoder** for a real accuracy gain once the basic pipeline works:
  ```bash
  uv run lerobot-train ... --policy.type=smolvla \
    --policy.freeze_vision_encoder=false --policy.train_expert_only=false
  ```
  Costs more VRAM/step time — try it after a first frozen-encoder run succeeds.

---

## 12. Step 7 — Evaluate on the real arm (standalone, no Teensy, no master)

This is the one place `owns_actuation=True` (the default) is correct: the trained policy replaces
the human/master entirely, so the host must directly drive the slave.

**Physically disable or unplug the Teensy's bilateral firmware / master arm before this step** —
otherwise both the Teensy and the policy will be commanding the slave's `Goal_Position`
simultaneously.

```bash
uv run lerobot-record \
  --robot.type=forte_arm --robot.port_can1=/dev/ttyUSB0 --robot.port_can2=/dev/ttyUSB1 \
  --robot.id=forte_v1 \
  --robot.joint_limits="{shoulder_yaw: [-90,90], shoulder_pitch: [-90,10], shoulder_roll: [-45,45], elbow_pitch: [0,135]}" \
  --robot.max_relative_target=5.0 \
  --dataset.repo_id=${HF_USER}/eval_forte_<task_name> \
  --dataset.single_task="<same task description used during training>" \
  --dataset.num_episodes=10 \
  --dataset.fps=15 \
  --policy.path=${HF_USER}/smolvla_forte_<task_name>
```

`--robot.joint_limits` above is a placeholder — replace with your arm's real safe range (§4). An
undertrained policy can command large, sudden jumps; `max_relative_target` caps per-step movement
as a second line of defense.

Report success rate across the 10 episodes and compare against a teleoperated baseline. If it
fails at one specific point in the task, record 10–20 more demonstrations targeting exactly that
failure, rather than re-collecting everything.

---

## 13. Troubleshooting log (things actually hit while building this)

| Symptom | Cause | Fix |
| ------- | ----- | --- |
| `uv sync` fails: "requested Python version (>=3.10) does not satisfy Python>=3.12" | This package's `pyproject.toml` `requires-python` was looser than `lerobot`'s own `>=3.12`. | Bumped `requires-python = ">=3.12"` here. Already fixed. |
| `ConnectionError: Failed to connect to CAN bus: The serial module is not installed` | `python-can`'s `slcan` backend needs `pyserial`, not pulled in by the `robstride` extra alone. | Added `lerobot[...,pyserial-dep]` to this package's dependencies. Already fixed. |
| `connect()` raises `... No such file or directory: '/dev/ttyUSB0'` | No CAN adapter plugged in / wrong port name. | Check `ls /dev/ttyUSB*` / `ls /dev/ttyACM*`, or `ip link show` for socketcan. |
| Smoke test only sees 4 of 8 motors | Host CAN adapter only physically tapped onto one of `CAN1`/`CAN2`. | See §1's open question — you likely need a second adapter on the other bus. |
| Slave arm jitters / fights the master during recording | `--robot.owns_actuation=false` wasn't passed, so the host is writing `Goal_Position` on top of the Teensy's own control loop. | Always pass it for teleoperated recording (§3, §9). |
| Recording captures master and slave at very different angles | Teensy wasn't `'e'`-enabled (no offset computed yet), or was enabled while master/slave weren't in matching poses. | Re-run the Teensy's enable sequence with both arms in the same pose first. |

---

## 14. What's still open

- [ ] Confirm `motor_type` ("o0") against the actual Robstride motor nameplate/model, both arms.
- [ ] Confirm the master arm actually shares the slave's gear ratios (assumed, not verified).
- [ ] Confirm the host's CAN adapter(s) physically reach both `CAN1` and `CAN2` (§1).
- [ ] Set real `joint_limits` / `max_relative_target` in `ForteArmConfig` for standalone/eval mode.
- [ ] Wire up `lower_arm_roll`, `wrist_pitch`, gripper on both arms once motorized, extending `JOINTS`.
- [ ] First real dataset + SmolVLA training run once the above are done.
