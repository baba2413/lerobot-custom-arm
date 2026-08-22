# lerobot_robot_forte_arm

Host-side LeRobot integration for "Forte," a custom Robstride-motor robot arm rig (a teleoperated
master+slave pair for data collection, and a standalone slave-only build for policy eval). This
package plugs into `lerobot` as robot/teleoperator subclasses (`draccus` `RobotConfig`/
`TeleoperatorConfig` registry) — it is not a fork of `lerobot`, it's a thin adapter.

**Read `RUNBOOK.md` and `SMOLVLA_GUIDE.md` before making non-trivial changes.** `RUNBOOK.md` is
the literal step-by-step operator checklist (what a human physically does, in order, phase by
phase). `SMOLVLA_GUIDE.md` is the background/rationale doc (why the pipeline works the way it
does, current limitations, open items in §14). This file (`CLAUDE.md`) is neither — it's oriented
at *you*, a Claude session picking up work here, and stays short on purpose. Don't duplicate those
two files' content back into this one as they change; update the doc whose job it actually is.

## The two repos

- **This repo** (`lerobot_robot_forte_arm`) — host-side Python. What you're in now.
- **`../teensy-forte`** (sibling directory, separate git repo) — the Teensy 4.x firmware
  (`teensy/teensy.ino`) that actually drives the CAN bus and talks to the host. **Every branch in
  that repo is an independent, standalone firmware snapshot for one specific job — not a feature
  branch meant to merge.** See `teensy-forte/BRANCHES.md` for what each branch does and which one
  to flash for what. The two branches this package actually talks to:
  - `teleop-bi-c` — bilateral teleoperation (master+slave), used for `lerobot-record`/
    `lerobot-teleoperate` via `ForteArm`/`ForteArmMasterTeleop`.
  - `goal` — standalone slave-only eval firmware, used via `ForteArmGoal`.

  Never assume matching feature parity between `teensy-forte` branches. If a firmware change is
  needed, it happens on the one specific branch the relevant Python class talks to, in place —
  don't port bilateral logic into `goal`, don't add goal-following to `teleop-bi-c`, etc.

## Hardware link: no direct CAN from the host, ever

The host has no CAN adapter. The **only** link to the motors is through the Teensy, which exposes
two channels depending on which branch is flashed:
- **USB serial** — human-supervised, single-character commands (`'e'`/`'c'`/`'d'`), sent by a
  person typing into a serial console (`minicom`/`screen`), never by this package's Python code.
- **Ethernet UDP** (direct cable, static IPs, no gateway) — the continuous machine-rate channel:
  telemetry (Teensy → host, `teleop-bi-c`) and/or goal-position streaming (host → Teensy, `goal`).

`teensy_link.py` has two classes for this, deliberately not sharing a base class (different wire
protocols):
- **`TeensyLink`** (bilateral, `teleop-bi-c` only) — **UDP-only, zero serial code path.** No
  `enable()`/`disable()`/`calibrate()` methods exist here on purpose: the operator sends
  `'e'`/`'c'`/`'d'` directly at the Teensy over `minicom`, which stays open for the entire
  `lerobot-record` session (including every episode's reset window) because Python never needs
  the serial port at all. Don't add write methods back onto this class without a very good reason
  — that was a deliberate, discussed design choice, not an oversight.
- **`TeensyGoalLink`** (`goal` branch only) — **UDP-only, zero serial code path, same as
  `TeensyLink`.** Two independent one-way UDP streams: `send_goal()` (host → Teensy, the
  continuous goal-position stream, fire-and-forget, 500ms firmware watchdog) and
  `get_positions_rad()`'s telemetry (Teensy → host, mirroring `TeensyLink`'s telemetry). No
  `calibrate()`/`disable()` methods here either — `'c'`/`'d'` are typed directly at the Teensy
  over minicom/screen, same rationale as `TeensyLink`.

If you're asked to add a new command Python needs to send at machine rate, it goes over UDP, not
serial — serial in this codebase is reserved for human-typed, single-character, occasional
commands.

## Units: radians, everywhere, no exceptions

Every `.pos` value in this pipeline — recorded dataset, live teleop, goal commands — is a raw
motor-shaft **radian**, matching the Robstride/firmware convention (`P_MIN`/`P_MAX`, the status
line, the UDP wire protocol are all radians natively). There is:
- **No gear-ratio conversion.** `config.JOINTS`' gear ratios are recorded as hardware
  documentation only; nothing applies them. A trained policy doesn't care whether a number is
  "physically real," only that recording and eval agree — so a conversion here would only be
  another place for a train/eval mismatch to hide.
- **No degrees anywhere.** An earlier revision converted rad→deg on every read and deg→rad back on
  every write, for no functional reason — removed for the identical reason above.

If you're tempted to add a unit conversion "for readability," don't — it has been explicitly
rejected twice in this project's history for the same root reason. Real physical angles (if ever
needed for kinematics or safety limits) belong in a presentation/analysis layer, never threaded
back through the record/train/eval path.

## Delta-from-baseline, not absolute position

The Robstride protocol's `UNCALIBRATED` fault bit implies the motor's absolute position reference
isn't guaranteed stable across power cycles, so `.pos` values are zero-relative, not raw absolute
— but *how* they're zeroed differs by pipeline, and this is a common source of confusion:
- **Bilateral (`ForteArm`/`ForteArmMasterTeleop`)**: zeroing happens **firmware-side**, on
  `teleop-bi-c`, via the operator sending `'c'` at the Teensy. Host-side `_baseline_rad` is a
  fixed `0.0` for every motor — the class trusts whatever the firmware's status line reports.
- **Eval (`ForteArmGoal`)**: zeroing happens **host-side**, dynamically, via `wait_for_positions()`
  capturing each motor's first reading at `connect()` as that session's baseline.

Both require the arm to be physically posed the same way (e.g. a marked "home" pose) at the start
of every session — this is a manual discipline, not something the code verifies. See
`SMOLVLA_GUIDE.md` §2/Phase 4b in `RUNBOOK.md`.

## Environment: `uv`, not conda

`uv sync` manages an isolated `.venv/` here; every dependency (torch, lerobot, transformers,
accelerate, etc.) is isolated in it. `uv run <command>` always uses it automatically. There is no
conda environment involved in this project, regardless of what's on the host's `PATH` — ignore any
system/conda Python entirely. `lerobot` itself is a local editable checkout
(`file:///home/daros/workspace2/lerobot`) pulled in via extras in `pyproject.toml`
(`intelrealsense`, `hardware`, `viz`, `dataset`, `training`, `smolvla`) — if a command fails with
`ImportError: '<pkg>' is required but not installed`, the fix is almost always adding the right
extra there and re-running `uv sync`, not `pip install`ing directly into the venv.

## Repo layout — what's live vs dead code

- `robot.py` (`ForteArm`), `teleop_master.py` (`ForteArmMasterTeleop`) — bilateral, `teleop-bi-c`.
- `robot_goal.py` (`ForteArmGoal`) — eval, `goal` branch.
- `teensy_link.py` — both link classes (`TeensyLink`, `TeensyGoalLink`) plus `wait_for_positions()`.
- `config.py` — `JOINTS` mapping, `ForteArmConfig`, `ForteArmMasterTeleopConfig`, `ForteArmGoalConfig`.
- `smoke_test.py`, `goal_limit_bench.py` — standalone diagnostic CLI scripts (`forte-arm-smoke-test`,
  `forte-arm-goal-limit-bench` console scripts in `pyproject.toml`).
- `camera.py`, `motors_bus.py`, `robstride_motors_bus.py`, `test.py` — **dead code**, not imported
  anywhere in the active pipeline. Leftovers from an earlier, abandoned design that talked to CAN
  directly from the host via `python-can` (see `SMOLVLA_GUIDE.md` §1 for why that got scrapped).
  Don't build on these or assume they reflect current architecture; safe to ignore or delete.

Plugin registration note: `ForteArmGoal` (in `robot_goal.py`) is never explicitly imported by
`__init__.py`, yet `--robot.type=forte_arm_goal` still resolves. This isn't broken — `lerobot`'s
`make_device_from_device_class()` fallback derives the device class name by stripping `Config` off
the registered config class name (`ForteArmGoalConfig` → `ForteArmGoal`) and searches sibling
modules for it. Keep new robot/teleop classes following the `<Thing>` class in a module discoverable
next to `<Thing>Config`'s registration if you add one, rather than assuming you need to wire up
explicit imports.

## Working conventions specific to this project

- **Every recorded dataset needs a genuinely unique `--dataset.repo_id`.** There's no automatic
  collision handling; a common pattern seen in practice is appending a timestamp to the task name.
  Whatever string is used at record time is the literal, exact string needed again at train/eval
  time — it is not a category label. See RUNBOOK.md Phase 6b for renaming/deleting datasets you no
  longer want (locally and on the Hub — they're independent, a local `mv`/`rm` never touches Hub).
- **This environment has no git credential helper or `gh` CLI.** `git push` will fail here every
  time (`could not read Username for 'https://github.com'`) — this is expected, not a bug to fix.
  Commit locally as normal; the user pushes from their own terminal.
- **Only commit when asked.** Implementing a fix and committing it are treated as separate asks in
  this project's history — don't assume "fix this" implies "and commit it," though it very often
  is followed by an explicit "commit" request right after.
- **Firmware changes go on the one specific branch the task needs, in place** — not a new branch,
  unless the user explicitly asks for one. Confirm branch/IP/port defaults explicitly when they
  aren't already pinned down by a prior decision; don't silently invent networking parameters.
- **Root-cause fixes over symptom patches**, especially for anything touching what gets recorded
  and later trained on — a mismatch in the training data distribution should be fixed at the
  source, not masked with a clamp or a fallback at the point it becomes visible.
