# CLAUDE.md — NightShift

## What this project is
A retrofit robot cell that loads/unloads existing CNC lathes and VMCs overnight in Indian machine shops, with an AI "watchman" that detects tool breakage, jams and chip buildup from spindle current, vibration and sound, then stops safely and alerts the owner. Sold as a rental billed per extra machine-hour. See `PLAN.md` for the full plan and milestones.

## Team
- Shrish — backend, cloud, AI infra, business side
- CS cofounder — cell controller, vision, watchman
- Hardware cofounder — arm integration, gripper, fixtures, guarding, wiring

## Architecture rules
- Every piece of hardware sits behind an interface in `cell/drivers/` with a sim or mock implementation. Business logic never imports a vendor SDK directly.
- The cell controller is an explicit state machine. New behavior = new state or transition + tests, not ad-hoc flags.
- The watchman runs independently of the controller and can command a feed-hold/stop through `CncIo` at any time.
- The telemetry agent must work offline and sync later. Assume the shop's internet drops for hours.

## Safety rules (non-negotiable)
- Physical safety is hardware: guarding, e-stops, interlocks. Software never overrides, bypasses, or simulates away a hardware safety signal.
- Never remove, weaken, or "simplify" a safety check, timeout, or interlock read, even to make a test pass. If a test fails because of a safety check, the test or the logic above it is wrong.
- Any unknown or unexpected state → go to SAFE (robot stopped at a safe pose, CNC feed hold, alert sent).
- Code touching `cell/controller`, `cell/drivers/cnc_io`, or anything named `safety` requires plan mode and a human review before merging.

## Conventions
- Python 3.11+, type hints everywhere, `ruff` + `mypy`, `pytest`.
- Tests first for state-machine behavior. Fault injection tests live in `tests/faults/`.
- Sensor data is saved as Parquet with a sidecar JSON of machine, tool, material and labels. Raw data goes in `data/` (git-ignored).
- Config in YAML per cell (`cells/<cell-id>.yaml`); no hardcoded poses, timeouts or pin numbers in code.
- Log in structured JSON; every state transition is logged with a timestamp and reason.

## Working style
- Start by reading `PLAN.md` section 4 to find the current milestone.
- Propose a short plan before writing code for anything non-trivial.
- Prefer small changes: one driver, one state, one feature per change.
- When hardware behavior is uncertain, write it as a mock with the assumption stated in a comment, and flag it for the humans to verify on real hardware.
