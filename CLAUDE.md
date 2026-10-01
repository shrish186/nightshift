# CLAUDE.md — NightShift

## What this project is
A retrofit robot cell that loads/unloads existing CNC lathes and VMCs overnight in Indian machine shops, with an AI "watchman" that detects tool breakage, jams and chip buildup from spindle current, vibration and sound, then stops safely and alerts the owner. Sold as a rental billed per extra machine-hour. See `PLAN.md` for the full plan and milestones.

**Current priority (until January): the watchman as a standalone product for December shop pilots**: an ESP32-S3 node per machine, a Raspberry Pi hub per shop, and a cloud owner app. The arm/cell code (`cell/controller`, `cell/drivers`) is frozen as-is; change it only when a founder asks. Design: `docs/watchman-architecture.md`; hardware: `hardware/bom.md`.

## Team
- Shrish — backend, cloud, AI infra, business side
- CS cofounder — cell controller, vision, watchman
- Hardware cofounder — arm integration, gripper, fixtures, guarding, wiring

## Architecture rules
- Every piece of hardware sits behind an interface in `cell/drivers/` with a sim or mock implementation. Business logic never imports a vendor SDK directly.
- The cell controller is an explicit state machine. New behavior = new state or transition + tests, not ad-hoc flags.
- The watchman runs independently of the controller and can command a feed-hold/stop through `CncIo` at any time.
- The telemetry agent must work offline and sync later. Assume the shop's internet drops for hours.

## Repo layout
- `cell/`: robot cell (controller, drivers + sims, watchman used by the cell). Frozen until January.
- `node/lib/hardstop/`: portable C99 hard-stop rules shared by the firmware and the parity tests. `node/firmware/`: ESP32-S3 PlatformIO project.
- `hub/`: Raspberry Pi shop service (MQTT ingest, adaptive detectors, references, operator page).
- `telemetry/`: offline-first queue + outbound sync. `cloud/`: owner web app + API.
- `tools/record/`: makerspace recording tool (Parquet + sidecar JSON).
- `docs/`, `hardware/`: design and BOM.

## Watchman rules (non-negotiable)
- The watchman is a monitoring and process-stop device, not a safety function. Operator safety stays with the machine's guarding and e-stop.
- The node's hard-stop rules run locally and never wait on the network.
- Feed-hold relay: energised = hold. A dead node leaves the machine running. A latched hold clears only with the node's local button.
- **Never remote start, resume or release.** No such message, route, API or firmware command may exist. Remote pause only, signed, logged with who and when, and confirmed by the node.
- Alert-only is the default. Auto-stop is an explicit opt-in per machine.
- The C hard-stop rules (`node/lib/hardstop`) and the Python reference (`cell/watchman/hardstop.py`) must make identical decisions on the golden datasets. Any change to either needs the parity suite green.
- Secrets (API keys, tokens, node keys) only in `.env` files that are never committed. Never echo them.

## Safety rules (non-negotiable)
- Physical safety is hardware: guarding, e-stops, interlocks. Software never overrides, bypasses, or simulates away a hardware safety signal.
- On real hardware, e-stop and guard circuits go through a certified safety relay. Python and the edge computer are never part of the safety circuit.
- Never remove, weaken, or "simplify" a safety check, timeout, or interlock read, even to make a test pass. If a test fails because of a safety check, the test or the logic above it is wrong.
- Any unknown or unexpected state → go to SAFE (robot stopped at a safe pose, CNC feed hold, alert sent).
- Code touching `cell/controller`, `cell/drivers/cnc_io`, `node/lib/hardstop`, the relay or remote-pause path, or anything named `safety` requires plan mode and a human review before merging.
- Every safety or hard-stop rule gets a mutant: switch it off and prove a test fails. Commit before mutation runs; restore mutated files from a backup, never with `git checkout`.

## Conventions
- Python 3.12+ (the hub runs in Docker), type hints everywhere, `ruff` + `mypy`, `pytest`.
- Tests first for state-machine behavior. Fault injection tests live in `tests/faults/`.
- Sensor data is saved as Parquet with a sidecar JSON of machine, tool, material and labels. Raw data goes in `data/` (git-ignored).
- Config in YAML per cell (`cells/<cell-id>.yaml`); no hardcoded poses, timeouts or pin numbers in code.
- Log in structured JSON; every state transition is logged with a timestamp and reason.

## Working style
- Start by reading `PLAN.md` section 4 to find the current milestone.
- Propose a short plan before writing code for anything non-trivial.
- Prefer small changes: one driver, one state, one feature per change.
- When hardware behavior is uncertain, write it as a mock with the assumption stated in a comment, and flag it for the humans to verify on real hardware.
