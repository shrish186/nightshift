# NightShift

Retrofit robot cell + AI watchman for CNC machines. See `PLAN.md` (plan, milestones,
business) and `CLAUDE.md` (architecture and safety rules). Everything here runs in
simulation (M0); no hardware yet.

## Setup

```bash
python3 -m venv .venv && .venv/bin/pip install -e ".[dev]"
.venv/bin/pytest -q
```

## Demo: `python -m cell.sim.run`

Live terminal view of the cell: controller state, door/clamp sensors vs physical truth,
arm and gripper, watchman current/vibration sparklines, events, and the unsafe-event
count. `--speed` is sim seconds per real second.

```bash
# clean run: records a watchman reference (supervised), then 5 unattended cycles
.venv/bin/python -m cell.sim.run --cycles 5 --speed 8

# tool breaks mid-cut on cycle 3: the watchman holds the machine, the cell goes SAFE
.venv/bin/python -m cell.sim.run --cycles 5 --speed 8 --fault TOOL_BREAK@MACHINING+4#3

# chip buildup: alert first, stop only past the hard limit (long cut)
.venv/bin/python -m cell.sim.run --cycles 2 --cycle-s 30 --speed 10 --fault CHIP_BUILDUP@CLOSE_DOOR#1

# door jammed + open sensor stuck on: fools the software, the hardware interlock stops the arm
# (record the reference first so cycle 1 opens the door from closed)
.venv/bin/python -m cell.sim.run --no-live --cycles 1 --store data/demo-refs.json
.venv/bin/python -m cell.sim.run --cycles 1 --speed 4 --store data/demo-refs.json \
  --fault DOOR_STUCK@PICK_RAW#1 --fault DOOR_SENSORS_STUCK_OPEN@OPEN_DOOR_LOAD+1.1#1

# single-sensor clamp machine (tug test before letting go), random faults
.venv/bin/python -m cell.sim.run --cycles 6 --no-release-sensor --random-faults 2 --seed 7

.venv/bin/python -m cell.sim.run --list-faults     # every injectable fault
.venv/bin/python -m cell.sim.run --no-live ...     # summary only (CI); --log FILE for JSON events
```

Exit code 0 = no unsafe events (ending in SAFE is a correct outcome); 1 = unsafe events.
