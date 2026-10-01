"""Golden input streams for the hard-stop parity suite.

Both implementations get exactly the same inputs. Three families:
  - sim:   realistic sensor streams from the sim, every watchman fault, both cutting
           modes (detected from current / CNC hint), plus a millisecond-counter wrap.
  - edge:  hand-built streams at every threshold and +/-1 float32 ULP, confirm windows
           +/-1 ms, invalid inputs, flatline/clipping, cut ending mid-confirm.
  - fuzz:  seeded random walks with random step times and hints.
The edge and fuzz streams depend only on the config and fixed seeds (not on the sim),
so they are also the locked set in golden_expected.json.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field

import numpy as np

from cell.clock import SimClock
from cell.drivers.sim.sensors import SensorFault, SimSensors
from cell.watchman.hardstop import HardStopConfig

f32 = np.float32
WRAP = 2**32


@dataclass
class Stream:
    name: str
    t_ms: list[int] = field(default_factory=list)
    current: list[float] = field(default_factory=list)
    vib: list[float] = field(default_factory=list)
    clipped: list[bool] = field(default_factory=list)
    hint: list[int] = field(default_factory=list)

    def add(self, t: int, a: float, v: float, clipped: bool = False, hint: int = -1) -> None:
        self.t_ms.append(t % WRAP)
        self.current.append(float(a))
        self.vib.append(float(v))
        self.clipped.append(clipped)
        self.hint.append(hint)

    def __len__(self) -> int:
        return len(self.t_ms)


def up(x: float) -> float:
    return float(np.nextafter(f32(x), f32(np.inf)))


def down(x: float) -> float:
    return float(np.nextafter(f32(x), f32(-np.inf)))


# --- sim ---


def sim_stream(seed: int, fault: SensorFault | None, cnc_hint: bool, t0: int = 0) -> Stream:
    clock = SimClock()
    schedule = [(5.0, False), (12.0, True)] * 4  # idle 5 s, cut 12 s, x4
    cutting = [False]
    sensors = SimSensors(clock, cutting=lambda: cutting[0], seed=seed)
    name = f"sim-s{seed}-{fault.name if fault else 'clean'}-{'cnc' if cnc_hint else 'detect'}"
    s = Stream(name + (f"-t0{t0}" if t0 else ""))
    for i, (dur, cut) in enumerate(schedule):
        cutting[0] = cut
        for k in range(int(dur * 10)):
            if fault is not None and i == 3 and k == 40:  # second cut, 4 s in
                sensors.inject(fault)
            f = sensors.read()
            assert f is not None
            s.add(t0 + round(clock.now() * 1000), f.spindle_current_a, f.vibration_rms_g,
                  hint=int(cut) if cnc_hint else -1)  # fmt: skip
            clock.advance(0.1)
    return s


def sim_streams() -> list[Stream]:
    faults = [None, SensorFault.TOOL_BREAK, SensorFault.JAM, SensorFault.CHIP_BUILDUP,
              SensorFault.TOOL_WEAR]  # fmt: skip
    out = [sim_stream(seed, f, hint) for seed in range(3) for f in faults for hint in (False, True)]
    out += [sim_stream(9, SensorFault.TOOL_BREAK, False, t0=WRAP - 30_000)]  # wraps mid-run
    return out


# --- edge ---


def _cut(s: Stream, t: int, ms: int, a: float, v: float = 0.8, step: int = 100) -> int:
    for k in range(0, ms, step):
        s.add(t + k, a, v)
    return t + ms


def edge_streams(c: HardStopConfig) -> list[Stream]:
    out = []

    # cut-on threshold: exactly at it never starts a cut; 1 ULP above does.
    s = Stream("edge-cut-on-threshold")
    t = _cut(s, 0, 2000, c.cut_on_a)
    t = _cut(s, t, 2000, up(c.cut_on_a))
    out.append(s)

    # cut-on confirm: above for cut_on_ms - 1 then drop (no cut), then exactly cut_on_ms.
    for extra, label in ((-1, "short"), (0, "exact")):
        s = Stream(f"edge-cut-on-confirm-{label}")
        t = 1000
        s.add(0, 0.5, 0.05)
        s.add(t, 12.0, 0.8)
        s.add(t + c.cut_on_ms + extra, 12.0, 0.8)
        s.add(t + c.cut_on_ms + extra + 1, 0.5, 0.05)
        out.append(s)

    # cut-off: hysteresis band (between off and on) keeps cutting; below off for the
    # confirm window ends it, at exactly cut_off_a it doesn't.
    s = Stream("edge-cut-off")
    t = _cut(s, 0, 2000, 12.0)
    mid = (c.cut_on_a + c.cut_off_a) / 2
    t = _cut(s, t, c.cut_off_ms + 1000, mid)
    t = _cut(s, t, c.cut_off_ms + 1000, c.cut_off_a)
    t = _cut(s, t, c.cut_off_ms + 1000, down(c.cut_off_a), v=0.05)
    out.append(s)

    # overload current: exactly at limit (no), 1 ULP above for confirm-1 ms and confirm ms.
    s = Stream("edge-overload-current")
    t = _cut(s, 0, 3000, 12.0)
    t = _cut(s, t, 1000, c.overload_a)
    t = _cut(s, t, 500, 12.0)
    for k in (0, c.overload_confirm_ms - 1):
        s.add(t + k, up(c.overload_a), 0.8)
    t += c.overload_confirm_ms
    s.add(t, 12.0, 0.8)
    t += 500
    for k in (0, c.overload_confirm_ms):
        s.add(t + k, up(c.overload_a), 0.8)
    out.append(s)

    # overload vibration at the limit and 1 ULP above.
    s = Stream("edge-overload-vibration")
    t = _cut(s, 0, 3000, 12.0)
    t = _cut(s, t, 1000, 12.0, v=c.vib_max_g)
    _cut(s, t, 1000, 12.0, v=up(c.vib_max_g))
    out.append(s)

    # tool break: holding exactly at ratio x mean never fires (those samples count toward
    # the mean, so the limit only drops); 1 ULP below a steady mean fires after confirm.
    limit = float(f32(f32(c.break_ratio) * f32(12.0)))
    s = Stream("edge-tool-break-at-limit")
    t = _cut(s, 0, c.settle_ms + 2000, 12.0)
    _cut(s, t, c.break_confirm_ms + 1000, limit)
    out.append(s)
    s = Stream("edge-tool-break-below")
    t = _cut(s, 0, c.settle_ms + 2000, 12.0)
    _cut(s, t, c.break_confirm_ms + 300, down(limit))
    out.append(s)

    # tool break during settle is ignored; mean needs min_settled samples.
    s = Stream("edge-break-during-settle")
    t = _cut(s, 0, 400, 12.0)
    _cut(s, t, c.settle_ms + 1000, 3.0)
    out.append(s)

    # cut ends while a tool-break confirm is pending (CNC hint drops).
    s = Stream("edge-cut-ends-mid-confirm")
    for k in range(0, c.settle_ms + 2000, 100):
        s.add(k, 12.0, 0.8, hint=1)
    t = c.settle_ms + 2000
    s.add(t, 2.0, 0.8, hint=1)
    s.add(t + c.break_confirm_ms // 2, 2.0, 0.8, hint=0)
    s.add(t + c.break_confirm_ms + 50, 2.0, 0.8, hint=1)
    s.add(t + 2 * c.break_confirm_ms, 2.0, 0.8, hint=1)
    out.append(s)

    # invalid inputs: NaN, +/-inf, negative, each on its own step, mid-cut.
    s = Stream("edge-invalid-inputs")
    t = _cut(s, 0, 2000, 12.0)
    for a, v in [(float("nan"), 0.8), (12.0, float("nan")), (float("inf"), 0.8),
                 (12.0, float("-inf")), (-1.0, 0.8), (12.0, -0.001), (12.0, 0.8)]:  # fmt: skip
        t += 100
        s.add(t, a, v)
    out.append(s)

    # sensor faults: vibration flat while cutting, and ADC clipping, around the window.
    s = Stream("edge-sensor-faults")
    t = _cut(s, 0, 2000, 12.0)
    t = _cut(s, t, c.sensor_fault_ms + 500, 12.0, v=down(c.vib_floor_g))
    t = _cut(s, t, 500, 12.0)
    for k in range(0, c.sensor_fault_ms + 300, 100):
        s.add(t + k, 12.0, 0.8, clipped=True)
    out.append(s)

    # millisecond counter wraps during a tool-break confirm window.
    s = Stream("edge-wraparound")
    t0 = WRAP - c.settle_ms - 2000
    t = _cut(s, t0, c.settle_ms + 1800, 12.0)
    _cut(s, t, c.break_confirm_ms + 400, 1.0)
    out.append(s)
    return out


# --- fuzz ---


def fuzz_streams(n: int = 12, steps: int = 3000) -> list[Stream]:
    out = []
    for seed in range(n):
        rng = random.Random(seed)
        s = Stream(f"fuzz-{seed}")
        t = rng.choice([0, WRAP - 50_000])
        a, v = 1.0, 0.05
        hint_mode = rng.choice([-1, 0, 1])
        for _ in range(steps):
            t += rng.choice([1, 50, 100, 100, 100, 250, 1000])
            a = max(0.0, a + rng.gauss(0, 1.5)) if rng.random() > 0.02 else rng.uniform(0, 40)
            v = max(0.0, v + rng.gauss(0, 0.2)) if rng.random() > 0.02 else rng.uniform(0, 6)
            if rng.random() < 0.003:
                a = rng.choice([float("nan"), float("inf"), -0.5])
            hint = hint_mode if hint_mode < 0 else int(rng.random() < 0.7)
            s.add(t, a, v, clipped=rng.random() < 0.01, hint=hint)
        out.append(s)
    return out
