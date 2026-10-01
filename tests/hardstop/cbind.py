"""Compile node/lib/hardstop/hardstop.c on the host and call it through ctypes."""

from __future__ import annotations

import ctypes
import os
import shutil
import subprocess
from pathlib import Path

from cell.watchman.hardstop import HardStopConfig

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "node" / "lib" / "hardstop" / "hardstop.c"
# Same flags as the firmware: no FMA contraction (it changes float rounding).
CFLAGS = ["-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic", "-ffp-contract=off"]


class HsConfig(ctypes.Structure):
    _fields_ = [  # must match hs_config_t field for field
        ("cut_on_a", ctypes.c_float),
        ("cut_off_a", ctypes.c_float),
        ("cut_on_ms", ctypes.c_uint32),
        ("cut_off_ms", ctypes.c_uint32),
        ("settle_ms", ctypes.c_uint32),
        ("break_ratio", ctypes.c_float),
        ("break_confirm_ms", ctypes.c_uint32),
        ("min_settled", ctypes.c_uint32),
        ("overload_a", ctypes.c_float),
        ("vib_max_g", ctypes.c_float),
        ("overload_confirm_ms", ctypes.c_uint32),
        ("vib_floor_g", ctypes.c_float),
        ("sensor_fault_ms", ctypes.c_uint32),
    ]


def compile_lib(out_dir: Path) -> ctypes.CDLL:
    cc = os.environ.get("CC", "cc")
    if shutil.which(cc) is None:
        raise RuntimeError(f"C compiler {cc!r} not found: the parity suite needs one")
    lib = out_dir / "libhardstop.so"
    subprocess.run([cc, *CFLAGS, "-shared", "-fPIC", str(SRC), "-o", str(lib), "-lm"], check=True)
    dll = ctypes.CDLL(str(lib))
    dll.hs_init.argtypes = [ctypes.c_void_p, ctypes.POINTER(HsConfig)]
    dll.hs_step.argtypes = [
        ctypes.c_void_p, ctypes.c_uint32, ctypes.c_float, ctypes.c_float, ctypes.c_int, ctypes.c_int
    ]  # fmt: skip
    dll.hs_step.restype = ctypes.c_uint32
    dll.hs_state_size.restype = ctypes.c_uint32
    dll.hs_cut_mean.argtypes = [ctypes.c_void_p]
    dll.hs_cut_mean.restype = ctypes.c_float
    dll.hs_rms.argtypes = [
        ctypes.POINTER(ctypes.c_int16), ctypes.c_uint32, ctypes.c_float,
        ctypes.POINTER(ctypes.c_uint32),
    ]  # fmt: skip
    dll.hs_rms.restype = ctypes.c_float
    return dll


class CHardStop:
    """Same interface as cell.watchman.hardstop.HardStop, backed by the C library."""

    def __init__(self, dll: ctypes.CDLL, cfg: HardStopConfig) -> None:
        self._dll = dll
        self._state = ctypes.create_string_buffer(dll.hs_state_size())
        c = HsConfig(**{f[0]: getattr(cfg, f[0]) for f in HsConfig._fields_})
        dll.hs_init(self._state, ctypes.byref(c))

    def step(self, t_ms: int, current_a: float, vib_g: float, clipped: bool, hint: int) -> int:
        return int(self._dll.hs_step(self._state, t_ms & 0xFFFFFFFF, current_a, vib_g,
                                     int(clipped), hint))  # fmt: skip

    def cut_mean(self) -> float:
        return float(self._dll.hs_cut_mean(self._state))


def c_rms(dll: ctypes.CDLL, samples: list[int], scale: float) -> tuple[float, int]:
    arr = (ctypes.c_int16 * len(samples))(*samples)
    clipped = ctypes.c_uint32(0)
    value = dll.hs_rms(arr, len(samples), scale, ctypes.byref(clipped))
    return float(value), int(clipped.value)
