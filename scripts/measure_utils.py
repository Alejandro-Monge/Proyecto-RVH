"""Measurement utilities shared by the diagnostic/real-time scripts.

GPU sampling (best-effort via nvidia-smi) and timing summary. Not part of the
pipeline; it lives in scripts/ because it is measurement instrumentation.
"""

from __future__ import annotations

import shutil
import statistics
import subprocess
import threading
from dataclasses import dataclass
from typing import Optional


@dataclass
class GpuSummary:
    available: bool
    util_mean: Optional[float] = None
    util_peak: Optional[float] = None
    mem_peak_mb: Optional[float] = None


class GpuSampler:
    """Samples GPU utilization and memory in a background thread (best-effort).

    Uses `nvidia-smi`; if absent, it stays idle and reports `available=False`.
    """

    def __init__(self, interval_s: float = 0.1) -> None:
        self._interval = interval_s
        self._util: list[float] = []
        self._mem: list[float] = []
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._available = shutil.which("nvidia-smi") is not None

    def _poll(self) -> None:
        while not self._stop.is_set():
            try:
                out = subprocess.check_output(
                    [
                        "nvidia-smi",
                        "--query-gpu=utilization.gpu,memory.used",
                        "--format=csv,noheader,nounits",
                    ],
                    text=True,
                ).strip().splitlines()[0]
                util_s, mem_s = out.split(",")
                self._util.append(float(util_s))
                self._mem.append(float(mem_s))
            except Exception:
                pass
            self._stop.wait(self._interval)

    def __enter__(self) -> "GpuSampler":
        if self._available:
            self._thread = threading.Thread(target=self._poll, daemon=True)
            self._thread.start()
        return self

    def __exit__(self, *exc) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def summary(self) -> GpuSummary:
        if not self._available or not self._util:
            return GpuSummary(available=self._available)
        return GpuSummary(
            available=True,
            util_mean=statistics.mean(self._util),
            util_peak=max(self._util),
            mem_peak_mb=max(self._mem) if self._mem else None,
        )


@dataclass
class TimingSummary:
    n: int
    mean_ms: float
    p50_ms: float
    p95_ms: float

    @property
    def fps(self) -> float:
        return 1000.0 / self.mean_ms if self.mean_ms > 0 else float("inf")


def summarize_ms(samples_ms: list[float]) -> TimingSummary:
    """Summarizes a list of per-frame latencies (ms)."""
    ordered = sorted(samples_ms)
    n = len(ordered)
    p50 = ordered[int(0.50 * (n - 1))]
    p95 = ordered[int(0.95 * (n - 1))]
    return TimingSummary(
        n=n, mean_ms=statistics.mean(ordered), p50_ms=p50, p95_ms=p95
    )
