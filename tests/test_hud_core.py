"""Headless verification of hud_core.js (HUD aggregators) — module 5 of the
client front-end. Verifies: median/p95/max with
known sets (even/odd/1/empty→null not NaN), PIN of the nearest-rank convention
against the player formula (pct of causal_player), accumulation of skipped and
snapshot idempotency. The DOM overlay (hud.js) is validated by visual inspection.

pytest tests/test_hud_core.py -v
"""
from __future__ import annotations

import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_hud_core_headless():
    r = subprocess.run(["node", str(ROOT / "tests" / "hud_core_check.mjs")],
                       capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, f"hud_core FALLÓ:\n{r.stdout}\n{r.stderr}"
    assert "OK:" in r.stdout
