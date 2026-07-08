"""vision --3d — LAZY import: the modes without 3D (--readout/--split/live) do NOT
load ws_emitter or websockets. Verified by importing the vision-line module and
asserting on sys.modules (the web imports live ONLY in the --3d branch
`_run_web3d` and in `build_vision_view`, which only runs with the emitter).

.venv/bin/python -m pytest tests/test_vision_lazy.py -v
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

_CHILD = r"""
import sys
sys.path.insert(0, r"{root}/scripts")
sys.path.insert(0, r"{root}/scripts/demo_causal")
import run_harness_causal           # noqa: F401  (línea de visión: readout/split/live)
web = [m for m in sys.modules
       if m == "ws_emitter" or m.startswith("ws_emitter.")
       or m == "websockets" or m.startswith("websockets.")]
assert not web, f"la línea de visión cargó web al importar: {{web}}"
print("LAZY_OK")
"""


def test_linea_vision_no_carga_web_al_importar():
    child = _CHILD.format(root=str(ROOT))
    r = subprocess.run([sys.executable, "-c", child],
                       capture_output=True, text=True, timeout=180)
    assert r.returncode == 0, r.stderr[-2000:]
    assert "LAZY_OK" in r.stdout
