"""Shared infrastructure for the paper's FINAL FIGURES (PEWASUN) — a SEPARATE LAYER from production.

This package (scripts/paper_figures/) ONLY READS the production state and generates figures in
verify_out/imagenes_finales/. It does not touch reconstruct.py, the net, the detector, data or GT.

Two pieces:

1) PROVENANCE GUARD (`check_provenance`): PINNED md5s of the current production state.
   If an input does not match, generation ABORTS: it is impossible to produce a figure over
   old or accidentally mutated data. If production changes legitimately, the pins are
   updated deliberately, never silently.

2) EDITORIAL STYLE (`apply_style`, `fig_size`, `save_fig`): paper column dimensions
   parameterized in ONE constant, 7–8 pt serif typography, thin marks, hairline grid,
   no titles inside the image (the caption goes in the paper). Validated categorical colors
   (CVD ΔE 73.6).
"""
from __future__ import annotations

import hashlib
import subprocess
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
OUT_DIR = ROOT / "verify_out" / "imagenes_finales"
DATA_DIR = OUT_DIR / "datos"          # intermediate derivatives of the figures package (not production)

# --------------------------------------------------------------------------- #
# 1) Provenance guard — md5 of the production state
# --------------------------------------------------------------------------- #
# The pins are re-verified on disk before generating any figure.
PINNED_MD5 = {
    # --- perception/events data (post-agnostic_nms + corrected GT) ---
    "verify_out/tracks_rusia.csv": "91598cce96feb632afe45372061f64a4",
    "verify_out/events/rusia/track_events.csv": "d5f1260d8d83ed633278f8c810fa570f",
    "verify_out/events/rusia/reconnected/tracks_reconnected.csv": "6ecb2083ee0b9e006295499b1fc05fb8",
    "verify_out/events/rusia/reconnected/track_events.csv": "5e86f8cac3197ee1f3668fa830258771",
    "verify_out/events/rusia/reconnected/matches.csv": "198864d0641e3b52454c221797ccd98d",
    "verify_out/events/rusia/matches.csv": "abf7f5b5c7e7d46ec76d8668c02634c5",
    # omissions/window_sweep sealed after the ev22 GT correction (t_entrada 49.792→51.008),
    # derived over matching_core; verified byte-identical when re-derived.
    "verify_out/events/rusia/omissions.csv": "f9bbbf6d38d7f35edd6a3a5263689ad8",
    "verify_out/events/rusia/window_sweep.csv": "21b6510a88b88f17c567352242b638d0",
    # --- current outputs of the measurement instrument (evaluate_shadow_fidelity) ---
    "verify_out/events/rusia/shadow_gt_matches.csv": "8999e3b642fc34e820fd8079c1368664",
    "verify_out/events/rusia/shadow_gt_losses.csv": "56fa6cc12bafc2472707c314015f3aa7",
    # --- CORRECTED ground truth (exit_lane ×6; ev59 entry_lane 2→3; ev22 t_entrada; + track_events per_lane) ---
    "data/ground_truth/rusia/ground_truth_rusia_v2_carril_processed.csv": "1fa1bce5f8ded807050bc9881e86095a",
    "data/ground_truth/rusia/ground_russia_tnew.csv": "25c030452b7994db6f55abc2f7231415",
    # --- site calibration / configuration ---
    "config/detector.yaml": "9c6b81d2257cbaaff6ea245baece4d8f",
    "config/tracker.yaml": "360e8e69a89e7e3d29266e1bf1905f89",
    "config/positioning.yaml": "cffea0cdc3ac5f0725d54c84b986d3a9",
    "config/sites/rusia/site.yaml": "c4de4e0d35082ffa82c53273cdd20a20",
    "config/sites/rusia/lane_turn_rule.yaml": "4057ffc46d7313c00a7f22370219778f",  # re-derived (sur per_lane + oeste per_lane) by derive_lane_turn_rule.py
    "config/sites/rusia/turns.csv": "ffd054fc24d2ea5d48274dbce5936f99",
    "verify_out/homography_gcp/homography_img2ground.npy": "86adc1d06fb88459f4ce2ea341efc3cc",  # C 15 pts
    "verify_out/homography_gcp/control_points.csv": "7276ae6d779ecc06b8d85d1854e3c0ca",  # 15 official
    "verify_out/homography_improved/clean_frame_2541.png": "6696dffd1ffba604b966244d8b50b06f",  # frame ref
    "config/sites/rusia/sumo/intersection_rusia.net.xml": "90cc6b86f5f0323f950549f92343ee2b",  # +:J5_20 (v117)
    "config/sites/rusia/sumo/lane_map.yaml": "e78d9a79d4bd42d0b578c780c02064d7",  # net_md5 updated
    "config/sites/rusia/zones.yaml": "5c58014549604aec2288d5ecbe5df7ec",
    "config/sites/rusia/approach_zones.yaml": "a3041d3d412d4de5fca464365c1f9b95",  # west band calibrated (c2 123.4 / c3 95.3; c1 intact)
    # --- production code (context stamp: the figure came out of THIS pipeline) ---
    "digital_shadow/sumo/reconstruct.py": "3ec52992b60cf90c730293ab4c324e82",
    "digital_shadow/perception/detector.py": "668877b131415ab4b90877f5c9dd05ca",
    "scripts/run_harness_viz.py": "8054922a583b9805fa71047492cbb260",  # width per class in VTYPE_DEF (cosmetic, byte-identical readout)
    "scripts/derive_track_events.py": "94cc358e55539614bb926bbf385f1269",
    "scripts/measure_fragmentation_baseline.py": "cbd360261e58b4f3e3aeeb228531d9ab",
}


def md5_of(relpath: str) -> str:
    h = hashlib.md5()
    with open(ROOT / relpath, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def check_provenance(relpaths: list[str]) -> dict[str, str]:
    """Verifies that each input matches its pinned md5. Aborts if anything does not add up.

    Returns {relpath: md5} to stamp into the figure's record. An input with pin None
    is only recorded (it exists but has no historical md5 to verify).
    """
    stamped, bad = {}, []
    for rp in relpaths:
        if rp not in PINNED_MD5:
            bad.append(f"  {rp}: NO PINEADO — añadirlo a PINNED_MD5 a conciencia, no generar a ciegas")
            continue
        actual = md5_of(rp)
        expected = PINNED_MD5[rp]
        if expected is not None and actual != expected:
            bad.append(f"  {rp}: md5 {actual} != pineado {expected} (¿datos viejos o producción mutada?)")
        stamped[rp] = actual
    if bad:
        raise RuntimeError(
            "GUARDIA DE PROCEDENCIA: la generación se ABORTA. Insumos que no cuadran con el "
            "estado de producción:\n" + "\n".join(bad)
        )
    return stamped


# --------------------------------------------------------------------------- #
# 2) Paper editorial style — lives in style.py (single source shared with
#    figuras_resultados). Here the same names are RE-EXPORTED with the same
#    values. FULL_WIDTH_MM keeps the LEGACY value (181.6 = 7.15 in) of the
#    validated figures; the exact IEEE value (7.16 in) is used only by new figures.
#    LOCAL to this world (not shared): C_SERIES (A=perception/B=SUMO),
#    C_ORDINAL, C_NEUTRAL, hatch, PINNED_MD5, OUT_DIR/DATA_DIR.
# --------------------------------------------------------------------------- #
try:                                   # imported as a package (paper_figures.common)
    from paper_figures.style import (  # noqa: F401
        COLUMN_WIDTH_MM, FULL_WIDTH_MM_LEGACY, DPI_RASTER,
        C_INK, C_INK2, C_MUTED, C_GRID,
        apply_style, fig_size, save_fig as _style_save_fig,
    )
except ImportError:                    # imported flat (sys.path = this directory)
    from style import (  # noqa: F401
        COLUMN_WIDTH_MM, FULL_WIDTH_MM_LEGACY, DPI_RASTER,
        C_INK, C_INK2, C_MUTED, C_GRID,
        apply_style, fig_size, save_fig as _style_save_fig,
    )

FULL_WIDTH_MM = FULL_WIDTH_MM_LEGACY   # 181.6 — inherited, do NOT migrate (regression)

C_SERIES = {"A": "#2a78d6", "B": "#1baf7a"}     # A = perception (vision) · B = SUMO drawn
C_ORDINAL = {"hi": "#256abf", "mid": "#86b6ef"}  # blue ordinal ramp (full > partial)
C_NEUTRAL = "#c3c2b7"                            # absence/loss (neutral, not a series)


def save_fig(fig, name: str, inputs_md5: dict[str, str], script: str, nota: str = "") -> None:
    """Wrapper of style.save_fig with the directories and stamp of THIS world."""
    _style_save_fig(fig, name, inputs_md5, script, OUT_DIR, DATA_DIR, nota)


def git_untracked_note() -> str:
    """The project does not use git (permanent convention); this is recorded in the stamp."""
    return "repo sin git por convención del proyecto; la trazabilidad es por md5 pineado"


def ensure_dirs() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    DATA_DIR.mkdir(parents=True, exist_ok=True)
