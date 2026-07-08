"""style — SINGLE SOURCE of the editorial style of ALL the paper's figures
(implementation in imagenes_finales/ and results in figuras_resultados/).

Only what is genuinely common lives here:
  - IEEE editorial widths, typography (apply_style), fig_size, parameterized
    save_fig, DPI, neutral inks;
  - the paper SEMANTIC PAIR C_SOMBRA (vision/messages) + C_GT_REF + BAND_ALPHA:
    constant across ALL comparative figures.
LOCAL to each world (does not live here): provenance pins, implementation C_SERIES A/B
(perception/SUMO), ordinal ramps, output directories.

WIDTHS: single column 88.9 mm = 3.5 in EXACT (IEEE). Double column:
  - FULL_WIDTH_MM_IEEE = 181.864 (7.16 in exact) — for NEW figures (results).
  - FULL_WIDTH_MM_LEGACY = 181.6 (7.15 in, inherited) — implementation ONLY: the
    validated figures were generated this way and changing it would be a regression with no benefit.

SHADOW PALETTE (measured luminances):
  vision #2a78d6 (L=0.188, medium-dark gray) · messages #e69f00 (L=0.416, light
  gray; Okabe-Ito) · GT #0b0b0b (L=0.003, near black). ΔL vision↔messages=0.228:
  color AND robust in gray — a property, not a B/W restriction. Vermillion #d55e00
  discarded with data (L=0.222, ΔL=0.034 vs blue: indistinguishable in print).

PER-FIGURE REDUNDANCY, NOT HARDWIRED: color is the
primary discriminator and is ALWAYS present; each figure chooses ONE secondary
discriminator (line style OR marker), not both; full redundancy
(color+line+marker) is reserved for dense figures where color alone does not
separate, and the decision is documented IN THE INVENTORY ENTRY of that figure.
This module provides the palette; it does not impose line styles.

Percentile bands (p10–p90): fill of the SAME hue as its central curve,
alpha = BAND_ALPHA, no border; at most two overlapping bands per panel.
"""
from __future__ import annotations

from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]

# --------------------------------------------------------------------------- #
# Editorial widths (IEEE two columns, PE-WASUN 2026 confirmed)
# --------------------------------------------------------------------------- #
COLUMN_WIDTH_MM = 88.9              # single column = 3.5 in EXACT
FULL_WIDTH_MM_IEEE = 181.864        # double column = 7.16 in EXACT (new figures)
FULL_WIDTH_MM_LEGACY = 181.6        # 7.15 in inherited (implementation ONLY, do not migrate)
DPI_RASTER = 600                    # review PNG; the vector PDF is the primary one

# --------------------------------------------------------------------------- #
# Neutral inks (shared chrome)
# --------------------------------------------------------------------------- #
C_INK = "#0b0b0b"
C_INK2 = "#52514e"
C_MUTED = "#898781"
C_GRID = "#e1e0d9"

# --------------------------------------------------------------------------- #
# Paper semantic pair: the two shadows + GT (constant across every comparison)
# --------------------------------------------------------------------------- #
C_SOMBRA = {"vision": "#2a78d6", "mensajes": "#e69f00"}
C_GT_REF = C_INK
BAND_ALPHA = 0.18                   # p10-p90 band: same hue, no border


def apply_style() -> None:
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Nimbus Roman No9 L", "Nimbus Roman", "STIX Two Text",
                       "Times New Roman", "DejaVu Serif"],
        "mathtext.fontset": "stix",
        "font.size": 8,
        "axes.labelsize": 8,
        "axes.titlesize": 8,        # panel subtitles (a)/(b) — never a figure title
        "xtick.labelsize": 7,
        "ytick.labelsize": 7,
        "legend.fontsize": 7,
        "axes.linewidth": 0.6,
        "xtick.major.width": 0.6,
        "ytick.major.width": 0.6,
        "xtick.major.size": 2.5,
        "ytick.major.size": 2.5,
        "axes.edgecolor": C_INK2,
        "axes.labelcolor": C_INK,
        "xtick.color": C_INK2,
        "ytick.color": C_INK2,
        "axes.grid": False,
        "grid.color": C_GRID,
        "grid.linewidth": 0.5,
        "legend.frameon": False,
        "figure.dpi": 110,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.02,
        "pdf.fonttype": 42,         # embedded TrueType (usual editorial requirement)
        "ps.fonttype": 42,
    })


def fig_size(width_mm: float = COLUMN_WIDTH_MM, aspect: float = 0.62) -> tuple[float, float]:
    """(width, height) in inches from the editorial width in mm and a height/width ratio."""
    w_in = width_mm / 25.4
    return (w_in, w_in * aspect)


def save_fig(fig, name: str, inputs_md5: dict[str, str], script: str,
             out_dir: Path, data_dir: Path, nota: str = "",
             script_dir_label: str = "scripts/paper_figures",
             ancho_nota: str = f"{COLUMN_WIDTH_MM} mm (pendiente confirmar IEEE/ACM)") -> None:
    """Saves PDF (primary) + PNG 600 dpi (review) and stamps the provenance in datos/.

    The per-figure stamp makes it auditable WHICH exact production state generated it, without opening
    the script: date, generating script, inputs and their md5s verified by the guard.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    data_dir.mkdir(parents=True, exist_ok=True)
    pdf = out_dir / f"{name}.pdf"
    png = out_dir / f"{name}.png"
    fig.savefig(pdf)
    fig.savefig(png, dpi=DPI_RASTER)
    stamp = data_dir / f"provenance_{name}.txt"
    lines = [
        f"figura: {name}",
        f"generada: {datetime.now().isoformat(timespec='seconds')}",
        f"script: {script_dir_label}/{script}",
        f"ancho editorial: {ancho_nota}",
        "insumos verificados por la guardia de procedencia (md5):",
        *(f"  {p} = {m}" for p, m in sorted(inputs_md5.items())),
    ]
    if nota:
        lines.append(f"nota: {nota}")
    stamp.write_text("\n".join(lines) + "\n")
    print(f"[ok] {pdf.relative_to(ROOT)} + .png ({DPI_RASTER} dpi) · sello: {stamp.relative_to(ROOT)}")
