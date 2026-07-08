"""F11 — Waiting-boxes layer over the causal demo: still of the VIDEO | SHADOW IN SUMO split.

REPRESENTATIVE INSTANT of the system in operation, NOT a typical count: t=146.0 s is
deliberately the RICHEST frame of the tramo — queues in the four approaches, boxes anchored to
the lane by class, and the causal bus (12×2.5, real vType)
crossing the west stop line just born from its box (the box→vehicle seam visible).
Presenting it as a normal state would be misleading: it is an illustrative example of the operation.

STATEMENTS (they also go in the README and the stamp):
  - The waiting boxes are causal PRESENTATION, not measurement: nothing drawn feeds
    a metric; the command's --readout mode never draws them.
  - The shadow's occupancy is MEASURED in the vision/zones layer against the occupancy GT
    (measure_occupancy_accuracy, only the labeled segment) — NOT by lane membership
    in SUMO.

Source: verify_out/demo_causal/demo_causal_tramo.mp4 (render of the permanent command with
--waiting-boxes ON by default; integrated FULL PROBE PASS; its complete provenance lives in
verify_out/demo_causal/provenance_demo_causal_tramo.txt and is re-stamped here by md5).

LANGUAGE: the paper is in ENGLISH. The source mp4 keeps its banners in
Spanish (it will be re-recorded later with the 3D — it is not touched); this extractor REPAINTS the HUD
strip and redraws the labels in English ("VIDEO", "SHADOW IN SUMO" and the translated lag
line) with the same typography/positions as the runner's hud() (layout read from the runner
itself: single source). The line's VALUES are a 1:1 transcription of those burned into the
source frame (a measured datum from the render; only the text is translated).
"""
from __future__ import annotations

import hashlib
import re
import sys
from pathlib import Path

import cv2
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from paper_figures import common  # noqa: E402

INPUTS = [
    "config/sites/rusia/approach_zones.yaml",     # geometry/operating point of the drawn layer
]
MP4 = ROOT / "verify_out/demo_causal/demo_causal_tramo.mp4"
T_FRAME = 146.0
DT = 12.0 / 355.0
# HUD line in English: values = 1:1 transcription of those burned into the source frame
# (measured by the render; see docstring). If T_FRAME changes, update the transcription.
HUD_LINE_EN = "t_video = 146.03 s    in scene:  3    instantiation lag: median 0.11 s  [0.11-0.12 s]"


def _layout_from_runner():
    """VID_W and HUD_H from the runner itself (single source of the split layout)."""
    src = (ROOT / "scripts/demo_causal/run_harness_causal.py").read_text()
    m = re.search(r"^VID_W, VID_H, SUMO_W, HUD_H = (\d+), (\d+), (\d+), (\d+)", src, re.M)
    return int(m.group(1)), int(m.group(4))


def repaint_hud_english(frame):
    """Repaints the HUD strip (banners) in English over the extracted BGR frame, with the
    typography/positions of the runner's hud(). The visual content under the strip is not touched."""
    vid_w, hud_h = _layout_from_runner()
    cv2.rectangle(frame, (0, 0), (frame.shape[1], hud_h), (12, 12, 12), -1)
    cv2.putText(frame, "VIDEO", (14, 26), cv2.FONT_HERSHEY_SIMPLEX,
                0.66, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(frame, "SHADOW IN SUMO", (vid_w + 14, 26), cv2.FONT_HERSHEY_SIMPLEX,
                0.66, (120, 255, 120), 2, cv2.LINE_AA)
    cv2.putText(frame, HUD_LINE_EN, (14, 52), cv2.FONT_HERSHEY_SIMPLEX,
                0.52, (170, 255, 170), 1, cv2.LINE_AA)
    return frame


def main() -> None:
    stamp = common.check_provenance(INPUTS)
    common.apply_style()

    stamp["verify_out/demo_causal/demo_causal_tramo.mp4"] = \
        hashlib.md5(MP4.read_bytes()).hexdigest()
    cap = cv2.VideoCapture(str(MP4))
    fi = int(round(T_FRAME / DT))
    cap.set(cv2.CAP_PROP_POS_FRAMES, fi)
    ok, frame = cap.read()
    cap.release()
    assert ok, f"frame {fi} ilegible en {MP4.name}"
    frame = repaint_hud_english(frame)
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    h, w = img.shape[:2]
    fig, ax = plt.subplots(figsize=common.fig_size(common.FULL_WIDTH_MM, aspect=h / w))
    ax.imshow(img, extent=(0, w, h, 0))
    ax.set_xlim(0, w)
    ax.set_ylim(h, 0)
    ax.axis("off")

    common.save_fig(fig, "fig11_cajas_espera", stamp, "fig11_cajas_espera.py",
                    nota=f"still del split a t={T_FRAME:.1f} s (frame {fi}) — INSTANTE "
                         "REPRESENTATIVO en operación, NO conteo típico (el frame más rico: "
                         "colas en 4 brazos + costura caja→bus causal). Cajas = PRESENTACIÓN "
                         "causal, no medición; la ocupación se mide en visión/zonas contra el "
                         "GT del tramo (measure_occupancy_accuracy), no por carril en SUMO. "
                         "Sonda TOTAL PASS integrada en el render fuente.")


if __name__ == "__main__":
    main()
