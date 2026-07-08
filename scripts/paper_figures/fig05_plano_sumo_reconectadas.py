"""F5 — RECONNECTED trajectories projected onto the SUMO plane (the "8" properly laid out).

PROVENANCE: **BATCH / OFFLINE — NOT what runs live** (batch-global reconnection,
offline upper bound; live production = causal-greedy, declared 10/15 gap). The homography does NOT
change with the reconnection (it is an image↔ground calibration): what changes are the TRAJECTORIES
that are projected. Same canvas, H, net, filters and color convention as F4.
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from paper_figures import fig04_plano_sumo  # noqa: E402

INPUTS = [
    "verify_out/events/rusia/reconnected/tracks_reconnected.csv",
    "verify_out/events/rusia/reconnected/track_events.csv",
    "verify_out/homography_gcp/homography_img2ground.npy",
    "verify_out/homography_gcp/control_points.csv",
    "config/sites/rusia/sumo/intersection_rusia.net.xml",
]

if __name__ == "__main__":
    fig04_plano_sumo.main(
        tracks_csv="verify_out/events/rusia/reconnected/tracks_reconnected.csv",
        events_csv="verify_out/events/rusia/reconnected/track_events.csv",
        inputs=INPUTS,
        name="fig05_plano_sumo_reconectadas",
        nota="BATCH/OFFLINE (techo; producción en vivo = causal-greedy); H oficial C-15pts",
    )
