"""PRESENTATION smoothing (drawing layer), separate from the MEASURED shadow.

PRINCIPLE: this filter touches ONLY the position sent to moveToXY for the demo SCREENSHOT.
The fidelity shadow (Section 7) is measured in SEPARATE loops that use the RAW recon position
(getPosition after step). render() emits no metric (it does not call getPosition nor write CSV), so
smoothing here cannot alter any fidelity number. Two paths from the same recon: measured (raw) / drawn
(smoothed). Mechanical proof: `draw_xy` is only created inside render(); the `veh` of run_core (source of
the metrics) does not contain it.

PROBLEM: the detector box of a stationary vehicle flickers → the ground contact oscillates (back and
forth, ~6–14 Hz). After the homography and the recon, the drawn trail vibrates ~5–14 px in the image
(worse in the far field).

FILTER: CAUSAL dead-band with anchor RE-CENTERED by moving median (suitable for live video: it uses only
the past, without accumulated lag, without needing the future — unlike a zero-phase filter).
- If the new position is < X px (image) from the current CENTER → it is FROZEN; the center is re-estimated
  as the MOVING MEDIAN of the recon positions of the frozen segment over a past window
  (`median_window_s`). Since the jitter is symmetric, the median is the REAL CENTER → the stopped vehicle
  is drawn centered, NOT at the entry point into the dead-band (avoids the ≤X offset of the fixed anchor).
  The bounded window makes the median STABLE (it does not chase every oscillation) and, in a slow creep,
  FOLLOW the advance (without accumulated offset).
- If the new position jumps outside X → the vehicle started moving: the center jumps to the new position
  and the segment (the window) is RESET.
The X threshold is in IMAGE px (perceptual uniformity despite the perspective); the distance is evaluated
by projecting to the image with the homography.

KNOWN LIMIT (there is no X that separates cleanly, see the sweep): a creep (a crawling queue) faster than
~X/median_window_s still stair-steps. The X trade-off (config: presentation.smooth_deadband_px) balances
killing the jitter of the stopped one vs not stair-stepping the slow movement.

I/O CONTRACT:
  presentation_smooth(recon_xy (N,2) world, recon_t (N,) s, H (3,3) image→world, deadband_px, median_window_s)
    -> draw_xy (N,2) world to draw. Does not mutate recon_xy. Causal: draw_xy[i] depends only on recon_xy[:i+1].
  load_presentation(site_id) -> (deadband_px, median_window_s) from the site config.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml


def load_presentation(site_id: str) -> tuple[float, float]:
    """(deadband_px, median_window_s) from zones.yaml → presentation."""
    p = Path(__file__).resolve().parents[2] / "config" / "sites" / site_id / "zones.yaml"
    with open(p, "r", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)["presentation"]
    return float(cfg["smooth_deadband_px"]), float(cfg["smooth_median_window_s"])


def load_handoff_max_hold(site_id: str) -> float:
    """PRESENTATION bound (s) of the vision shadow's handoff marker, from
    approach_zones.yaml → presentation.handoff_max_hold_s (a file NOT pinned, separate
    from zones.yaml, with its own life cycle for the queue layer). A presentation cap,
    NOT prediction: the log NEVER queries delay/cross_t/inst_t (see the yaml)."""
    p = (Path(__file__).resolve().parents[2] / "config" / "sites" / site_id
         / "approach_zones.yaml")
    with open(p, "r", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)["presentation"]
    return float(cfg["handoff_max_hold_s"])


def presentation_smooth(recon_xy: np.ndarray, recon_t: np.ndarray, H: np.ndarray,
                        deadband_px: float, median_window_s: float) -> np.ndarray:
    """Causal dead-band with anchor re-centered by moving median; threshold in image px. See module."""
    recon_xy = np.asarray(recon_xy, float)
    recon_t = np.asarray(recon_t, float)
    if len(recon_xy) == 0:
        return recon_xy.copy()
    Hinv = np.linalg.inv(H)

    def to_img(w):
        q = Hinv @ np.array([w[0], w[1], 1.0])
        return q[:2] / q[2]

    draw = recon_xy.copy()
    seg_t: list[float] = []          # times of the positions of the frozen segment (past window)
    seg_p: list[np.ndarray] = []     # recon positions of the frozen segment
    center = None
    center_img = None
    for i in range(len(recon_xy)):
        t = recon_t[i]
        p = recon_xy[i]
        p_img = to_img(p)
        if center is not None and np.hypot(p_img[0] - center_img[0], p_img[1] - center_img[1]) < deadband_px:
            # FROZEN: add the position and re-estimate the center as the median of the recent window
            seg_t.append(t)
            seg_p.append(p)
            while seg_t and t - seg_t[0] > median_window_s:   # prune the tail older than the window
                seg_t.pop(0)
                seg_p.pop(0)
            center = np.median(np.asarray(seg_p), axis=0)     # moving median = real center (symmetric jitter)
            center_img = to_img(center)
        else:
            # STARTED MOVING (left the dead-band): jump and reset the segment
            seg_t = [t]
            seg_p = [p.copy()]
            center = p.copy()
            center_img = p_img
        draw[i] = center
    return draw
