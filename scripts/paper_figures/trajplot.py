"""Shared helpers of the calibration/perception block (F1–F5) — figures layer, read-only.

Centralizes what the five figures share so that none re-implements it differently:
reference frame, OFFICIAL control points (the 15 from homography_gcp, the current
C-15pts calibration; NEVER control_points_improved), stop lines from zones.yaml, production
exclusion rule (excluded = tracks with no event in the corresponding track_events,
the same one run_core uses), stable per-track color and network lanes for the world plane.
"""
from __future__ import annotations

import csv
import re
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import yaml

ROOT = Path(__file__).resolve().parents[2]

FRAME = ROOT / "verify_out/homography_improved/clean_frame_2541.png"
CPTS_OFICIAL = ROOT / "verify_out/homography_gcp/control_points.csv"   # the 15 (calibration C)
NET = ROOT / "config/sites/rusia/sumo/intersection_rusia.net.xml"
ZONES = ROOT / "config/sites/rusia/zones.yaml"
H_NPY = ROOT / "verify_out/homography_gcp/homography_img2ground.npy"

MIN_OBS = 25            # same filter as the project's analysis scripts (declared in README)
ARM_EDGES = {"E1", "-E1", "E3", "-E3", "E5", "-E5", "E6", "-E6", "E0", "-E0"}

C_CPT = "#eda100"       # control points (yellow, black border: legible over asphalt)
C_STOP = "#2a78d6"      # stop lines
C_NET = "#b9b8b0"       # network lanes (recessive)


def load_frame_rgb() -> np.ndarray:
    return cv2.cvtColor(cv2.imread(str(FRAME)), cv2.COLOR_BGR2RGB)


def load_cpts() -> pd.DataFrame:
    """Available OFFICIAL control points (image px + world)."""
    df = pd.read_csv(CPTS_OFICIAL)
    return df[df.available.astype(str) == "True"].reset_index(drop=True)


def load_stop_lines() -> dict[str, np.ndarray]:
    z = yaml.safe_load(ZONES.read_text())
    return {arm: np.array(seg, float) for arm, seg in z["stop_lines"].items()}


def load_tracks(tracks_csv: str, events_csv: str) -> pd.DataFrame:
    """Tracks with ground contact, WITHOUT those excluded by zone (run_core production rule:
    excluded = track with no event in the corresponding track_events) and with ≥ MIN_OBS obs."""
    from digital_shadow.geometry.ground_contact import add_ground_contact
    df = pd.read_csv(ROOT / tracks_csv)
    add_ground_contact(df)
    events = pd.read_csv(ROOT / events_csv)
    df = df[df.track_id.isin(set(events.track_id))]
    keep = df.groupby("track_id").size()
    return df[df.track_id.isin(keep[keep >= MIN_OBS].index)]


def track_color(k: int):
    """Stable color per track index (tab20). It only distinguishes neighboring trajectories; it does
    not encode a nameable identity (no legend) — same criterion as the control views."""
    import matplotlib.pyplot as plt
    return plt.get_cmap("tab20")(k % 20)


def draw_tracks_camera(ax, df: pd.DataFrame, lw: float = 0.7, alpha: float = 0.8) -> int:
    n = 0
    for k, (_tid, g) in enumerate(df.groupby("track_id")):
        g = g.sort_values("frame_index")
        ax.plot(g.gx, g.gy, color=track_color(k), lw=lw, alpha=alpha, zorder=2,
                solid_capstyle="round")
        n += 1
    return n


def project_world(H: np.ndarray, gx, gy) -> np.ndarray:
    p = np.stack([np.asarray(gx, float), np.asarray(gy, float), np.ones(len(gx))])
    w = H @ p
    return (w[:2] / w[2]).T


def draw_tracks_world(ax, df: pd.DataFrame, H: np.ndarray, lw: float = 0.7,
                      alpha: float = 0.8) -> int:
    n = 0
    for k, (_tid, g) in enumerate(df.groupby("track_id")):
        g = g.sort_values("frame_index")
        w = project_world(H, g.gx.to_numpy(), g.gy.to_numpy())
        ax.plot(w[:, 0], w[:, 1], color=track_color(k), lw=lw, alpha=alpha, zorder=2,
                solid_capstyle="round")
        n += 1
    return n


def net_lane_shapes() -> list[np.ndarray]:
    """Polylines of the lanes of the network APPROACHES (replica of the
    plot_birdseye_hires.net_lane_shapes parser, with an absolute path to not depend on the cwd)."""
    txt = NET.read_text()
    shapes = []
    for m in re.finditer(r'<edge id="([^"]+)"[^>]*>(.*?)</edge>', txt, re.S):
        eid, body = m.group(1), m.group(2)
        if eid not in ARM_EDGES:
            continue
        for lm in re.finditer(r'shape="([^"]+)"', body):
            shapes.append(np.array([tuple(map(float, q.split(","))) for q in lm.group(1).split()]))
    return shapes


def world_view_limits(shapes: list[np.ndarray], margin: float = 8.0):
    allp = np.vstack(shapes)
    return ((allp[:, 0].min() - margin, allp[:, 0].max() + margin),
            (allp[:, 1].min() - margin, allp[:, 1].max() + margin))


def add_scale_bar(ax, xlim, ylim, length_m: float = 20.0):
    """Scale bar (map with no axes): lower-left corner, in meters."""
    x0 = xlim[0] + 0.05 * (xlim[1] - xlim[0])
    y0 = ylim[0] + 0.06 * (ylim[1] - ylim[0])
    ax.plot([x0, x0 + length_m], [y0, y0], color="#0b0b0b", lw=1.2, solid_capstyle="butt")
    ax.text(x0 + length_m / 2, y0 + 0.012 * (ylim[1] - ylim[0]), f"{length_m:.0f} m",
            ha="center", va="bottom", fontsize=6, color="#0b0b0b")
