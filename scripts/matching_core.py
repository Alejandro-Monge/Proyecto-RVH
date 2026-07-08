"""matching_core — PURE core of the shadow↔GT matching (protocol Section 6).

SINGLE executable specification of the matching, character by character, for BOTH
shadows (vision and messages), under the single-source doctrine. The id column
names are generic: vision uses track_id/event_id; messages, its
own ids.

Event contract (both sides, DataFrames):
  entry_arm (str), entry_time_s (float), exit_time_s (float), exit_arm (str),
  plus one id column per side (sh_id_col / gt_id_col, names DIFFERENT from each other).

Criterion (Section 6.2): edge valid only if SAME entry approach and |Δt_entry| <=
window; lexicographic cost — the entry dominates in DELTA_S δ-bands and the exit
breaks the tie within a band; one-to-one minimum-cost assignment (Hungarian) per approach
(block-diagonal). The window is provided by the CALLER (each shadow fixes it with its own
sensitivity sweep, Section 6.3, over its own clock).

No I/O, no config, no pins, no dependence on any world.
"""
from __future__ import annotations

import numpy as np
import pandas as pd
from scipy.optimize import linear_sum_assignment

DELTA_S = 0.1          # "entry tie" band = frame temporal resolution (~3 f at 29.6 fps)
K_EXIT_ARM = 1_000_000.0   # penalty for a different exit approach (> max |Δt_exit|, ~180 s)
BIG = 10_000_000.0     # weight of the entry band (> max exit_term) → the entry dominates
INVALID = 1e15         # forbidden edge (different approach or out of window)


def lexi_cost(dt_entry: float, dt_exit: float, exit_match: bool, window: float) -> float:
    """Lexicographic cost: entry (in DELTA bands) dominates; exit breaks the tie within a band."""
    if dt_entry > window:
        return INVALID
    exit_term = (0.0 if exit_match else K_EXIT_ARM) + dt_exit
    return np.floor(dt_entry / DELTA_S) * BIG + exit_term


def match_arm(sh: pd.DataFrame, gt: pd.DataFrame, window: float,
              sh_id_col: str = "track_id", gt_id_col: str = "event_id"):
    """Matches the events of ONE entry approach. Returns (pares, free_sh_idx, free_gt_idx).

    `pares`: list of dicts with the pair's indices and metrics. Indices are positional in sh/gt.
    """
    if len(sh) == 0 or len(gt) == 0:
        return [], list(range(len(sh))), list(range(len(gt)))
    cost = np.full((len(sh), len(gt)), INVALID)
    for i, (_, s) in enumerate(sh.iterrows()):
        for j, (_, g) in enumerate(gt.iterrows()):
            dt_e = abs(s.entry_time_s - g.entry_time_s)
            dt_x = abs(s.exit_time_s - g.exit_time_s)
            cost[i, j] = lexi_cost(dt_e, dt_x, s.exit_arm == g.exit_arm, window)
    ri, cj = linear_sum_assignment(cost)
    pares, used_s, used_g = [], set(), set()
    for i, j in zip(ri, cj):
        dt_e = abs(sh.iloc[i].entry_time_s - gt.iloc[j].entry_time_s)
        if dt_e > window:        # filler assignment (there was no valid edge) → discarded
            continue
        used_s.add(i); used_g.add(j)
        pares.append({
            "sh_i": i, "gt_j": j,
            sh_id_col: sh.iloc[i][sh_id_col], gt_id_col: gt.iloc[j][gt_id_col],
            "entry_arm": sh.iloc[i].entry_arm,
            "dt_entry": sh.iloc[i].entry_time_s - gt.iloc[j].entry_time_s,   # SIGNED (shadow−GT)
            "sh_exit": sh.iloc[i].exit_arm, "gt_exit": gt.iloc[j].exit_arm,
            "exit_correct": bool(sh.iloc[i].exit_arm == gt.iloc[j].exit_arm),
            "dt_exit": sh.iloc[i].exit_time_s - gt.iloc[j].exit_time_s,
        })
    free_s = [i for i in range(len(sh)) if i not in used_s]
    free_g = [j for j in range(len(gt)) if j not in used_g]
    return pares, free_s, free_g


def run_matching(shadow: pd.DataFrame, gt: pd.DataFrame, arms, window: float,
                 sh_id_col: str = "track_id", gt_id_col: str = "event_id"):
    """Matches per approach and aggregates. Returns (pares, ghosts_df, omissions_df, contested)."""
    pares, fant, omis = [], [], []
    contested = 0  # shadow events with >=2 candidate GT in window (plateau metric #3)
    for a in arms:
        sh_a = shadow[shadow.entry_arm == a].reset_index(drop=True)
        gt_a = gt[gt.entry_arm == a].reset_index(drop=True)
        for _, s in sh_a.iterrows():
            cands = (gt_a.entry_time_s - s.entry_time_s).abs() <= window
            if cands.sum() >= 2:
                contested += 1
        p, fs, fg = match_arm(sh_a, gt_a, window, sh_id_col, gt_id_col)
        pares.extend(p)
        fant.extend(sh_a.iloc[fs].assign(entry_arm=a).to_dict("records"))
        omis.extend(gt_a.iloc[fg].assign(entry_arm=a).to_dict("records"))
    return pd.DataFrame(pares), pd.DataFrame(fant), pd.DataFrame(omis), contested


def audit_ties(shadow: pd.DataFrame, gt: pd.DataFrame, arms, window: float) -> int:
    """Counts REAL ties: shadow events with >=2 candidate GT in the SAME δ-band."""
    n = 0
    for a in arms:
        sh_a = shadow[shadow.entry_arm == a]
        gt_a = gt[gt.entry_arm == a]
        for _, s in sh_a.iterrows():
            dts = (gt_a.entry_time_s - s.entry_time_s).abs()
            in_win = dts[dts <= window]
            if len(in_win) < 2:
                continue
            bands = np.floor(in_win.to_numpy() / DELTA_S)
            _, counts = np.unique(bands, return_counts=True)
            if (counts >= 2).any():
                n += 1
    return n


def per_arm_report(pares, fant, omis, shadow, gt, arms) -> pd.DataFrame:
    rows = []
    for a in list(arms) + ["TOTAL"]:
        if a == "TOTAL":
            n_gt, n_sh = len(gt), len(shadow)
            m = len(pares)
            f = len(fant); o = len(omis)
            exitok = pares.exit_correct.sum() if len(pares) else 0
        else:
            n_gt = (gt.entry_arm == a).sum()
            n_sh = (shadow.entry_arm == a).sum()
            m = (pares.entry_arm == a).sum() if len(pares) else 0
            f = (fant.entry_arm == a).sum() if len(fant) else 0
            o = (omis.entry_arm == a).sum() if len(omis) else 0
            exitok = pares[pares.entry_arm == a].exit_correct.sum() if len(pares) else 0
        rows.append(dict(
            brazo_entrada=a, gt=n_gt, sombra=n_sh, casados=m, fantasmas=f, omisiones=o,
            recall=round(m / n_gt, 3) if n_gt else float("nan"),
            precision=round(m / n_sh, 3) if n_sh else float("nan"),
            salida_ok=int(exitok),
            acierto_salida=round(exitok / m, 3) if m else float("nan"),
        ))
    return pd.DataFrame(rows)
