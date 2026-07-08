"""occupancy_compare — shadow↔GT occupancy series comparator (Section 7.7, option (c)).

SOURCE-AGNOSTIC: consumes two DataFrames with schema
(sim_time_s, brazo, carril, ocupacion, detenidos) and produces the fidelity
metrics with a lag sweep. The SAME implementation serves messages
(lag* = channel delay) and vision (lag* = reconstruction delay).
No path I/O, no world config, no state.

Fixed rules:
  - Grid GATING BEFORE any MAE (verify_grid): single equal step,
    same origin phase, same keys per tick, contiguous intersection. What is
    trimmed is REPORTED (grid_report), never silently truncated.
  - Sign convention: shadow(t) is compared against GT(t − lag), lag ≥ 0 =
    the shadow lags behind.
  - lag* is estimated ONLY at the APPROACH level (aggregated counts, stable argmin);
    the lanes INHERIT the lag* of their approach — never a per-lane argmin
    (counts 0-5 = optimized noise).
  - Always the triple (MAE@0, lag*, MAE@lag*) + the full curve; the core has
    no "optimal only" mode. delta = MAE@0 − MAE@lag* separates temporal offset
    from count error.
  - Multi-rep aggregation (MC sweep, downstream): MEDIAN curve first and
    lag* from the argmin of that aggregated curve; NEVER average lag* per-rep.
Deterministic: no iterated sets, outputs ordered by keys.
"""
from __future__ import annotations

import numpy as np
import pandas as pd

STEP = 0.1
LAGS_DEFAULT = np.round(np.arange(0.0, 3.0001, 0.1), 1)


class GridMismatch(RuntimeError):
    """The grid verification failed: comparing would measure misalignment, not error."""


def _ticks(df) -> np.ndarray:
    return np.round(np.sort(df.sim_time_s.unique()), 1)


def verify_grid(sombra: pd.DataFrame, gt: pd.DataFrame, step: float = STEP) -> dict:
    """Gating A: shared grid and common origin, or GridMismatch. Returns the
    grid_report with what is trimmed (edges outside the intersection)."""
    fails = []
    ts, tg = _ticks(sombra), _ticks(gt)
    for nombre, t in [("sombra", ts), ("gt", tg)]:
        d = set(np.round(np.diff(t), 6))
        if d != {round(step, 6)}:
            fails.append(f"{nombre}: paso no uniforme {sorted(d)[:4]} (esperado {step})")
        fase = set(np.round((t / step) % 1, 6)) - {0.0, 1.0}
        if fase:
            fails.append(f"{nombre}: fase de origen desalineada de la rejilla {sorted(fase)[:4]}")
    ks = sombra.groupby(["brazo", "carril"]).ngroups
    kg = gt.groupby(["brazo", "carril"]).ngroups
    keys_s = set(map(tuple, sombra[["brazo", "carril"]].drop_duplicates().itertuples(index=False)))
    keys_g = set(map(tuple, gt[["brazo", "carril"]].drop_duplicates().itertuples(index=False)))
    if keys_s != keys_g:
        fails.append(f"claves brazo-carril distintas: solo_sombra={sorted(keys_s-keys_g)}, "
                     f"solo_gt={sorted(keys_g-keys_s)}")
    comun = np.round(np.array(sorted(set(ts) & set(tg))), 1)
    if len(comun) == 0:
        fails.append("intersección temporal vacía")
    else:
        huecos = int((np.round(np.diff(comun), 1) != round(step, 1)).sum())
        if huecos:
            fails.append(f"intersección con {huecos} huecos internos")
    if fails:
        raise GridMismatch("verify_grid FALLÓ:\n  - " + "\n  - ".join(fails))
    solo_g = sorted(set(tg) - set(ts))
    solo_s = sorted(set(ts) - set(tg))
    return dict(
        step=step, claves_por_tick=ks,
        ticks_comunes=len(comun), rango_comun=[float(comun[0]), float(comun[-1])],
        recorte_solo_gt=dict(n=len(solo_g),
                             rango=[float(solo_g[0]), float(solo_g[-1])] if solo_g else None),
        recorte_solo_sombra=dict(n=len(solo_s),
                                 rango=[float(solo_s[0]), float(solo_s[-1])] if solo_s else None),
        nota_recorte="los ticks fuera de la intersección se EXCLUYEN y se declaran aquí; "
                     "no hay truncado silencioso",
    )


def _arm_matrix(df: pd.DataFrame, metrica: str) -> pd.DataFrame:
    """(tick × approach) with the metric aggregated per approach (sum of lanes)."""
    g = df.groupby(["sim_time_s", "brazo"])[metrica].sum().unstack("brazo")
    g.index = np.round(g.index, 1)
    return g.sort_index()


def _lane_matrix(df: pd.DataFrame, metrica: str) -> pd.DataFrame:
    g = df.set_index(["sim_time_s", "brazo", "carril"])[metrica].unstack(["brazo", "carril"])
    g.index = np.round(g.index, 1)
    return g.sort_index().sort_index(axis=1)


def _mae_curve(sh: pd.DataFrame, gt: pd.DataFrame, comun: np.ndarray,
               lags: np.ndarray) -> pd.DataFrame:
    """MAE(lag) per column: shadow(t) vs GT(t−lag), over the t of the
    intersection whose t−lag exists in the GT. Reports n per lag (no silence)."""
    rows = []
    gt_idx = set(np.round(gt.index, 1))
    for lag in lags:
        t_ok = np.array([t for t in comun if round(t - lag, 1) in gt_idx])
        if not len(t_ok):
            continue
        a = sh.loc[t_ok]
        b = gt.loc[np.round(t_ok - lag, 1)]
        b.index = a.index
        d = (a - b).abs()
        for col in sh.columns:
            rows.append(dict(clave=col if isinstance(col, str) else "/".join(map(str, col)),
                             lag_s=float(lag), n_ticks=len(t_ok),
                             mae=float(d[col].mean()),
                             exact=float((d[col] == 0).mean())))
    return pd.DataFrame(rows)


def compare_series(sombra: pd.DataFrame, gt: pd.DataFrame,
                   lags: np.ndarray = LAGS_DEFAULT,
                   metricas=("ocupacion", "detenidos")) -> dict:
    rep = verify_grid(sombra, gt)
    comun = np.round(np.array(sorted(set(_ticks(sombra)) & set(_ticks(gt)))), 1)

    aligned_parts, mae_parts, summary_rows = [], [], []
    for met in metricas:
        sh_arm, gt_arm = _arm_matrix(sombra, met), _arm_matrix(gt, met)
        sh_ln, gt_ln = _lane_matrix(sombra, met), _lane_matrix(gt, met)

        # aligned series (lag 0) — figure input, lane level
        a = sombra.set_index(["sim_time_s", "brazo", "carril"])[met]
        b = gt.set_index(["sim_time_s", "brazo", "carril"])[met]
        al = pd.concat([b, a], axis=1, keys=["gt", "sombra"]).reset_index()
        al = al[al.sim_time_s.round(1).isin(set(comun))].assign(metrica=met)
        aligned_parts.append(al)

        curva_arm = _mae_curve(sh_arm.loc[comun], gt_arm, comun, lags).assign(
            metrica=met, nivel="brazo")
        mae_parts.append(curva_arm)

        # lag* PER APPROACH; lanes inherit
        for brazo in sh_arm.columns:
            c = curva_arm[curva_arm.clave == brazo].reset_index(drop=True)
            i0 = c[c.lag_s == 0.0].index[0]
            istar = int(c.mae.idxmin())
            lag_star = float(c.loc[istar, "lag_s"])
            summary_rows.append(dict(
                metrica=met, nivel="brazo", brazo=brazo, carril="",
                mae_0=round(float(c.loc[i0, "mae"]), 4),
                lag_star_s=lag_star,
                mae_lag_star=round(float(c.loc[istar, "mae"]), 4),
                delta=round(float(c.loc[i0, "mae"] - c.loc[istar, "mae"]), 4),
                exact_0=round(float(c.loc[i0, "exact"]), 4),
                exact_lag_star=round(float(c.loc[istar, "exact"]), 4)))
            for col in [k for k in sh_ln.columns if k[0] == brazo]:
                cl = _mae_curve(sh_ln.loc[comun, [col]], gt_ln[[col]], comun,
                                np.round(np.array([0.0, lag_star]), 1)).assign(metrica=met)
                c0 = cl[cl.lag_s == 0.0].iloc[0]
                cs = cl[cl.lag_s == lag_star].iloc[0]
                summary_rows.append(dict(
                    metrica=met, nivel="carril", brazo=brazo, carril=int(col[1]),
                    mae_0=round(float(c0.mae), 4), lag_star_s=lag_star,
                    mae_lag_star=round(float(cs.mae), 4),
                    delta=round(float(c0.mae - cs.mae), 4),
                    exact_0=round(float(c0.exact), 4),
                    exact_lag_star=round(float(cs.exact), 4)))

    aligned = pd.concat(aligned_parts).sort_values(
        ["metrica", "brazo", "carril", "sim_time_s"]).reset_index(drop=True)
    mae_lag = pd.concat(mae_parts).sort_values(
        ["metrica", "clave", "lag_s"]).reset_index(drop=True)
    summary = pd.DataFrame(summary_rows).sort_values(
        ["metrica", "brazo", "nivel", "carril"]).reset_index(drop=True)
    return dict(grid_report=rep, aligned=aligned, mae_lag=mae_lag, summary=summary)
