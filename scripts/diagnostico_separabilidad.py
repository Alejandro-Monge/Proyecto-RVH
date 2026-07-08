"""Lane separability DIAGNOSTIC — ALL approaches, hypothetical per_lane boundaries.

READ-ONLY: does not touch production or the classifier. It answers "do the lanes separate
everywhere, including south (currently blind) and west (grouped 2-3), and how much would it cost/gain to
classify per_lane?" with figures + raw numbers, n ALWAYS visible.

ANTI-SELF-DECEPTION FRAMEWORK: a high per_lane accuracy over n=3-4 is NOT evidence of
classifiability — those are 3 examples. Where it applies, it reports "geometrically separable with
n=X, insufficient sample to validate". The real validation of any promising separation
goes over the LONG video (more sample); this only says whether the idea HAS MERIT.

Method: same inputs and s coordinate as production/fig08 — RECONNECTED tracks (band
calibration source; chains identical to the causal ones) matched to the
CORRECTED GT; s = production's lane_cross_coord at the stop-line crossing. Per_lane accuracy by
LOO PER VEHICLE (per-lane median without the one under evaluation, nearest neighbor) — never per-frame.
Hypothetical boundaries = midpoint between per-lane GT medians (dotted gray); the real production
ones in dashed black where they exist.

Outputs: verify_out/diagnostico_separabilidad/ (6 PNG + separabilidad.csv + README.md).
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from paper_figures import common, trajplot  # noqa: E402
import derive_track_events as dte  # noqa: E402
from digital_shadow.config.site import load_site_config  # noqa: E402
from digital_shadow.geometry.ground_contact import add_ground_contact  # noqa: E402

OUT = ROOT / "verify_out" / "diagnostico_separabilidad"
C_LANE = {1: "#2a78d6", 2: "#1baf7a", 3: "#eda100"}
PANELS = [("entry", a) for a in ("este", "norte", "oeste", "sur")] + \
         [("exit", "este"), ("exit", "oeste")]
MARGIN = 165
INPUTS = [
    "verify_out/events/rusia/reconnected/tracks_reconnected.csv",
    "verify_out/events/rusia/reconnected/matches.csv",
    "data/ground_truth/rusia/ground_truth_rusia_v2_carril_processed.csv",
    "config/sites/rusia/zones.yaml",
    "scripts/derive_track_events.py",
]


def loo_accuracy(svals: dict[int, list[float]]):
    """LOO per vehicle: classifies each s by the per-lane median WITHOUT it. (ok, n)."""
    ok = n = 0
    for lane, vals in svals.items():
        for k, s in enumerate(vals):
            meds = {}
            for l2, v2 in svals.items():
                rest = [x for j, x in enumerate(v2) if not (l2 == lane and j == k)]
                if rest:
                    meds[l2] = float(np.median(rest))
            if not meds:
                continue
            pred = min(meds, key=lambda l2: abs(meds[l2] - s))
            ok += int(pred == lane)
            n += 1
    return ok, n


def current_accuracy(svals, band):
    """Accuracy of the CURRENT production state over the same sample."""
    if not band or band.get("resolution") == "arm_only":
        return None, sum(len(v) for v in svals.values())        # does not classify lane
    ok = n = 0
    for lane, vals in svals.items():
        for s in vals:
            lab = dte.classify_lane(s, band)
            if not lab:
                continue
            n += 1
            hit = (lane in [int(x) for x in lab.split("-")]) if "-" in lab else (lab.isdigit() and int(lab) == lane)
            ok += int(hit)
    return ok, n


def main() -> None:
    stamp = common.check_provenance(INPUTS)
    common.apply_style()
    OUT.mkdir(parents=True, exist_ok=True)

    site = load_site_config("rusia")
    center = np.array([site.frame_width / 2.0, site.frame_height / 2.0])
    stops = dte.load_stop_lines("rusia")
    bands = dte.load_lane_bands("rusia")
    img = trajplot.load_frame_rgb()
    h, w = img.shape[:2]
    tracks = pd.read_csv(ROOT / "verify_out/events/rusia/reconnected/tracks_reconnected.csv")
    add_ground_contact(tracks)
    matches = pd.read_csv(ROOT / "verify_out/events/rusia/reconnected/matches.csv")[["track_id", "event_id"]]
    gt = pd.read_csv(ROOT / "data/ground_truth/rusia/ground_truth_rusia_v2_carril_processed.csv")
    mg = matches.merge(gt, on="event_id")

    rows = []
    for kind, arm in PANELS:
        arm_col, lane_col = f"{kind}_arm", f"{kind}_lane"
        sel = mg[mg[arm_col] == arm]
        seg = stops[arm]
        p1 = seg[0]
        u = (seg[1] - seg[0]) / np.linalg.norm(seg[1] - seg[0])
        _p, nh = dte._inward_normal(seg, center)
        band = bands.get(kind, {}).get(arm)

        svals: dict[int, list] = {}
        trails = []
        for r in sel.itertuples():
            g = tracks[tracks.track_id == r.track_id].sort_values("frame_index")
            if not len(g):
                continue
            lane = int(getattr(r, lane_col))
            s = dte.lane_cross_coord(g, seg, center, kind=kind)
            trails.append((lane, g, s))
            if s is not None:
                svals.setdefault(lane, []).append(float(s))

        # ---- raw numbers ----
        meds = {l: float(np.median(v)) for l, v in sorted(svals.items())}
        stats = {l: dict(n=len(v), med=meds[l], sd=float(np.std(v)),
                         lo=float(np.min(v)), hi=float(np.max(v)))
                 for l, v in sorted(svals.items())}
        lanes_sorted = sorted(meds, key=lambda l: meds[l])
        gaps = []
        for a, b in zip(lanes_sorted, lanes_sorted[1:]):
            gap = abs(meds[b] - meds[a])
            solape_rango = not (stats[a]["hi"] < stats[b]["lo"] or stats[b]["hi"] < stats[a]["lo"])
            limpio = (not solape_rango) and gap > (stats[a]["sd"] + stats[b]["sd"])
            gaps.append((a, b, gap, limpio, solape_rango))
        ok_loo, n_loo = loo_accuracy(svals)
        ok_cur, n_cur = current_accuracy(svals, band)
        n_min = min((st["n"] for st in stats.values()), default=0)
        veredicto = ("separa limpio" if gaps and all(g[3] for g in gaps) else
                     "solapa" if gaps and any(g[4] for g in gaps) else "mixto")
        if n_min < 5 and veredicto == "separa limpio":
            veredicto += f" (n_min={n_min}: muestra INSUFICIENTE para validar)"
        for l, st in stats.items():
            rows.append(dict(marco=kind, brazo=arm, carril=l, **st))
        rows.append(dict(marco=kind, brazo=arm, carril="RESUMEN",
                         n=sum(st["n"] for st in stats.values()),
                         med=np.nan, sd=np.nan, lo=np.nan, hi=np.nan,
                         brechas="; ".join(f"{a}|{b}={g:.1f}px{'✓' if li else ('✗solape' if so else '~')}"
                                           for a, b, g, li, so in gaps),
                         loo_perlane=f"{ok_loo}/{n_loo}",
                         actual=("no clasifica (arm_only)" if ok_cur is None else f"{ok_cur}/{n_cur}"),
                         veredicto=veredicto))

        # ---- figure ----
        fig, ax = plt.subplots(figsize=common.fig_size(common.COLUMN_WIDTH_MM, aspect=0.72))
        ax.imshow(img, extent=(0, w, h, 0), zorder=0)
        for lane, g, s in trails:
            ax.plot(g.gx, g.gy, color=C_LANE[lane], lw=0.9, alpha=0.8, zorder=2)
            if s is not None:
                pt = p1 + s * u
                ax.plot(*pt, marker="o", ms=4.2, mfc=C_LANE[lane], mec="#0b0b0b", mew=0.5, zorder=4)
        ax.plot(seg[:, 0], seg[:, 1], color="#0b0b0b", lw=1.1, alpha=0.85, zorder=3)
        real_fr = []
        if band and band.get("resolution") != "arm_only" and "centroids_s" in band:
            vals = sorted(band["centroids_s"].values())
            real_fr = [(x + y) / 2 for x, y in zip(vals, vals[1:])]
        for sb in real_fr:
            q = p1 + sb * u
            ax.plot([q[0] - 26 * nh[0], q[0] + 26 * nh[0]], [q[1] - 26 * nh[1], q[1] + 26 * nh[1]],
                    color="#0b0b0b", lw=1.0, ls=(0, (3, 2)), zorder=4)
        hyp = [(meds[a] + meds[b]) / 2 for a, b in zip(lanes_sorted, lanes_sorted[1:])]
        for sb in hyp:
            q = p1 + sb * u
            xs = [q[0] - 34 * nh[0], q[0] + 34 * nh[0]]
            ys = [q[1] - 34 * nh[1], q[1] + 34 * nh[1]]
            ax.plot(xs, ys, color="white", lw=2.6, alpha=0.9, zorder=3.5, solid_capstyle="round")
            ax.plot(xs, ys, color="#898781", lw=1.2, ls=(0, (1, 1.4)), zorder=3.6)
        cx0, cy0 = seg[:, 0].min() - MARGIN, seg[:, 1].min() - MARGIN
        cx1, cy1 = seg[:, 0].max() + MARGIN, seg[:, 1].max() + MARGIN
        ax.set_xlim(max(0, cx0), min(w, cx1))
        ax.set_ylim(min(h, cy1), max(0, cy0))
        ax.axis("off")
        handles = [plt.Line2D([], [], color=C_LANE[l], lw=1.6,
                              label=f"carril {l} (GT, $n$={stats[l]['n']})")
                   for l in sorted(stats)]
        if real_fr:
            handles.append(plt.Line2D([], [], color="#0b0b0b", lw=1.0, ls=(0, (3, 2)),
                                      label="frontera REAL (producción)"))
        handles.append(plt.Line2D([], [], color="#898781", lw=1.2, ls=(0, (1, 1.4)),
                                  label="frontera per-lane HIPOTÉTICA"))
        ax.legend(handles=handles, loc="lower left", framealpha=0.9, frameon=True,
                  edgecolor="none", facecolor="white", borderpad=0.35, fontsize=6)
        name = f"diag_{'entrada' if kind == 'entry' else 'salida'}_{arm}"
        fig.savefig(OUT / f"{name}.png", dpi=300)
        plt.close(fig)
        print(f"{name}: {rows[-1]['n']} veh · LOO per-lane {rows[-1]['loo_perlane']} · "
              f"actual {rows[-1]['actual']} · {rows[-1]['veredicto']}")

    df = pd.DataFrame(rows)
    df.to_csv(OUT / "separabilidad.csv", index=False)
    (OUT / "provenance.txt").write_text("\n".join(
        ["diagnóstico separabilidad (read-only); insumos:"] +
        [f"  {p} = {m}" for p, m in sorted(stamp.items())]) + "\n")
    print(f"[ok] {OUT.relative_to(ROOT)}: 6 figuras + separabilidad.csv")


if __name__ == "__main__":
    main()
