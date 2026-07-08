"""Shadow↔GT matching (sub-step B) — baseline of the per-approach route metric.

Matches the shadow's ROUTE EVENTS (event-per-track layer, sub-step A) against the 73
route events of GT v2, by ENTRY APPROACH + entry time, within a tolerance
window. It implements the metrics protocol (docs/protocolo_metricas_sombras.md Section 6) and
produces the baseline of route detection (precision/recall) and exit accuracy, BROKEN DOWN
PER APPROACH and aggregated.

GUARDRAILS:
  - Reads the events CSV (verify_out/events/<id>/track_events.csv) and the GT; it does NOT touch the
    events layer or the GT.
  - Reads the tolerance window from config (site.yaml: matching_tolerance_s). It does not create config.
  - Writes only to verify_out/events/<id>/. It lives as a script (module consolidation deferred).

SHADOW UNIVERSE: only the complete CROSS-ARM events (movement ==
  'cross') are "routes" candidate to match. The same-arm ones (incl. traffic stopped at red in the
  south) and the partials are SET ASIDE: they do NOT match and are NOT route ghosts (a real parked car is
  not a hallucinated route). Their count is reported separately. The GT is 73 complete routes
  (entry_arm != exit_arm, 0 U-turns): no same-arm could match.

COST CRITERION (Section 6.2): cost = |Δt_entry|; edge valid only if SAME entry approach and
  |Δt_entry| <= window; ONE-TO-ONE minimum-cost assignment (Hungarian, scipy
  linear_sum_assignment), resolved per approach (the problem is block-diagonal).

TIE-BREAK BY EXIT (Section 6.2): when two GT fall in the same entry δ-band of a single shadow
  event (a real entry tie), the EXIT breaks the tie. It is encoded as a scalar lexicographic
  cost: cost = floor(|Δt_entry|/DELTA)*BIG + exit_term, with
  exit_term = (exit_arm different ? K : 0) + |Δt_exit|, and BIG > max(exit_term) so the
  entry DOMINATES and the exit only orders within a DELTA band. DELTA is the frame's
  temporal resolution (~0.1 s), not a free threshold. The script reports how many real ties there are
  (audit: if ~0, the tie-break is safe but rarely triggers).

OUTPUTS (verify_out/events/<id>/): matches.csv, fantasmas.csv, omissions.csv, window_sweep.csv.

NOTE — what this baseline measures (honest): the matching anchors by entry approach, so the
  entry approach of a matched pair is correct BY CONSTRUCTION; entry-approach errors
  show up as GHOSTS/OMISSIONS, not as mis-matched pairs. What is measured directly is
  (a) route DETECTION (precision/recall per entry approach) and (b) EXIT APPROACH accuracy
  among the matched. The lane and the fine Section 7.1 come when the events layer gains lane.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from digital_shadow.config.site import load_site_config

# The pure matching logic lives in scripts/matching_core.py — the SINGLE
# Section 6 specification for BOTH shadows (vision and messages; single-source
# doctrine). It is RE-EXPORTED here because reconnect_fragments.py imports
# `match_events_to_gt as mm` and uses mm.run_matching / mm.per_arm_report;
# byte-identical equivalence against the reference artifacts.
from matching_core import (DELTA_S, K_EXIT_ARM, BIG, INVALID, lexi_cost,  # noqa: F401
                           match_arm, run_matching, audit_ties, per_arm_report)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--site", default="rusia")
    ap.add_argument("--events", default=None)
    ap.add_argument("--out-dir", default=None)
    args = ap.parse_args()

    site = load_site_config(args.site)
    arms = list(site.arm_set)
    window = site_tolerance = float(_read_tolerance(args.site))

    events_path = Path(args.events) if args.events else ROOT / "verify_out" / "events" / args.site / "track_events.csv"
    gt_path = ROOT / "data" / "ground_truth" / args.site / f"ground_truth_{args.site}_v2_carril_processed.csv"
    out_dir = Path(args.out_dir) if args.out_dir else ROOT / "verify_out" / "events" / args.site
    out_dir.mkdir(parents=True, exist_ok=True)

    ev = pd.read_csv(events_path)
    shadow = ev[(ev.status == "complete") & (ev.movement == "cross")].reset_index(drop=True)
    same_arm = int(((ev.status == "complete") & (ev.movement == "same")).sum())
    partial = int((ev.status == "partial").sum())
    gt = pd.read_csv(gt_path)

    # --- Audit of real ties ---
    print("=== AUDITORÍA del desempate (empates reales en δ-banda) ===")
    for w in (window, 3.0):
        print(f"  ventana {w:.1f}s: eventos de sombra con >=2 GT en la MISMA δ-banda ({DELTA_S}s): "
              f"{audit_ties(shadow, gt, arms, w)}")

    # --- Baseline at the config window ---
    pares, fant, omis, contested = run_matching(shadow, gt, arms, window)
    print(f"\n=== LÍNEA BASE (ventana = matching_tolerance_s = {window:.2f}s, de site.yaml) ===")
    print(f"Universo sombra (cross-arm) = {len(shadow)} | GT = {len(gt)} | "
          f"apartados: same-arm {same_arm}, parciales {partial} (no casan, no fantasmas de ruta)")
    rep = per_arm_report(pares, fant, omis, shadow, gt, arms)
    print(rep.to_string(index=False))
    res = pares.dt_entry
    print(f"\nDesfase de entrada CON SIGNO (sombra − GT) de los casados: "
          f"media {res.mean():+.3f}s, mediana {res.median():+.3f}s, |mediana| {res.abs().median():.3f}s")
    print(f"Casos disputados (>=2 GT en ventana): {contested}")

    # --- Write artifacts ---
    pares.to_csv(out_dir / "matches.csv", index=False)
    (fant if len(fant) else pd.DataFrame(columns=shadow.columns)).to_csv(out_dir / "fantasmas.csv", index=False)
    (omis if len(omis) else pd.DataFrame(columns=gt.columns)).to_csv(out_dir / "omissions.csv", index=False)
    rep.to_csv(out_dir / "baseline_por_brazo.csv", index=False)

    # --- Window sensitivity sweep (Section 6.3) ---
    sweep = []
    for w in np.round(np.arange(0.5, 3.0001, 0.1), 2):
        p, f, o, c = run_matching(shadow, gt, arms, float(w))
        r = p.dt_entry
        sweep.append(dict(
            ventana_s=w, casados=len(p), fantasmas=len(f), omisiones=len(o),
            residual_abs_mediana=round(r.abs().median(), 3) if len(p) else float("nan"),
            residual_con_signo_mediana=round(r.median(), 3) if len(p) else float("nan"),
            disputados=c,
            acierto_salida=round(p.exit_correct.mean(), 3) if len(p) else float("nan"),
        ))
    sweep_df = pd.DataFrame(sweep)
    sweep_df.to_csv(out_dir / "window_sweep.csv", index=False)

    # --- Section 7.2 DELAY: dt_entry is t_stopcross_shadow − t_stopcross_GT per
    # matched vehicle (events and GT both anchor at the stop line). SAME definition
    # as the message shadow (matching_mensajes.py); it is reported as an artifact
    # with the full distribution.
    d = pares.dt_entry
    rows_delay = [dict(brazo="TOTAL", n=len(d), mediana_s=round(d.median(), 3),
                       media_s=round(d.mean(), 3), std_s=round(d.std(), 3),
                       p10_s=round(d.quantile(0.10), 3), p90_s=round(d.quantile(0.90), 3),
                       min_s=round(d.min(), 3), max_s=round(d.max(), 3))]
    for a in arms:
        da = pares[pares.entry_arm == a].dt_entry
        if len(da):
            rows_delay.append(dict(brazo=a, n=len(da), mediana_s=round(da.median(), 3),
                                   media_s=round(da.mean(), 3), std_s=round(da.std(), 3),
                                   p10_s=round(da.quantile(0.10), 3),
                                   p90_s=round(da.quantile(0.90), 3),
                                   min_s=round(da.min(), 3), max_s=round(da.max(), 3)))
    pd.DataFrame(rows_delay).to_csv(out_dir / "delay_cruce.csv", index=False)
    print(f"\n=== DELAY §7.2 (cruce de pare, sombra−GT) -> delay_cruce.csv ===")
    print(pd.DataFrame(rows_delay).to_string(index=False))
    print(f"\n=== BARRIDO DE VENTANA [0.5, 3.0]s (paso 0.1) ===")
    print(sweep_df.to_string(index=False))
    print(f"\nArtefactos -> {out_dir.relative_to(ROOT)}/ "
          f"(matches.csv, fantasmas.csv, omissions.csv, baseline_por_brazo.csv, window_sweep.csv)")
    return 0


def _read_tolerance(site_id: str) -> float:
    """matching_tolerance_s from site.yaml (not in SiteConfig; read directly)."""
    import yaml
    with open(ROOT / "config" / "sites" / site_id / "site.yaml", "r", encoding="utf-8") as fh:
        return float(yaml.safe_load(fh)["matching_tolerance_s"])


if __name__ == "__main__":
    raise SystemExit(main())
