"""Validates the lane classifier (ENTRY and EXIT) against GT v2 and measures the Section 7.1 levels:
APPROACH → APPROACH+ENTRY LANE → FULL ROUTE (4 fields). Over RECONNECTED tracks.

Lane CONDITIONED on the approach (the matching anchors on approach+entry time → correct entry
approach by construction). Per-approach resolution according to `lane_bands` in zones.yaml (entry/exit).

Lane accuracy by the resolution rule:
  per_lane  : shadow lane == GT lane.
  grouped   : GT 1 ↔ '1'; GT 2 or 3 ↔ '2-3' (west entry only, SENSOR limit).
  arm_only  : not resolved (south entry only, SAMPLE limit) → does not contribute.
East exit: ~14% BEHAVIORAL error (not sensor) → individual by the rule; a faithful shadow
reflects the human spread.

Gives TWO estimates (in-sample deployed + honest LOO), count + Wilson CI, confusion matrix,
and the south→west anchor (egress lane compared against the GT, not against a fixed number). Does NOT write artifacts.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from digital_shadow.config.site import load_site_config
from digital_shadow.geometry.ground_contact import add_ground_contact
import derive_track_events as dte

ARMS = ["este", "norte", "oeste", "sur"]


def wilson(k, n, z=1.96):
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n; d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = (z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))) / d
    return max(0.0, c - h), min(1.0, c + h)


def norm_lane(v):
    """Normalizes the lane label from the CSV (which the round-trip may read as 1.0/NaN)."""
    if pd.isna(v) or v == "":
        return ""
    s = str(v)
    return s[:-2] if s.endswith(".0") else s


def group_label(arm, lane, side_bands):
    # FORCED PER-LANE on the 4 approaches: grouping lanes (grouped) or turning off
    # the classification (arm_only) to raise the accuracy CONTRADICTS the spatial
    # fidelity (project doctrine). `side_bands`/`resolution` remain a parameter with
    # no effect. The number is measured per-lane even if the sample is scarce
    # (sur), with its caveat, not by grouping.
    return str(int(lane))


def loo(sub):
    sub = sub.reset_index(drop=True); n = len(sub); e = 0
    for i in range(n):
        cent = sub.drop(i).groupby("glabel").s.mean()
        if cent.empty:
            continue
        pred = min(cent.index, key=lambda k: abs(cent[k] - sub.loc[i, "s"]))
        if pred != sub.loc[i, "glabel"]:
            e += 1
    return e, n


def main():
    site = load_site_config("rusia")
    center = np.array([site.frame_width / 2.0, site.frame_height / 2.0])
    stop = dte.load_stop_lines("rusia")
    bands = dte.load_lane_bands("rusia")
    eb, xb = bands.get("entry", {}), bands.get("exit", {})
    ev = pd.read_csv(ROOT / "verify_out/events/rusia/reconnected/track_events.csv")
    sh_entry = {t: norm_lane(v) for t, v in zip(ev.track_id, ev.entry_lane)}
    sh_exit = {t: norm_lane(v) for t, v in zip(ev.track_id, ev.exit_lane)}
    sh_exitarm = dict(zip(ev.track_id, ev.exit_arm))
    gt = pd.read_csv(ROOT / "data/ground_truth/rusia/ground_truth_rusia_v2_carril_processed.csv").set_index("event_id")
    matches = pd.read_csv(ROOT / "verify_out/events/rusia/reconnected/matches.csv")
    tracks = pd.read_csv(ROOT / "verify_out/events/rusia/reconnected/tracks_reconnected.csv"); add_ground_contact(tracks)
    by = {t: g.sort_values("frame_index") for t, g in tracks.groupby("track_id")}

    rec = []
    for _, m in matches.iterrows():
        e = gt.loc[m.event_id]; tid = int(m.track_id); g = by.get(tid)
        s_exit = dte.lane_cross_coord(g, stop[e.exit_arm], center, kind="exit") if (g is not None and e.exit_arm in stop) else None
        rec.append(dict(event_id=int(m.event_id), tid=tid,
                        entry_arm=e.entry_arm, gt_el=int(e.entry_lane), sh_el=sh_entry.get(tid, ""),
                        gt_xarm=e.exit_arm, gt_xl=int(e.exit_lane), sh_xl=sh_exit.get(tid, ""),
                        sh_xarm=sh_exitarm.get(tid, ""), s_exit=s_exit))
    mt = pd.DataFrame(rec)
    n_gt = {a: int((gt.entry_arm == a).sum()) for a in ARMS}

    # ---------- 1. EXIT: confusion + LOO by exit_arm ----------
    print("=" * 88)
    print("1) CARRIL DE SALIDA — confusión (desplegado) + LOO honesto, por exit_arm (pares casados)")
    print("=" * 88)
    for arm in ARMS:
        sub = mt[mt.gt_xarm == arm]
        res = xb.get(arm, {}).get("resolution")
        ncent = len(xb.get(arm, {}).get("centroids_s", {}))
        if ncent <= 1:
            print(f"\n{arm}: 1 carril de egreso (trivial); {len(sub)} casados, exit_lane=1")
            continue
        s2 = sub[sub.sh_xl != ""]
        ok = sum(group_label(arm, r.gt_xl, xb) == r.sh_xl for r in s2.itertuples()); n = len(s2)
        lo, hi = wilson(ok, n)
        print(f"\n{arm} ({res}): acierto carril salida = {ok}/{n} = {ok/n*100 if n else 0:.0f}% IC95[{lo*100:.0f},{hi*100:.0f}]%")
        conf = {}
        for r in s2.itertuples():
            t = group_label(arm, r.gt_xl, xb); conf[(t, r.sh_xl)] = conf.get((t, r.sh_xl), 0) + 1
        for (t, p), c in sorted(conf.items()):
            print(f"    GT {t} -> shadow {p} : {c}{'' if t == p else '  <-- error'}")
        # LOO
        sl = sub[sub.s_exit.notna()].copy()
        sl["glabel"] = [group_label(arm, l, xb) for l in sl.gt_xl]
        e, n2 = loo(sl[["glabel", "s_exit"]].rename(columns={"s_exit": "s"}))
        lo, hi = wilson(e, n2)
        print(f"    LOO honesto: {e}/{n2} = {e/n2*100 if n2 else 0:.0f}% err IC95[{lo*100:.0f},{hi*100:.0f}]%")

    # ---------- 2. south->west ANCHOR (observed egress vs GT; lane READ from the GT, NOT hardcoded) ----------
    print("\n" + "=" * 88)
    print("2) ANCLA sur→oeste (giro a la izquierda → egreso oeste; carril de salida vs GT)")
    print("=" * 88)
    anchor = mt[(mt.entry_arm == "sur") & (mt.gt_xarm == "oeste")]
    for r in anchor.itertuples():
        gt_lab = group_label(r.gt_xarm, r.gt_xl, xb)      # GT lane (read, NOT a fixed number)
        ok = (r.sh_xarm == r.gt_xarm and r.sh_xl != "" and r.sh_xl == gt_lab)
        print(f"  ev{r.event_id} tid{r.tid}: GT exit {r.gt_xarm}/{gt_lab} | shadow exit {r.sh_xarm}/{r.sh_xl}  {'OK' if ok else 'REVISAR'}")

    # ---------- 3. Section 7.1 LEVELS: approach -> approach+entry lane -> FULL ROUTE ----------
    print("\n" + "=" * 88)
    print("3) NIVELES §7.1: BRAZO (recall) → +CARRIL ENTRADA → RUTA COMPLETA (4 campos)")
    print("=" * 88)
    def el_ok(r):
        return r.sh_el != "" and group_label(r.entry_arm, r.gt_el, eb) == r.sh_el
    def xl_ok(r):
        return r.sh_xl != "" and group_label(r.gt_xarm, r.gt_xl, xb) == r.sh_xl
    def xarm_ok(r):
        return r.sh_xarm == r.gt_xarm
    print(f"{'brazo':6} {'GT':>3} {'brazo':>11} {'+carril_ent':>12} {'ruta_completa':>14}")
    tot = dict(gt=0, m=0, e=0, f=0); totres = dict(gt=0, e=0, f=0)
    for arm in ARMS:
        sub = mt[mt.entry_arm == arm]; g = n_gt[arm]; m = len(sub)
        e = sum(el_ok(r) for r in sub.itertuples())
        f = sum(el_ok(r) and xarm_ok(r) and xl_ok(r) for r in sub.itertuples())
        print(f"{arm:6} {g:>3} {m}/{g}={m/g*100:>3.0f}%   {e}/{g}={e/g*100:>3.0f}%      {f}/{g}={f/g*100:>3.0f}%")
        tot["gt"] += g; tot["m"] += m; tot["e"] += e; tot["f"] += f
        if eb.get(arm, {}).get("resolution") != "arm_only":
            totres["gt"] += g; totres["e"] += e; totres["f"] += f
    print(f"{'TOTAL':6} {tot['gt']:>3} {tot['m']}/{tot['gt']}={tot['m']/tot['gt']*100:>3.0f}%   "
          f"{tot['e']}/{tot['gt']}={tot['e']/tot['gt']*100:>3.0f}%      "
          f"{tot['f']}/{tot['gt']}={tot['f']/tot['gt']*100:>3.0f}%")
    print(f"  (sin sur —brazos con carril de entrada—: +carril_ent {totres['e']}/{totres['gt']}="
          f"{totres['e']/totres['gt']*100:.0f}%, ruta_completa {totres['f']}/{totres['gt']}={totres['f']/totres['gt']*100:.0f}%)")
    print("\nLa RUTA COMPLETA exige los 4 campos (entrada brazo+carril, salida brazo+carril). La caída")
    print("vs +carril_entrada es el coste de exigir también la salida (brazo de salida + carril de salida).")


if __name__ == "__main__":
    main()
