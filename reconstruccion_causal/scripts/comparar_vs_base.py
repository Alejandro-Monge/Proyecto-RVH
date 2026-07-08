"""
comparar_vs_base — classified diff between the output of the CAUSAL reconstructor
and that of the collaborator's base over the SAME CSV.

Verification criterion: every difference must be explained by
one of the corrected leaks; an unclassifiable difference = FAILED
verification. Classes:

  IDENTICO                common row (veh, tick) with the same values.
  COLA_FINAL_Y_RETIRO     causal-only row with tick > the veh's last base tick:
                          UNCERTAIN extrapolation of the silence tail +
                          shift of the removal by timeout (leaks 1 and 3).
  AUSENCIA_POR_TIMEOUT    base-only row within a [removal, resume] window
                          of the causal: the base kept extrapolating because it
                          knew the end of the gap; the causal removed by timeout
                          and re-acquired when the messages resumed (leak 1).
  FRAME_HELD_CAUSAL       common row where the causal froze the frame (the
                          HELD-until-usable-anchor policy) because the base
                          interpolated toward an anchor outside the s+buffer
                          window (the interp causality violation, corrected).
  UNEXPLAINED_*           anything else -> verification FAILURE.

Leak 2 (route) does not appear in the state log by design (keepRoute=2 forces
the exact position); it is verified separately by comparing the position_error
of both and the causal's ruta_report.
"""

import argparse
import csv
import json
import math
import os
from collections import defaultdict

TICK = 0.1
TOL = 1e-6


def load_log(run_dir):
    path = os.path.join(run_dir, "results", "validation_log.csv")
    with open(path) as f:
        return list(csv.DictReader(f))


def load_summary(run_dir):
    with open(os.path.join(run_dir, "results", "summary.json")) as f:
        return json.load(f)


def fnum(s):
    return None if s in ("", None) else float(s)


def ang_delta(a, b):
    return abs((a - b + 180) % 360 - 180)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", required=True, help="dir de corrida de la base")
    ap.add_argument("--causal", required=True, help="dir de corrida causal")
    ap.add_argument("--out", default="", help="ruta del reporte JSON (opcional)")
    args = ap.parse_args()

    base_rows = {(r["veh"], int(r["tick_i"])): r for r in load_log(args.base)}
    causal_rows = {(r["veh"], int(r["tick_i"])): r for r in load_log(args.causal)}
    causal_sum = load_summary(args.causal)
    base_sum = load_summary(args.base)

    delay_ticks = int(round(causal_sum["delay_aplicado_s"] / TICK))

    # absence windows of the causal: [tick(retire_t), tick(resume_t)+delay+5]
    # (the +delay+5 covers staging and insertion after re-acquisition)
    absence = defaultdict(list)
    for veh, eps in causal_sum.get("retiro_report", {}).items():
        for ep in eps:
            t_ret = int(round(ep["retire_t"] / TICK))
            if "resume_t" in ep:
                t_res = int(round(ep["resume_t"] / TICK)) + delay_ticks + 5
            else:
                t_res = 10 ** 9   # final removal: to infinity
            absence[veh].append((t_ret, t_res))

    base_max_tick = defaultdict(int)
    for (veh, tick) in base_rows:
        base_max_tick[veh] = max(base_max_tick[veh], tick)

    classes = defaultdict(int)
    unexplained = []
    max_deltas = {"pos_m": 0.0, "heading_deg": 0.0, "sim_time_s": 0.0}

    common = base_rows.keys() & causal_rows.keys()
    for key in common:
        b, c = base_rows[key], causal_rows[key]
        dx = float(b["x_cmd"]) - float(c["x_cmd"])
        dy = float(b["y_cmd"]) - float(c["y_cmd"])
        dpos = math.hypot(dx, dy)
        dang = ang_delta(float(b["heading_cmd"]), float(c["heading_cmd"]))
        bs, cs = fnum(b["sim_time_csv"]), fnum(c["sim_time_csv"])
        dst = (abs(bs - cs) if (bs is not None and cs is not None)
               else (0.0 if bs is None and cs is None else float("inf")))
        max_deltas["pos_m"] = max(max_deltas["pos_m"], dpos)
        max_deltas["heading_deg"] = max(max_deltas["heading_deg"], dang)
        if dst != float("inf"):
            max_deltas["sim_time_s"] = max(max_deltas["sim_time_s"], dst)
        same = (dpos <= TOL and dang <= TOL and dst <= TOL
                and b["provenance"] == c["provenance"]
                and (b["seg"] or "") == (c["seg"] or ""))
        if same:
            classes["IDENTICO"] += 1
        elif (c["provenance"] == "INTERPOLATED"
              and (c["seg"] or "").endswith(("(HELD)", "(DR)"))
              and b["provenance"] == "INTERPOLATED"):
            classes["FRAME_HELD_CAUSAL"] += 1
        else:
            classes["UNEXPLAINED_VALOR_DISTINTO"] += 1
            if len(unexplained) < 20:
                unexplained.append({"clase": "VALOR_DISTINTO", "veh": key[0],
                                    "tick": key[1], "dpos_m": round(dpos, 6),
                                    "dang": round(dang, 6),
                                    "prov_base": b["provenance"],
                                    "prov_causal": c["provenance"],
                                    "seg_base": b["seg"], "seg_causal": c["seg"]})

    for key in causal_rows.keys() - base_rows.keys():
        veh, tick = key
        if tick > base_max_tick.get(veh, -1):
            classes["COLA_FINAL_Y_RETIRO"] += 1
        else:
            # within the base life: only expected right after re-acquisition
            # if the base had HELD there; any extra causal row within-life
            # is suspicious
            classes["UNEXPLAINED_SOLO_CAUSAL"] += 1
            if len(unexplained) < 20:
                unexplained.append({"clase": "SOLO_CAUSAL", "veh": veh,
                                    "tick": tick,
                                    "prov": causal_rows[key]["provenance"]})

    for key in base_rows.keys() - causal_rows.keys():
        veh, tick = key
        if any(a <= tick <= z for (a, z) in absence.get(veh, [])):
            classes["AUSENCIA_POR_TIMEOUT"] += 1
        else:
            classes["UNEXPLAINED_SOLO_BASE"] += 1
            if len(unexplained) < 20:
                unexplained.append({"clase": "SOLO_BASE", "veh": veh,
                                    "tick": tick,
                                    "prov": base_rows[key]["provenance"]})

    # leak 2 (route): the commanded position must reproduce exactly in both
    fuga2 = {
        "position_error_max_base": base_sum["error_pos_m"]["max"],
        "position_error_max_causal": causal_sum["error_pos_m"]["max"],
        "ruta_causal": causal_sum.get("ruta_report"),
    }
    # leak 1: shift of the final removal per vehicle (base: last_tick+1)
    retiro_final_delta = []
    for veh, eps in causal_sum.get("retiro_report", {}).items():
        fin = [e for e in eps if "resume_t" not in e]
        if fin and veh in base_max_tick:
            delta = fin[-1]["retire_t"] - (base_max_tick[veh] + 1) * TICK
            retiro_final_delta.append(round(delta, 3))
    retiro_final_delta.sort()

    n_unexpl = sum(v for k, v in classes.items() if k.startswith("UNEXPLAINED"))
    report = {
        "base": args.base, "causal": args.causal,
        "filas_base": len(base_rows), "filas_causal": len(causal_rows),
        "clases": dict(classes),
        "max_deltas_en_comunes": {k: round(v, 9) for k, v in max_deltas.items()},
        "fuga2_ruta": fuga2,
        "fuga1_retiro_final_delta_s": {
            "n": len(retiro_final_delta),
            "min": retiro_final_delta[0] if retiro_final_delta else None,
            "mediana": retiro_final_delta[len(retiro_final_delta) // 2]
                       if retiro_final_delta else None,
            "max": retiro_final_delta[-1] if retiro_final_delta else None,
        },
        "unexplained_ejemplos": unexplained,
        "verdict": "PASS" if n_unexpl == 0 else "FAIL",
    }
    if args.out:
        os.makedirs(os.path.dirname(args.out), exist_ok=True)
        with open(args.out, "w") as f:
            json.dump(report, f, indent=2)

    print(json.dumps({k: report[k] for k in
                      ["filas_base", "filas_causal", "clases",
                       "max_deltas_en_comunes", "fuga1_retiro_final_delta_s",
                       "verdict"]}, indent=2))
    print(f"fuga2 position_error_max base/causal = "
          f"{fuga2['position_error_max_base']}/{fuga2['position_error_max_causal']}")
    if unexplained:
        print("\nEJEMPLOS NO EXPLICADOS:")
        for u in unexplained[:10]:
            print(" ", u)
    raise SystemExit(0 if n_unexpl == 0 else 1)


if __name__ == "__main__":
    main()
