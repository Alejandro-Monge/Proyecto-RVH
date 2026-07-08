"""Derives the LANE→TURN rule from GT v2 and emits `config/sites/<id>/lane_turn_rule.yaml`.

It drives the PREDICTED destination commit (under delay): on crossing the stop line, the entry lane
predicts the turn (and via `turns.csv` the exit approach → edge for `changeTarget`). Verification:
the lane predicts the turn 88–100 % (LOO), 100 % in the south.

KEY: the rule is derived at the **resolution the CLASSIFIER can deliver** (from
`zones.yaml: lane_bands.entry`), not at the GT's:
  - per_lane (east/north): rule per individual lane.
  - grouped (west {2,3}): lane 1 individual + group {2,3}.
  - arm_only (south): the classifier does NOT read the south lane from the IMAGE with confidence (10/14
    measurable at the stop line, wide CI) → **arm_only**. It is NOT that the south GT is missing (it is complete): it is
    that the image does not support it. At runtime the south falls to a FALLBACK with no turn prediction (its approach
    prior is ~uniform: 5 right/5 through/5 left → there is no reliable default). Declared, not hidden.

Read-only over GT/config; writes the yaml with --write. Net/image are not touched.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import pandas as pd
import yaml

ROOT = Path(__file__).resolve().parent.parent


def loo_accuracy(sub, key_fn):
    """Leave-one-out accuracy: predict each event's turn by the majority of its KEY (lane or
    group) without counting itself. Returns (correct, total)."""
    correct = 0
    keys = sub.apply(key_fn, axis=1)
    for ev in sub.index:
        k = keys[ev]
        others = sub[(keys == k) & (sub.index != ev)]
        if len(others):
            pred = others["turn"].mode().iloc[0]
        else:
            pred = sub.loc[ev, "turn"]  # no neighbors: do not penalize (degenerate case)
        correct += int(pred == sub.loc[ev, "turn"])
    return correct, len(sub)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--site", default="rusia")
    ap.add_argument("--write", action="store_true")
    args = ap.parse_args()

    gt = pd.read_csv(ROOT / "data/ground_truth/rusia/ground_truth_rusia_v2_carril_processed.csv")
    with open(ROOT / f"config/sites/{args.site}/zones.yaml") as fh:
        res = {a: b.get("resolution") for a, b in yaml.safe_load(fh)["lane_bands"]["entry"].items()}

    lines = [
        "# Regla CARRIL→GIRO calibrada del GT v2 (NO de la imagen). Predice el giro desde el carril",
        "# de entrada para el commit de destino PREDICHO (bajo delay); por turns.csv → brazo/edge de",
        "# salida para changeTarget. DERIVADA por scripts/derive_lane_turn_rule.py. NO editar a mano.",
        "# Resolución = la que el CLASIFICADOR entrega (zones.yaml: lane_bands.entry), no la del GT:",
        "#   per_lane (carril individual) · grouped (oeste: 1 + {2,3}) · arm_only (sur: el clasificador",
        "#   NO lee el carril del sur de la imagen con confianza —GT completo, imagen insuficiente—,",
        "#   prior de brazo ~uniforme → en runtime FALLBACK sin predicción de giro, declarado).",
        f"site_id: {args.site}",
        "derived_from: data/ground_truth/rusia/ground_truth_rusia_v2_carril_processed.csv",
        "derived_by: scripts/derive_lane_turn_rule.py",
        "rule:",
    ]
    print(f"{'brazo':6s} {'res':9s} | regla carril/grupo → giro (N) | LOO")
    for arm in ["sur", "norte", "este", "oeste"]:
        sub = gt[gt["entry_arm"] == arm].copy().set_index("event_id")
        r = res.get(arm, "per_lane")
        if r == "grouped":
            key_fn = lambda row: "1" if int(row["entry_lane"]) == 1 else "2-3"
        elif r == "arm_only":
            key_fn = lambda row: "ARM"           # a single key → predicts the approach majority
        else:
            key_fn = lambda row: str(int(row["entry_lane"]))
        # rule: key → majority turn (+N)
        keys = sub.apply(key_fn, axis=1)
        rule = {}
        for k in sorted(set(keys)):
            grp = sub[keys == k]
            rule[k] = (grp["turn"].mode().iloc[0], len(grp))
        corr, tot = loo_accuracy(sub, key_fn)
        # arm_prior: distribution of approach turns (to show whether there is a reliable default)
        dist = sub["turn"].value_counts().to_dict()
        usable = r != "arm_only"
        rule_yaml = "{" + ", ".join(f"\"{k}\": {v[0]}" for k, v in rule.items()) + "}" if usable else "{}"
        lines.append(f"  {arm}:")
        lines.append(f"    resolution: {r}")
        lines.append(f"    lane_turn: {rule_yaml}   # carril/grupo → giro (vacío si arm_only)")
        lines.append(f"    usable: {str(usable).lower()}   # arm_only (sur) → False: fallback sin predicción")
        lines.append(f"    arm_prior: {dict(dist)}   # distribución de giros del brazo")
        rule_str = ", ".join(f"{k}→{v[0]}(N={v[1]})" for k, v in rule.items())
        print(f"{arm:6s} {r:9s} | {rule_str:42s} | {corr}/{tot}={100*corr/tot:.0f}%"
              + ("" if usable else f"  [FALLBACK: prior {dict(dist)} ~uniforme]"))

    yaml_text = "\n".join(lines) + "\n"
    out = ROOT / f"config/sites/{args.site}/lane_turn_rule.yaml"
    if args.write:
        out.write_text(yaml_text, encoding="utf-8")
        print(f"\nESCRITO: {out.relative_to(ROOT)}")
    else:
        print("\n--- vista previa (usa --write) ---\n" + yaml_text)


if __name__ == "__main__":
    main()
