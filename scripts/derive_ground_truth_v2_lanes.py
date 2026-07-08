"""Derives the OFFICIAL per-lane GT v2 from the labeled raw file (tnew).

INPUT (untouched, evidence): data/ground_truth/rusia/ground_russia_tnew.csv
  Input raw file, already in the CORRECT cardinal convention (verified
  against the video and against v1) — that is why it does NOT go through correct_ground_truth_orientation.py:
  re-flipping it would break it. It carries entry_lane/exit_lane (convention Section 2bis of
  docs/site_rusia_geometry.md) and times in seconds; turn/movement_status/inclusion empty.

OUTPUT (current official): data/ground_truth/rusia/ground_truth_rusia_v2_carril_processed.csv
  Same data + derived turn/movement_status/inclusion, in CANONICAL ORDER
  (each lane attached to its approach).

DERIVATION (same method as v1, scripts/correct_ground_truth_orientation.py):
  - turn: turns.csv, requiring agreement with geometry (cross product) AND with the net (J5).
  - movement_status: 'allowed' if the pair is among the J5 movements.
  - inclusion: 'complete' if entry and exit are observed (arm != NA and both times present).

CHECKS (aborts without writing if any fails):
  - 73 events.
  - triple turn coherence (table == geometry == net) per pair.
  - cross-check against validated v1: for each O-D pair, derived turn/status must match
    those of v1 (the 72 shared). The extra event (north->south, confirmed real,
    73 vehicles total) is derived the same way.
"""
from __future__ import annotations

import csv
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
GT = ROOT / "data" / "ground_truth" / "rusia"
TNEW_IN = GT / "ground_russia_tnew.csv"                         # input raw file (UNTOUCHED)
V1_PROCESSED = GT / "_v1_por-brazo" / "ground_truth_rusia_processed.csv"  # validated v1, archived (control)
TURNS = ROOT / "config" / "sites" / "rusia" / "turns.csv"
NET = ROOT / "config" / "sites" / "rusia" / "sumo" / "intersection_rusia.net.xml"
OUT = GT / "ground_truth_rusia_v2_carril_processed.csv"         # CURRENT OFFICIAL

# Canonical order: original + lane attached to its approach.
CANON = ["event_id", "entry_arm", "entry_lane", "exit_arm", "exit_lane", "vehicle_class",
         "entry_time_s", "exit_time_s", "notes", "turn", "movement_status", "inclusion"]

EDGE_TO_CARD = {"E1": "oeste", "E3": "este", "E5": "sur", "E6": "norte"}
DIR_TO_TURN = {"r": "right", "s": "through", "l": "left", "t": "u_turn"}
ARM_VEC = {"norte": (0, 1), "sur": (0, -1), "este": (1, 0), "oeste": (-1, 0)}


def turn_by_geometry(entry: str, exit_: str) -> str:
    if entry == exit_:
        return "u_turn"
    ix, iy = ARM_VEC[entry]
    hin = (-ix, -iy)
    hout = ARM_VEC[exit_]
    cross = hin[0] * hout[1] - hin[1] * hout[0]
    dot = hin[0] * hout[0] + hin[1] * hout[1]
    if cross > 0:
        return "left"
    if cross < 0:
        return "right"
    return "through" if dot > 0 else "u_turn"


def load_net_moves(path: Path) -> dict[tuple[str, str], str]:
    text = path.read_text()
    pat = re.compile(r'<connection from="(E[1356])" to="(-E[1356])"[^>]*?dir="([a-z])"')
    moves: dict[tuple[str, str], str] = {}
    for m in pat.finditer(text):
        frm = EDGE_TO_CARD[m.group(1)]
        to = EDGE_TO_CARD[m.group(2)[1:]]
        moves[(frm, to)] = DIR_TO_TURN[m.group(3)]
    return moves


def load(p: Path) -> list[dict]:
    with p.open() as f:
        return list(csv.DictReader(f))


def main() -> int:
    net_moves = load_net_moves(NET)
    turns_tbl = {(r["entry_arm"], r["exit_arm"]): (r["turn"], r["status"])
                 for r in load(TURNS)}
    tnew = load(TNEW_IN)
    v1 = load(V1_PROCESSED)

    if len(tnew) != 73:
        print(f"ABORTADO: se esperaban 73 eventos en tnew, hay {len(tnew)}")
        return 1

    # v1: map pair -> (turn, status), requiring internal consistency
    v1_pair: dict[tuple[str, str], tuple[str, str]] = {}
    for r in v1:
        k = (r["entry_arm"], r["exit_arm"])
        val = (r["turn"], r["movement_status"])
        if k in v1_pair and v1_pair[k] != val:
            print(f"ABORTADO: v1 inconsistente en {k}: {v1_pair[k]} vs {val}")
            return 1
        v1_pair[k] = val

    out_rows = []
    turn_bad, v1_bad, trunc = [], [], []
    for r in tnew:
        a, b = r["entry_arm"], r["exit_arm"]
        geo = turn_by_geometry(a, b)
        net = net_moves.get((a, b))
        tab = turns_tbl.get((a, b), (None, None))[0]
        if not (geo == net == tab) or tab is None:
            turn_bad.append((r["event_id"], a, b, tab, geo, net))
            continue
        turn = tab
        status = "allowed" if (a, b) in net_moves else "not_in_net"
        observed = (a not in ("", "NA") and b not in ("", "NA")
                    and r["entry_time_s"] not in ("", "NA")
                    and r["exit_time_s"] not in ("", "NA"))
        inclusion = "complete" if observed else "truncated"
        if not observed:
            trunc.append(r["event_id"])
        # cross-check against v1 (shared pair)
        if (a, b) in v1_pair and v1_pair[(a, b)] != (turn, status):
            v1_bad.append((r["event_id"], a, b, (turn, status), v1_pair[(a, b)]))

        row = dict(r)
        row["turn"], row["movement_status"], row["inclusion"] = turn, status, inclusion
        out_rows.append({k: row.get(k, "") for k in CANON})

    if turn_bad or v1_bad:
        for x in turn_bad:
            print("  TURN-INCOHERENTE", x)
        for x in v1_bad:
            print("  DISCREPA-vs-v1 ", x)
        print("\nABORTADO: controles fallaron. No se escribe nada.")
        return 1

    # --- Summary ---
    print(f"[entrada] {TNEW_IN.name}: {len(tnew)} eventos (intacto)")
    print(f"[net] {len(net_moves)} movimientos en J5; turns.csv coherente con geometría y net.")
    print(f"[derivado] turn={dict(Counter(x['turn'] for x in out_rows))}")
    print(f"[derivado] movement_status={dict(Counter(x['movement_status'] for x in out_rows))}")
    print(f"[derivado] inclusion={dict(Counter(x['inclusion'] for x in out_rows))}")
    shared = sum(1 for r in tnew if (r['entry_arm'], r['exit_arm']) in v1_pair)
    print(f"[control v1] {shared}/73 eventos en pares compartidos; discrepancias turn/status: 0")
    print(f"[control trunc] eventos truncados: {len(trunc)} (todos complete)" if not trunc
          else f"[control trunc] truncados: {trunc}")

    with OUT.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CANON)
        w.writeheader()
        w.writerows(out_rows)
    print(f"\nESCRITO (oficial vigente): {OUT.relative_to(ROOT)}")
    print(f"  orden canónico: {','.join(CANON)}")
    print(f"  (intactos: {TNEW_IN.name}, {V1_PROCESSED.name} y demás v1)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
