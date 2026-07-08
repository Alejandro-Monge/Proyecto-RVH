"""[HISTORICAL — one-shot script already applied — DO NOT RE-RUN (there is a guard in main() that aborts).]

Corrects the orientation of the Russia ground truth (180° flip of the old convention).

CONTEXT: the original GT labeling was done against the PRE-ROTATION net frame
(cardinals flipped 180°), verified visually against the video. The correction is
a cardinal swap: NORTH<->SOUTH and EAST<->WEST, applied ONLY to the
cardinal fields (entry_arm/exit_arm).

A 180° flip is a PROPER rotation of the plane: the turn (left/through/right) is
INVARIANT. This script verifies it by re-deriving `turn` independently (by
geometry and by the net connections) and requiring it to match the original. The
same with `movement_status` (consistency check, not re-analysis). If something does not
match, it ABORTS and reports: the correction is not taken as valid.

It does not re-analyze the video nor re-derive `inclusion` nor the timecode->seconds conversion:
those fields are not cardinal and are kept as-is from the existing processed file.
"""
from __future__ import annotations

import csv
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
GT_DIR = ROOT / "data" / "ground_truth" / "rusia"
# The per-approach v1 files are archived in _v1_por-brazo/. The guard in main() prevents
# re-running (they are not re-written).
V1_DIR = GT_DIR / "_v1_por-brazo"
NET = ROOT / "config" / "sites" / "rusia" / "sumo" / "intersection_rusia.net.xml"

CRUDO_IN = V1_DIR / "ground_truth_rusia.csv"          # working raw file (timecode, OLD cardinals)
PROCESSED_IN = V1_DIR / "ground_truth_rusia_processed.csv"  # OLD processed (seconds + turn/status/inclusion)

RAW_OUT = V1_DIR / "ground_truth_rusia_raw.csv"        # new pristine corrected (timecode)
CRUDO_OUT = V1_DIR / "ground_truth_rusia.csv"          # corrected working raw file (timecode)
PROCESSED_OUT = V1_DIR / "ground_truth_rusia_processed.csv"  # corrected processed (seconds)

# 180° flip swap. Only affects cardinal fields.
SWAP = {"norte": "sur", "sur": "norte", "este": "oeste", "oeste": "este"}

# Physical edge<->cardinal mapping (hard-coded): the names
# E1..E6 are NOT cardinals after the net rotation.
EDGE_TO_CARD = {"E1": "oeste", "E3": "este", "E5": "sur", "E6": "norte"}
DIR_TO_TURN = {"r": "right", "s": "through", "l": "left", "t": "u_turn"}

# Unit vectors: direction from the J5 center toward each approach (North=+Y, East=+X).
ARM_VEC = {"norte": (0, 1), "sur": (0, -1), "este": (1, 0), "oeste": (-1, 0)}


def turn_by_geometry(entry: str, exit_: str) -> str:
    """Turn derived only from the geometry of the cardinals (independent of the net).

    The vehicle ENTERS from `entry` (heading toward the center = -vec(entry)) and EXITS toward
    `exit_` (heading = +vec(exit_)). The sign of the cross product gives left/right;
    parallels give through or a U-turn.
    """
    if entry == exit_:
        return "u_turn"
    ix, iy = ARM_VEC[entry]
    hin = (-ix, -iy)            # entry heading (toward the center)
    hout = ARM_VEC[exit_]       # exit heading
    cross = hin[0] * hout[1] - hin[1] * hout[0]
    dot = hin[0] * hout[0] + hin[1] * hout[1]
    if cross > 0:
        return "left"
    if cross < 0:
        return "right"
    return "through" if dot > 0 else "u_turn"


def load_net_movements(net_path: Path) -> dict[tuple[str, str], str]:
    """Allowed movements at J5 (cardinal->cardinal) with their turn, read from the net."""
    text = net_path.read_text()
    pat = re.compile(r'<connection from="(E[1356])" to="(-E[1356])"[^>]*?dir="([a-z])"')
    moves: dict[tuple[str, str], str] = {}
    for m in pat.finditer(text):
        frm = EDGE_TO_CARD[m.group(1)]
        to = EDGE_TO_CARD[m.group(2)[1:]]   # strip the '-' from the exit edge
        turn = DIR_TO_TURN[m.group(3)]
        moves[(frm, to)] = turn
    return moves


def main() -> int:
    # === GUARD: one-shot historical artifact, already applied. DO NOT RE-RUN.
    # Re-running it would re-apply the 180° swap on ALREADY-corrected data -> re-flips. And `turn`
    # is INVARIANT under 180°, so the internal checks below would NOT catch the double swap.
    # That is why it aborts here, before touching anything.
    print("HISTÓRICO — este script ya se ejecutó y NO debe re-ejecutarse: "
          "re-voltearía cardinales ya corregidos. Abortado a propósito.")
    return 2

    net_moves = load_net_movements(NET)
    print(f"[net] {len(net_moves)} movimientos permitidos en J5:")
    for (a, b), t in sorted(net_moves.items()):
        print(f"        {a:>5} -> {b:<5}  {t}")

    with PROCESSED_IN.open() as f:
        proc_rows = list(csv.DictReader(f))
    with CRUDO_IN.open() as f:
        crudo_reader = csv.DictReader(f)
        crudo_fields = crudo_reader.fieldnames
        crudo_rows = list(crudo_reader)

    assert len(proc_rows) == len(crudo_rows) == 72, "se esperaban 72 eventos"

    turn_mismatch, status_mismatch = [], []
    count_old: dict[str, int] = {}
    count_new: dict[str, int] = {}

    for proc, crud in zip(proc_rows, crudo_rows):
        assert proc["event_id"] == crud["event_id"]
        old_entry, old_exit = proc["entry_arm"], proc["exit_arm"]
        new_entry, new_exit = SWAP[old_entry], SWAP[old_exit]

        count_old[old_entry] = count_old.get(old_entry, 0) + 1
        count_new[new_entry] = count_new.get(new_entry, 0) + 1

        # --- turn check: two independent derivations must match the original ---
        geo_turn = turn_by_geometry(new_entry, new_exit)
        net_turn = net_moves.get((new_entry, new_exit))
        if not (geo_turn == net_turn == proc["turn"]):
            turn_mismatch.append((proc["event_id"], old_entry, old_exit, new_entry,
                                  new_exit, proc["turn"], geo_turn, net_turn))

        # --- movement_status check: allowed if the pair is in the net ---
        derived_status = "allowed" if (new_entry, new_exit) in net_moves else "not_in_net"
        if derived_status != proc["movement_status"]:
            status_mismatch.append((proc["event_id"], new_entry, new_exit,
                                    proc["movement_status"], derived_status))

        # Apply the swap to the cardinal fields (rest untouched)
        proc["entry_arm"], proc["exit_arm"] = new_entry, new_exit
        crud["entry_arm"], crud["exit_arm"] = new_entry, new_exit

    print("\n=== CONTROL turn (invariante bajo rotación de 180°) ===")
    print(f"  mismatches: {len(turn_mismatch)}")
    print("\n=== CONTROL movement_status (re-derivado contra el net) ===")
    print(f"  mismatches: {len(status_mismatch)}")

    if turn_mismatch or status_mismatch:
        for r in turn_mismatch:
            print("  TURN  ", r)
        for r in status_mismatch:
            print("  STATUS", r)
        print("\nABORTADO: hay discrepancias. No se escribe nada.")
        return 1

    print("\n=== Conteo por brazo de ENTRADA (antes vs después) ===")
    for arm in ("norte", "sur", "este", "oeste"):
        print(f"  {arm:>5}: {count_old.get(arm,0):>2}  ->  "
              f"{count_new.get(SWAP[arm],0):>2}  (ahora rotulado '{SWAP[arm]}')")
    print(f"  TOTAL: {sum(count_old.values())} -> {sum(count_new.values())}")

    # --- Write (only if all checks passed) ---
    with PROCESSED_OUT.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=proc_rows[0].keys())
        w.writeheader()
        w.writerows(proc_rows)

    for path in (CRUDO_OUT, RAW_OUT):
        with path.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=crudo_fields)
            w.writeheader()
            w.writerows(crudo_rows)

    print("\nEscritos:")
    for p in (RAW_OUT, CRUDO_OUT, PROCESSED_OUT):
        print(f"  {p.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
