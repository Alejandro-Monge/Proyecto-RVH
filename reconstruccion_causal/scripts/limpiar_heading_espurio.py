"""
limpiar_heading_espurio — census and cleaning of the spurious headingDeg of the
Monte Carlo sweep (TraCI INVALID sentinel rounded to 6 digits in
post-exit messages with an empty laneId).

Cleaning mode: the spurious
headingDeg field is rewritten with the EXACT SENTINEL (-1073741824) — what
a correct logger would have written exactly as TraCI delivered it. The field is
not emptied because an empty field breaks the float parsing of ALL the consumers
(the causal reconstructor and the base player); the exact sentinel is
parseable, its semantics in moveToXY are official ("the edge's natural angle")
and it makes the boundary-sanitizing unnecessary (which becomes a no-op:
sanitize(-1073741824) == -1073741824). The rest of the line (x, y, timestamp,
empty laneId, speed, ...) stays INTACT: the message still exists and still
feeds the vehicle's life in the shadow (removal timeout).

The originals are NOT touched: the output goes to a parallel mirror path
(--dst-root), with before/after md5 per file in a JSON manifest.

Usage:
  # census without writing anything
  python3 scripts/limpiar_heading_espurio.py --src-root <montecarlo> --solo-censo
  # clean a single CSV (demo)
  python3 scripts/limpiar_heading_espurio.py --src-root <montecarlo> \
      --dst-root <montecarlo_limpio> --solo 2Hz/P30/rep000.csv
  # full sweep
  python3 scripts/limpiar_heading_espurio.py --src-root <montecarlo> --dst-root <montecarlo_limpio>
"""

import argparse
import csv
import hashlib
import json
import os
import sys
from collections import defaultdict

SENTINEL_EXACTO = "-1073741824"
# detection threshold: any physically impossible heading (the 6-digit rounding
# of the sentinel produces ~-1.07374e+09; a real heading is in [0,360))
UMBRAL_ESPURIO = -1000.0


def md5(path):
    h = hashlib.md5()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def procesar(src, dst=None):
    """Returns statistics; if dst, writes the clean copy."""
    with open(src, newline="") as f:
        rows = list(csv.DictReader(f))
        fields = list(rows[0].keys()) if rows else []
    st = {"filas": len(rows), "espurias": 0, "lane_vacio": 0,
          "espuria_sin_lane_vacio": 0, "lane_vacio_sin_espuria": 0,
          "vehiculos": set()}
    for r in rows:
        h = float(r["headingDeg"])
        espuria = h < UMBRAL_ESPURIO
        vacio = r["laneId"] == ""
        if espuria:
            st["espurias"] += 1
            st["vehiculos"].add(r["vehicleId"])
            if not vacio:
                st["espuria_sin_lane_vacio"] += 1
            if dst is not None:
                r["headingDeg"] = SENTINEL_EXACTO
        if vacio:
            st["lane_vacio"] += 1
            if not espuria:
                st["lane_vacio_sin_espuria"] += 1
    if dst is not None:
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        with open(dst, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            w.writerows(rows)
    return st


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src-root", required=True)
    ap.add_argument("--dst-root", default="")
    ap.add_argument("--solo", default="", help="ruta relativa de UN csv (demo)")
    ap.add_argument("--solo-censo", action="store_true")
    args = ap.parse_args()
    if not args.solo_censo and not args.dst_root:
        ap.error("se requiere --dst-root (o --solo-censo)")

    csvs = []
    for dirpath, _, files in os.walk(args.src_root):
        for fn in sorted(files):
            if fn.endswith(".csv") and fn.lower() != "checksums":
                rel = os.path.relpath(os.path.join(dirpath, fn), args.src_root)
                csvs.append(rel)
    if args.solo:
        csvs = [c for c in csvs if c == args.solo]
        if not csvs:
            sys.exit(f"no se encontró {args.solo} bajo {args.src_root}")

    manifest = {}
    tot = defaultdict(int)
    vehs_global = set()
    incoherencias = []
    for rel in csvs:
        src = os.path.join(args.src_root, rel)
        dst = None if args.solo_censo else os.path.join(args.dst_root, rel)
        md5_antes = md5(src)
        st = procesar(src, dst)
        entry = {"filas": st["filas"], "espurias_limpiadas": st["espurias"],
                 "lane_vacio": st["lane_vacio"],
                 "vehiculos": sorted(st["vehiculos"]),
                 "md5_original": md5_antes}
        if dst is not None:
            entry["md5_limpio"] = md5(dst)
        manifest[rel] = entry
        tot["archivos"] += 1
        tot["filas"] += st["filas"]
        tot["espurias"] += st["espurias"]
        tot["lane_vacio"] += st["lane_vacio"]
        vehs_global |= st["vehiculos"]
        if st["espuria_sin_lane_vacio"] or st["lane_vacio_sin_espuria"]:
            incoherencias.append((rel, st["espuria_sin_lane_vacio"],
                                  st["lane_vacio_sin_espuria"]))

    print(f"archivos={tot['archivos']} filas={tot['filas']} "
          f"espurias={tot['espurias']} ({100.0*tot['espurias']/max(tot['filas'],1):.1f}%) "
          f"lane_vacio={tot['lane_vacio']}")
    print(f"vehiculos afectados (global): {sorted(vehs_global, key=lambda v: int(v.split('_')[-1]) if v.split('_')[-1].isdigit() else 0)}")
    print(f"desalineaciones espuria<->lane_vacio: {len(incoherencias)}"
          + (f" -> {incoherencias[:5]}" if incoherencias else " (equivalencia exacta)"))
    if not args.solo_censo:
        man_path = os.path.join(args.dst_root, "MANIFIESTO_LIMPIEZA.json")
        os.makedirs(args.dst_root, exist_ok=True)
        with open(man_path, "w") as f:
            json.dump({"regla": f"headingDeg < {UMBRAL_ESPURIO} -> {SENTINEL_EXACTO}; "
                                "resto de la linea intacto; originales no tocados",
                       "totales": dict(tot), "archivos": manifest}, f, indent=1)
        print(f"manifiesto: {man_path}")


if __name__ == "__main__":
    main()
