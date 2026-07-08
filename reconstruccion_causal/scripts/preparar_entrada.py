"""
preparar_entrada — physical copy of a Monte Carlo sweep CSV into inputs/,
renaming vehicleId v<N> -> veh_<N>.

Why: the collaborator's base player sorts crossing_times.csv with
int(veh.split("_")[1]) and crashes with 'v7' IDs. The shim is COSMETIC (just the
id), applied equally to the base run and the causal one for comparability, and
does not touch the original source (via_b_runtime/ stays intact).
"""
import argparse
import csv
import hashlib
import os
import re
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, help="CSV fuente del barrido Monte Carlo")
    ap.add_argument("--dst", required=True, help="ruta de la copia adaptada en inputs/")
    args = ap.parse_args()

    with open(args.src) as f:
        rows = list(csv.DictReader(f))
        fieldnames = list(rows[0].keys())

    pat = re.compile(r"^v(\d+)$")
    renamed = 0
    for r in rows:
        m = pat.match(r["vehicleId"])
        if m:
            r["vehicleId"] = f"veh_{m.group(1)}"
            renamed += 1

    os.makedirs(os.path.dirname(args.dst), exist_ok=True)
    with open(args.dst, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)

    src_md5 = hashlib.md5(open(args.src, "rb").read()).hexdigest()
    dst_md5 = hashlib.md5(open(args.dst, "rb").read()).hexdigest()
    print(f"filas={len(rows)} renombradas={renamed}")
    print(f"src {src_md5}  {args.src}")
    print(f"dst {dst_md5}  {args.dst}")
    if renamed != len(rows):
        print("AVISO: hubo IDs que no matchearon v<N>; revisar.", file=sys.stderr)


if __name__ == "__main__":
    main()
