"""
manifiesto_denso — md5, counts and guard status per dense file.

Walks resultados_densos/, cross-references each dense file with its control
summary.json (_ctl/) and writes MANIFIESTO_DENSO.json at the root of
resultados_densos/.
"""
import csv
import gzip
import hashlib
import json
import os
import sys

ROOT = sys.argv[1] if len(sys.argv) > 1 else \
    "../via_b_runtime/RusiaV2IViaB/results/resultados_densos"


def md5(path):
    h = hashlib.md5()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


manifest, tot = {}, {"archivos": 0, "filas": 0, "bytes_gz": 0, "guardia_no_pass": []}
for dirpath, _, files in os.walk(ROOT):
    if "_ctl" in dirpath:
        continue
    for fn in sorted(files):
        if not fn.endswith(".csv.gz"):
            continue
        p = os.path.join(dirpath, fn)
        rel = os.path.relpath(p, ROOT)
        with gzip.open(p, "rt") as f:
            rd = csv.reader(f)
            next(rd)
            n = 0
            vehs = set()
            for row in rd:
                n += 1
                vehs.add(row[1])
        ctl = os.path.join(ROOT, "_ctl",
                           rel.replace("/", "_").replace(".csv.gz", ""),
                           "results", "summary.json")
        guardia = ""
        if os.path.exists(ctl):
            s = json.load(open(ctl))
            guardia = s["guardia"]["status"]
            if guardia != "PASS":
                tot["guardia_no_pass"].append(rel)
        manifest[rel] = {"md5": md5(p), "filas": n, "vehiculos": len(vehs),
                         "bytes_gz": os.path.getsize(p), "guardia": guardia}
        tot["archivos"] += 1
        tot["filas"] += n
        tot["bytes_gz"] += os.path.getsize(p)

out = os.path.join(ROOT, "MANIFIESTO_DENSO.json")
with open(out, "w") as f:
    json.dump({"contrato": ["sim_time_s", "vehicle_id", "x", "y", "lane_sumo",
                            "edge_sumo", "lane_state", "speed", "headingDeg",
                            "provenance", "sim_time_csv", "generation"],
               "resolucion_s": 0.1, "totales": tot, "archivos": manifest},
              f, indent=1)
print(f"{out}: {tot['archivos']} archivos, {tot['filas']} filas, "
      f"{tot['bytes_gz']/1e6:.1f} MB gz, guardia_no_pass={tot['guardia_no_pass']}")
