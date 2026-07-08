#!/usr/bin/env python3
"""Diagnosis of the Gate 2 drift: WHERE (frames) and WHAT (region) differs.

Goal: distinguish VIEWPORT non-determinism (which would affect the whole frame,
including the static network) from DYNAMIC CONTENT non-determinism (vehicles:
localized diffs that move with the traffic). Cross-references the entity count of the
ws_log at each sim-ms.
"""
import hashlib
import json
import os
import sys

import cv2
import numpy as np

SP = sys.argv[1]
s1 = os.path.join(SP, "on1/shots")
s2 = os.path.join(SP, "on2/shots")


def md5(p):
    return hashlib.md5(open(p, "rb").read()).hexdigest()


files = sorted(os.listdir(s1))
diff = [f for f in files if md5(os.path.join(s1, f)) != md5(os.path.join(s2, f))]
ident = [f for f in files if f not in set(diff)]


def ms(f):  # sh_000000300.png -> 300
    return int(f.split("_")[1].split(".")[0])


diff_ms = sorted(ms(f) for f in diff)
ident_ms = sorted(ms(f) for f in ident)
print(f"total={len(files)} idénticos={len(ident)} difieren={len(diff)}")
print(f"rango sim-ms difieren: [{diff_ms[0]}..{diff_ms[-1]}]")
print(f"rango sim-ms idénticos: [{ident_ms[0]}..{ident_ms[-1]}]")

# are the differing ones contiguous or scattered? count gaps of identical frames between diffs
dset = set(diff_ms)
first_diff = diff_ms[0]
# are there identical frames AFTER the first diff? (mix → intermittent dynamic content)
ident_after = [m for m in ident_ms if m > first_diff]
print(f"primer frame que difiere: {first_diff} ms")
print(f"idénticos DESPUÉS del primer diff: {len(ident_after)} "
      f"(si >0: la deriva es intermitente, no un bloque inicial)")

# --- WHAT differs: bounding box + centroid of the diff per frame, and magnitude ---
# Load positions/count from the ws_log if it exists (entity count per t_sim)
ws1 = os.path.join(SP, "on1/ws_log.jsonl")
ent_by_ms = {}
if os.path.exists(ws1):
    for line in open(ws1):
        try:
            o = json.loads(line)
        except Exception:
            continue
        if o.get("type") == "step" or "t_sim" in o:
            t = o.get("t_sim")
            n = len(o.get("active", []) or o.get("entities", []) or [])
            if t is not None:
                ent_by_ms[int(round(t * 1000))] = n

print("\n=== muestreo de frames que difieren (bbox del diff, magnitud) ===")
print(f"{'sim_ms':>8} {'px_dist':>8} {'maxdiff':>7} {'bbox (x0,y0,x1,y1)':>26} {'#ent':>5}")
sample = diff_ms[:: max(1, len(diff_ms) // 20)][:20]
for m in sample:
    f = f"sh_{m:09d}.png"
    a = cv2.imread(os.path.join(s1, f)).astype(int)
    b = cv2.imread(os.path.join(s2, f)).astype(int)
    d = np.abs(a - b).sum(2)
    mask = d > 0
    ys, xs = np.where(mask)
    bbox = (int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())) if len(xs) else None
    ent = ent_by_ms.get(m, "?")
    print(f"{m:>8} {int(mask.sum()):>8} {int(np.abs(a - b).max()):>7} {str(bbox):>26} {str(ent):>5}")

# --- save heatmaps of 3 representative frames for visual inspection ---
out = os.path.join(SP, "diag")
os.makedirs(out, exist_ok=True)
reps = [diff_ms[0], diff_ms[len(diff_ms) // 2], diff_ms[-1]]
for m in reps:
    f = f"sh_{m:09d}.png"
    a = cv2.imread(os.path.join(s1, f))
    b = cv2.imread(os.path.join(s2, f))
    d = np.abs(a.astype(int) - b.astype(int)).sum(2)
    heat = (np.clip(d, 0, 255)).astype(np.uint8)
    heat = cv2.applyColorMap(heat, cv2.COLORMAP_JET)
    heat[d == 0] = 0
    over = cv2.addWeighted(a, 0.5, heat, 0.5, 0)
    cv2.imwrite(os.path.join(out, f"diff_{m:09d}_A.png"), a)
    cv2.imwrite(os.path.join(out, f"diff_{m:09d}_overlay.png"), over)
print(f"\nheatmaps guardados en {out} (frames {reps})")
