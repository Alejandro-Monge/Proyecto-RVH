#!/usr/bin/env python3
"""Gate 2 (cross-run determinism) — fresh environment, healthy display.

Runs causal_player TWICE (on1, on2) over the SAME P30 sim with --shot-dir and
byte-compares the shadow shots. Config-driven: boundary / window / scheme come from
config/sites/rusia/demo_view.yaml (no hardcode). Reliable pattern: each pass opens ONE
long-lived sumo-gui and screenshots in a loop (like render_split) — NOT
launch/kill cycles. Between passes it checks that no zombie sumo-gui remains.

Usage: python3 _gate2_verify.py <scratch_dir>
"""
import hashlib
import os
import subprocess
import sys
import time

import yaml

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
CAUSAL = os.path.join(REPO, "reconstruccion_causal")
VIEW_YAML = os.path.join(REPO, "config/sites/rusia/demo_view.yaml")

SP = sys.argv[1]
os.makedirs(SP, exist_ok=True)

# --- config-driven: the demo view comes from the yaml, not the CLI ---
cfg = yaml.safe_load(open(VIEW_YAML))
boundary = ",".join(str(v) for v in cfg["boundary"])          # "-53.0,12.0,167.0,136.0"
scheme_file = os.path.join(REPO, "config/sites/rusia", cfg["scheme_file"])
print(f"[cfg] boundary={boundary}", flush=True)
print(f"[cfg] scheme_file={scheme_file}", flush=True)
print(f"[cfg] window_size={cfg['window_size']}", flush=True)


def stale_sumo():
    out = subprocess.run(["pgrep", "-a", "sumo"], capture_output=True, text=True).stdout
    return [l for l in out.splitlines() if l.strip()]


def run_on(tag):
    d = os.path.join(SP, tag)
    subprocess.run(["rm", "-rf", d])
    os.makedirs(os.path.join(d, "shots"))
    cmd = [
        "python3", "scripts/causal_player.py",
        "--csv", "inputs/cert/mc_5Hz_P30_000.csv",
        "--net", "inputs/intersection_rusia.net.xml",
        "--out", d,
        "--delay-s", "1.0", "--period-s", "0.2", "--interp", "linear",
        "--guard-mode", "assert", "--staging-mode", "park",
        "--gui", "--gui-settings-file", scheme_file,
        "--shot-dir", os.path.join(d, "shots"),
        # "=" form required: the boundary starts with '-' and argparse would take it
        # for a flag (--shot-boundary <val> fails with "expected one argument").
        f"--shot-boundary={boundary}",
        "--ws-log", os.path.join(d, "ws_log.jsonl"),
    ]
    env = dict(os.environ, SUMO_HOME="/usr/share/sumo", DISPLAY=":1")
    t0 = time.monotonic()
    with open(os.path.join(SP, f"{tag}.log"), "w") as logf:
        rc = subprocess.run(cmd, cwd=CAUSAL, env=env, stdout=logf,
                            stderr=subprocess.STDOUT).returncode
    dt = time.monotonic() - t0
    nshots = len(os.listdir(os.path.join(d, "shots")))
    print(f"[{tag}] rc={rc} shots={nshots} dt={dt:.1f}s", flush=True)
    return d


def md5(p):
    return hashlib.md5(open(p, "rb").read()).hexdigest()


print("=== PASE ON1 ===", flush=True)
d1 = run_on("on1")
st = stale_sumo()
print(f"[entre pases] sumo zombi: {st if st else 'ninguno'}", flush=True)
print("=== PASE ON2 ===", flush=True)
d2 = run_on("on2")

# --- GATE 2: byte-compare shadow shots ---
s1, s2 = os.path.join(d1, "shots"), os.path.join(d2, "shots")
f1, f2 = sorted(os.listdir(s1)), sorted(os.listdir(s2))
print("\n=== GATE 2 (determinismo cross-run) ===", flush=True)
print(f"count on1={len(f1)} on2={len(f2)} mismos-nombres={f1 == f2}", flush=True)
common = [f for f in f1 if f in set(f2)]
diff = [f for f in common if md5(os.path.join(s1, f)) != md5(os.path.join(s2, f))]
print(f"PNG byte-idénticos: {len(common) - len(diff)}/{len(common)} | difieren: {len(diff)}",
      flush=True)
if diff:
    print("primeros diferentes:", diff[:8], flush=True)
    # magnitude diagnosis (residual AA or a different view?)
    try:
        import cv2
        import numpy as np
        for f in diff[:3]:
            a = cv2.imread(os.path.join(s1, f)).astype(int)
            b = cv2.imread(os.path.join(s2, f)).astype(int)
            print(f"  {f}: maxdiff={int(np.abs(a - b).max())} "
                  f"px_dist={int((np.abs(a - b).sum(2) > 0).sum())}", flush=True)
    except Exception as e:
        print("  (sin diagnóstico cv2:", e, ")", flush=True)
print("GATE2_VERDE", len(diff) == 0 and len(common) > 0, flush=True)
print("DONE", flush=True)
