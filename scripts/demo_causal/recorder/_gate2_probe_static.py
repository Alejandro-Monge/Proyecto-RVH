#!/usr/bin/env python3
"""Decisive Gate 2 test: does the EMPTY NETWORK (no sim, no vehicles) render identically
across runs under the demo view? It isolates VIEW/render non-determinism from that of
dynamic content. Replicates the causal_player setup EXACTLY: scheme_file, 1148x646
window, setBoundary(rect from config) ONCE, gui.screenshot 1148x646.

Usage: python3 _gate2_probe_static.py <scratch_dir> <boundary "x0,y0,x1,y1"> <tag>
Takes ONE screenshot of the empty network (no stepping) and saves it as <tag>.png.
Launches sumo-gui ONCE, closes cleanly.
"""
import os
import sys
import time

import traci

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
NET = os.path.join(REPO, "reconstruccion_causal/inputs/intersection_rusia.net.xml")
SCHEME = os.path.join(REPO, "config/sites/rusia/sumo_demo_view.xml")

SP = sys.argv[1]
bnd = [float(v) for v in sys.argv[2].split(",")]
tag = sys.argv[3]
port = int(sys.argv[4])
out = os.path.join(SP, f"{tag}.png")

traci.start([
    "sumo-gui", "-n", NET,
    "--step-length", "0.1", "--begin", "0.20", "--end", "100000",
    "--time-to-teleport", "-1", "--collision.action", "warn",
    "--ignore-route-errors", "--no-step-log", "true",
    "--gui-settings-file", SCHEME, "--window-size", "1148,646",
], port=port)
traci.gui.setBoundary("View #0", *bnd)
# without inserting vehicles: just the network. A minimal step to force the first redraw
# (same paused mode as causal_player: the deferred redraw writes the screenshot).
traci.simulationStep(0.30)
traci.gui.screenshot("View #0", out, width=1148, height=646)
traci.simulationStep(0.40)
traci.simulationStep(0.50)
time.sleep(0.6)
traci.close()
time.sleep(1.0)
print(f"{tag}: escrito={os.path.exists(out)}", flush=True)
