"""VISION layer 3 — emitter prefix audit (analog of the messages prefix_audit.py,
adapted). Falsifiable: the stream up to frames ≤ N does not depend on tracks > N.

DIFFERENCES from the messages version (with line references):
- AXIS: `frame_index` (the tracks CSV also has `timestamp_s`, the decoy — analog of
  simTime/rxTime; the reconstructor indexes by frame_index: reconnect_fragments.py:69,
  causal_core.py:125). Truncation is by frame_index.
- MULTI-ARTIFACT: build_veh_causal reads tracks AND events; truncating only tracks
  leaves the events inconsistent → the track_events are REGENERATED from the truncated
  tracks (production derive_track_events.py), so the precompute sees consistent input.
- H is NOT scalar: the box (present source) is binding (analytic H 0); the delay +
  retention make the vehicle the SAFEST (past-only). The audit's H is MEASURED (first
  divergence of the honest emitter), not assumed.

Reuse: compare_prefix/prefix_lines/_clean_hello from prefix_audit; canonical_digest from
sonda_emisor (the end rule). Emitter determinism: prerequisite.
"""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DT = None   # set at runtime from the site (frame_index -> t_sim)


def _dt():
    global DT
    if DT is None:
        sys.path.insert(0, str(ROOT))
        from digital_shadow.config.site import load_site_config
        DT = float(1 / load_site_config("rusia").fps_ref)
    return DT


# --------------------------------------------------------------------------- #
# Input-artifact truncation (frame_index axis) + event regeneration
def truncate_tracks(tracks_in, n_frame, out_csv) -> int:
    """header + rows with int(frame_index) <= n_frame. Filter by WHOLE LINE (never
    splits a row); deterministic (preserves the order)."""
    tracks_in, out_csv = Path(tracks_in), Path(out_csv)
    with open(tracks_in) as f:
        header = f.readline()
        col = header.rstrip("\n").split(",").index("frame_index")
        kept = 0
        with open(out_csv, "w") as o:
            o.write(header)
            for line in f:
                if not line.strip():
                    continue
                if int(float(line.rstrip("\n").split(",")[col])) <= n_frame:
                    o.write(line)
                    kept += 1
    return kept


def regenerate_events(tracks_trunc, out_events) -> None:
    """track_events from the truncated tracks (production derive_track_events.py):
    CONSISTENT input for build_veh_causal (same track_id, frames <= N)."""
    r = subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "derive_track_events.py"),
         "--site", "rusia", "--tracks", str(Path(tracks_trunc).resolve()),
         "--out", str(Path(out_events).resolve())],
        cwd=str(ROOT), capture_output=True, text=True, timeout=300)
    if r.returncode != 0:
        raise RuntimeError(f"derive_track_events falló:\n{r.stderr[-1500:]}")


def run_emit(tracks, events, out_log, frame_cap=None, mutant_k=0) -> None:
    """Headless emitter (vision_emit_headless) over given tracks+events → canonical log.
    mutant_k>0: negative control (looks K frames into the future)."""
    import os
    env = dict(os.environ, SUMO_HOME="/usr/share/sumo", DISPLAY=os.environ.get("DISPLAY", ":1"),
               PYTHONPATH=str(ROOT / "scripts"))
    cmd = [sys.executable, "-m", "demo_causal.vision_emit_headless",
           "--out", str(out_log), "--video", "tramo"]
    if tracks is not None:
        cmd += ["--tracks", str(Path(tracks).resolve()),
                "--events", str(Path(events).resolve())]
    if frame_cap is not None:
        cmd += ["--frame-cap", str(int(frame_cap))]
    if mutant_k:
        cmd += ["--mutant-k", str(int(mutant_k))]
    r = subprocess.run(cmd, cwd=str(ROOT / "scripts"), env=env,
                       capture_output=True, text=True, timeout=600)
    if r.returncode != 0:
        raise RuntimeError(f"vision_emit_headless falló:\n{r.stderr[-1500:]}")


# --------------------------------------------------------------------------- #
# First divergence: first seq where the step bytes differ
def _steps_by_seq(log_path):
    out = {}
    for line in Path(log_path).read_bytes().splitlines():
        d = json.loads(line)
        if d["type"] == "step":
            out[int(d["seq"])] = line
    return out


def first_divergence(ref_log, trunc_log):
    """(seq, t_sim) of the first step where ref and trunc differ, over the COMMON seq
    range. None if they match across the whole common range (with the range covered)."""
    ref, tr = _steps_by_seq(ref_log), _steps_by_seq(trunc_log)
    common = sorted(set(ref) & set(tr))
    for seq in common:
        if ref[seq] != tr[seq]:
            return {"seq": seq, "t_sim": round(seq * _dt(), 4)}
    return {"seq": None, "t_sim": None,
            "comun_hasta": (common[-1] if common else None)}


# --------------------------------------------------------------------------- #
# The audit: byte-identical prefix for frames ≤ N − H_FRAMES.
# H_FRAMES = reid_window (MECHANICAL BOUND of the precompute, not the measured horizon 0):
# re-id operates over 140 frames (buffer_frames); the conservative margin goes to that
# bound (same logic as messages: H = delay_s, the buffer bound, not the measured drift).
H_FRAMES = 140


def audit(fractions, workdir, h_frames=H_FRAMES, tracks_csv=None, mutant_k=0,
          ref_cap=None) -> dict:
    import pandas as pd

    from ws_emitter.prefix_audit import PrefixLeak, compare_prefix, prefix_lines
    workdir = Path(workdir)
    workdir.mkdir(parents=True, exist_ok=True)
    tracks_csv = Path(tracks_csv) if tracks_csv else (ROOT / "verify_out/tracks_rusia.csv")
    max_frame = int(pd.read_csv(tracks_csv, usecols=["frame_index"])["frame_index"].max())
    dt = _dt()

    ref_log = workdir / "ref" / "ws_log.jsonl"
    (workdir / "ref").mkdir(parents=True, exist_ok=True)
    run_emit(None, None, ref_log, frame_cap=ref_cap, mutant_k=mutant_k)   # ref (same mutant)
    n_ref = sum(1 for l in Path(ref_log).read_bytes().splitlines()
                if json.loads(l)["type"] == "step")

    results, fail = [], None
    for frac in fractions:
        n = int(round(frac * max_frame))
        tag = f"trunc_{int(frac * 100):02d}"
        (workdir / tag).mkdir(parents=True, exist_ok=True)
        tcsv, ecsv = workdir / f"{tag}.csv", workdir / f"{tag}_ev.csv"
        truncate_tracks(tracks_csv, n, tcsv)
        regenerate_events(tcsv, ecsv)
        tlog = workdir / tag / "ws_log.jsonl"
        run_emit(tcsv, ecsv, tlog, frame_cap=n + 200, mutant_k=mutant_k)
        t_max = (n - h_frames) * dt
        diff = compare_prefix(ref_log, tlog, t_max)          # excludes the hello's artifact_md5
        compared = len(prefix_lines(ref_log, t_max)) - 1     # without the hello
        results.append({"frac": frac, "N": n, "t_max": round(t_max, 4),
                        "steps_comparados": compared,
                        "cobertura": round(compared / n_ref, 4), "diff": diff})
        if diff is not None and fail is None:
            fail = (frac, diff)

    report = {"H_frames": h_frames, "H_s": round(h_frames * dt, 4),
              "cota": "reid_window (mecánica, no el horizonte medido 0)",
              "max_frame": max_frame, "steps_referencia": n_ref,
              "resultados": results,
              "cobertura_maxima": max(r["cobertura"] for r in results)}
    if fail is not None:
        frac, diff = fail
        raise PrefixLeak(f"FUGA dentro de ≤ N−{h_frames} (trunc {frac}): "
                         f"seq={diff['seq']} t_sim={diff['t_sim']} campos={diff['campos']}\n"
                         f"{json.dumps(report, indent=2)}")
    return report
