"""PREFIX audit — anti-leak layer 3 of the messages emitter.

The FALSIFIABLE proof that the stream is invariant to truncating the future (the
"works live" argument): the INPUT artifact is truncated at T_N, re-run, and every step
with t_sim ≤ T_N − H must be BYTE-IDENTICAL to the one from the full run. A diff inside
the window = dependence of an early payload on data later than its horizon = LEAK,
flagged with seq, t_sim and field. It touches neither the player nor the emitter: it
runs them as subprocesses and consumes logs.

TIME AXIS:
the guard decides future/past over **simTime**, not rxTime — `Msg.t =
float(row["simTime"])` (guardia_causal.py:55; rxTime stays in `self.rx` and does not
govern), delivery orders by `m.t` ("= simTime, instantaneous emission") and `_record`
compares `msg_t` (simTime) against `t_decision + buffer`. Truncation cuts by simTime:
same axis as the H bound, and the physically meaningful cut ("the source stops emitting
at T_N"). Truncating by rxTime would have misaligned the window ~0.2 ms at the boundary
(undetectable false PASS/FAIL).

H — LEGITIMATE DECISION HORIZON = delay_s (guard buffer), a single addend, a bound
ENFORCED BY ASSERT in every run (not assumed):
  - buffer: `GuardedMessageStore(rows, buffer_s=args.delay_s)`
    (causal_player.py:158; help --delay-s: "the decision buffer is the same value");
    violation if `msg_t > t_decision + buffer` (guardia_causal.py: 123-134). Measured
    evidence: max_lookahead_s = 0.4471 < 1.0.
  - Discarded with a source (they do not add up): delivery by clock (stricter,
    guardia_causal.py:96-102); invariant of the shown state max_msg_ts ≤ t
    (causal_player.py check_state_use); slot window half=period/2 WITHIN slot_view
    watched (causal_core.py:173-181); timeout looks at PAST silence
    (causal_core.py:97); route extension queries the static network.

Deterministic TRUNCATION: header + every row with float(simTime) ≤ T_N — filter by
WHOLE LINE (never splits a row). T_N = quantiles of the simTime range.

Comparison: hello + steps with t_sim ≤ T_N − H, byte by byte, same count. The `end` is
left out by construction of the prefix; for a full log the end rule lives in
sonda_emisor.canonical_digest. Exit 0 (PASS) / 2 (FAIL).

Usage:
  python -m ws_emitter.prefix_audit --csv mc.csv --net red.net.xml \
      --period-s 0.5 --delay-s 1.0 --workdir out/ \
      [--fractions 0.25,0.5,0.75,0.9] [--mutant-k 11]
--mutant-k: NEGATIVE CONTROL (see _mutant_player.py): runs the player with the mutant
emitter that looks K ticks into the future; the audit MUST fail.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PLAYER = ROOT / "reconstruccion_causal" / "scripts" / "causal_player.py"


class PrefixLeak(AssertionError):
    """Diff inside the comparison window: leak (or loss) detected."""


# --------------------------------------------------------------------------- #
# Input-artifact truncation (simTime axis)
def simtime_span(csv_path):
    lo = hi = None
    with open(csv_path) as f:
        header = f.readline().rstrip("\n").split(",")
        col = header.index("simTime")
        for line in f:
            t = float(line.split(",")[col])
            lo = t if lo is None or t < lo else lo
            hi = t if hi is None or t > hi else hi
    return lo, hi


def truncate_csv(csv_in, t_max, csv_out) -> int:
    """header + rows with float(simTime) <= t_max. Filter by whole line: impossible to
    split a row. Deterministic (same input -> same bytes)."""
    kept = 0
    with open(csv_in) as fi, open(csv_out, "w") as fo:
        header = fi.readline()
        fo.write(header)
        col = header.rstrip("\n").split(",").index("simTime")
        for line in fi:
            if float(line.split(",")[col]) <= t_max + 1e-12:
                fo.write(line)
                kept += 1
    return kept


# --------------------------------------------------------------------------- #
# Prefix comparison
def prefix_lines(log_path, t_max):
    """hello + steps with t_sim <= t_max (line bytes, as-is from the log)."""
    out = []
    for line in Path(log_path).read_bytes().splitlines():
        d = json.loads(line)
        if d["type"] == "hello":
            out.append(line)
        elif d["type"] == "step" and d["t_sim"] <= t_max + 1e-9:
            out.append(line)
    return out


def _diff_fields(a: dict, b: dict, path=""):
    """Field paths with a different value between two dicts (recursive)."""
    diffs = []
    for k in sorted(set(a) | set(b)):
        p = f"{path}.{k}" if path else k
        if k not in a or k not in b:
            diffs.append(p + " (ausente en un lado)")
        elif isinstance(a[k], dict) and isinstance(b[k], dict):
            diffs.extend(_diff_fields(a[k], b[k], p))
        elif a[k] != b[k]:
            diffs.append(p)
    return diffs


# hello key EXCLUDED from the comparison (rule analogous to end.dropped):
# artifact_md5 pins THE INPUT of each run, and the truncation experiment consists
# precisely of giving it a different input — its difference is the experiment design,
# not a leak. EVERYTHING else in the hello must match.
HELLO_INPUT_KEYS = ("artifact_md5",)


def _clean_hello(line: bytes) -> bytes:
    from ws_emitter.canonical import canonical_bytes as _cb
    d = json.loads(line)
    for k in HELLO_INPUT_KEYS:
        d.pop(k, None)
    return _cb(d)


def compare_prefix(ref_log, trunc_log, t_max):
    """None if the prefix is byte-identical (hello without its input keys); otherwise a
    dict with seq/t_sim/fields."""
    ref = prefix_lines(ref_log, t_max)
    tr = prefix_lines(trunc_log, t_max)
    for k, (lr, lt) in enumerate(zip(ref, tr)):
        if k == 0:
            lr, lt = _clean_hello(lr), _clean_hello(lt)
        if lr != lt:
            dr, dt_ = json.loads(lr), json.loads(lt)
            return {"linea": k, "seq": dr.get("seq"), "t_sim": dr.get("t_sim"),
                    "campos": _diff_fields(dr, dt_)}
    if len(ref) != len(tr):
        return {"linea": min(len(ref), len(tr)), "seq": None, "t_sim": None,
                "campos": [f"conteo de steps en ventana: ref={len(ref)} "
                           f"trunc={len(tr)}"]}
    return None


# --------------------------------------------------------------------------- #
# Orchestration
def run_player(csv_path, net, out_dir, period_s, delay_s, ws_log,
               mutant_k=0):
    os.makedirs(out_dir, exist_ok=True)
    if mutant_k:
        cmd = [sys.executable, "-m", "ws_emitter._mutant_player",
               "--mutant-k", str(mutant_k)]
    else:
        cmd = [sys.executable, str(PLAYER)]
    cmd += ["--csv", str(csv_path), "--net", str(net), "--out", str(out_dir),
            "--period-s", str(period_s), "--delay-s", str(delay_s),
            "--occupancy-out", str(Path(out_dir) / "occ.csv"),
            "--ws-log", str(ws_log)]
    r = subprocess.run(cmd, capture_output=True, text=True, cwd=str(ROOT))
    if r.returncode != 0:
        raise RuntimeError(f"player fallo ({r.returncode}):\n{r.stderr[-2000:]}")


def audit(csv_path, net, period_s, delay_s, workdir, fractions,
          mutant_k=0) -> dict:
    workdir = Path(workdir)
    workdir.mkdir(parents=True, exist_ok=True)
    H = float(delay_s)   # horizon = guard buffer (see docstring)
    lo, hi = simtime_span(csv_path)

    ref_log = workdir / "ref" / "ws_log.jsonl"
    run_player(csv_path, net, workdir / "ref", period_s, delay_s, ref_log,
               mutant_k)
    n_ref_steps = sum(1 for l in Path(ref_log).read_bytes().splitlines()
                      if json.loads(l)["type"] == "step")

    results, fail = [], None
    for frac in fractions:
        t_n = lo + frac * (hi - lo)
        tag = f"trunc_{int(frac * 100):02d}"
        csv_t = workdir / f"{tag}.csv"
        kept = truncate_csv(csv_path, t_n, csv_t)
        tlog = workdir / tag / "ws_log.jsonl"
        run_player(csv_t, net, workdir / tag, period_s, delay_s, tlog, mutant_k)
        t_max = t_n - H
        diff = compare_prefix(ref_log, tlog, t_max)
        compared = len(prefix_lines(ref_log, t_max)) - 1   # without the hello
        results.append({"frac": frac, "T_N": round(t_n, 4),
                        "t_max": round(t_max, 4), "filas_csv": kept,
                        "steps_comparados": compared,
                        "cobertura": round(compared / n_ref_steps, 4),
                        "diff": diff})
        if diff is not None and fail is None:
            fail = (frac, diff)

    report = {"H_s": H, "simtime_span": [round(lo, 4), round(hi, 4)],
              "steps_referencia": n_ref_steps, "mutant_k": mutant_k or None,
              "resultados": results,
              "cobertura_maxima": max(r["cobertura"] for r in results)}
    if fail is not None:
        frac, diff = fail
        raise PrefixLeak(
            f"FUGA/perdida dentro de la ventana (trunc {frac}): seq={diff['seq']} "
            f"t_sim={diff['t_sim']} campos={diff['campos']}\n"
            f"reporte: {json.dumps(report, indent=2)}")
    return report


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--csv", required=True)
    ap.add_argument("--net", required=True)
    ap.add_argument("--period-s", type=float, required=True)
    ap.add_argument("--delay-s", type=float, default=1.0)
    ap.add_argument("--workdir", required=True)
    ap.add_argument("--fractions", default="0.25,0.5,0.75,0.9")
    ap.add_argument("--mutant-k", type=int, default=0,
                    help="CONTROL NEGATIVO: emisor mutante con lookahead de K "
                         "ticks (K*TICK debe exceder H para ser detectable)")
    args = ap.parse_args()
    fracs = [float(x) for x in args.fractions.split(",")]
    try:
        rep = audit(args.csv, args.net, args.period_s, args.delay_s,
                    args.workdir, fracs, args.mutant_k)
    except PrefixLeak as e:
        print(f"[prefix_audit] FAIL:\n{e}", file=sys.stderr)
        sys.exit(2)
    print("[prefix_audit] PASS:")
    print(json.dumps(rep, indent=2))
    sys.exit(0)


if __name__ == "__main__":
    main()
