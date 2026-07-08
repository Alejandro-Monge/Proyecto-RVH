"""SondaEmisor — anti-leak layer 2: invariants over the EMITTED PAYLOADS.

Audits the PRODUCT (the canonical JSONL log of the messages emitter), not the internal
state. Runs at CLOSE as an independent process (not inline, does not touch the player):

  python -m ws_emitter.sonda_emisor --log ws_log.jsonl --probe occ_probe.csv \
      --occupancy-config config/sites/rusia/occupancy.yaml [--sample K]

Exits with code 0 (PASS) or 2 (SondaError) — same policy as the causal guard.

INVARIANTS (MESSAGES profile; contract v3.0.0):
  1. `seq` strictly monotonic and WITHOUT GAPS, and grid |t_sim − seq·dt| ≤ TOL.
     "Without gaps" is a property of the CANONICAL LOG (it records every step; the
     network drop does not touch it): if the log were populated from the stream, this
     clause would no longer hold.
  2. `t_emit` ABSENT from the log (0 occurrences in the bytes).
  3. BIDIRECTIONAL ghost exclusion: per tick, set(entities.id) == set(eligible from the
     probe) — an emitted ghost AND an omitted eligible are both an emitter bug.
  4. Occupancy ORACLE: recompute occupancy_counts (the SINGLE implementation,
     occupancy_core) from the tick's probe and require EXACT equality with what was
     emitted — proves the occupancy used only that tick's state.
  5. `end`: present, last, `seq_final` == last seq, `sonda.status` present,
     `emit_errors == 0` (a log with emission errors is not complete truth). The
     COMPARISON between runs (layer 3) excludes `totals.dropped` (network-thread
     counter, not reconstruction), materialized in `canonical_digest()`.

Grid TOL (NOTHING of dt/2): the t_sim vs logical-clock divergence is MEASURED, not
estimated: the stage smoke test (`scripts/smoke_sumo_stage.py`) requires
|stage.sync(t_v) − t_v| < 0.002 s and observed 0 ms of accumulated drift over 5325
frames (docstring of `sumo/stage.py`; contract Section 3.2). TOL_TSIM_S = 0.002 inherits
THAT measured bound (the smoke PASS threshold, worst case of the stage's quantization).
dt/2 = 50 ms would have been 25× the observed divergence: an invariant almost incapable
of failing.

ORACLE via route (b): the input of the recompute is the player's EXISTING
`--occupancy-probe-out` record — written post-step from the SAME subscription cache of
the present tick (no access to other ticks or to messages), with byte-for-byte certified
passivity. WITHOUT the probe the run is NOT auditable for 3–4: the probe FAILS
explicitly, never a silent PASS.

VISION invariants — DECLARED, NOT implemented (no subject in the messages profile):
appearance ≥ cross_t; box extinction before the crossing; handoff window
[cross_t, appearance) with closed keys and no ETA; anchor consistent with lane.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import sys
from pathlib import Path

from ws_emitter.canonical import CONTRACT_VERSION, canonical_bytes

ROOT = Path(__file__).resolve().parents[1]

# MEASURED bound (stage smoke PASS threshold; see docstring). Do NOT derive from dt.
TOL_TSIM_S = 0.002

# end keys EXCLUDED from the between-runs comparison:
# network-thread counters, not reconstruction.
END_NETWORK_KEYS = ("dropped",)


class SondaError(AssertionError):
    """An emitter invariant broke (the run is NOT conformant)."""


# --------------------------------------------------------------------------- #
# Loading and structure
def load_log(log_path):
    raw = Path(log_path).read_bytes()
    lines = raw.splitlines()
    if not lines:
        raise SondaError("estructura: log vacío")
    msgs = []
    for k, line in enumerate(lines):
        try:
            d = json.loads(line)
        except ValueError as e:
            raise SondaError(f"estructura: línea {k} no es JSON: {e}") from e
        # every line must RE-CANONICALIZE to itself (canonical integrity)
        if canonical_bytes(d) != line + b"\n":
            raise SondaError(f"estructura: línea {k} no es canónica byte a byte")
        msgs.append(d)
    if msgs[0].get("type") != "hello":
        raise SondaError("estructura: la primera línea no es hello")
    if msgs[0].get("schema_version") != CONTRACT_VERSION:
        raise SondaError(
            f"estructura: schema_version {msgs[0].get('schema_version')!r} != "
            f"{CONTRACT_VERSION!r}")
    if msgs[-1].get("type") != "end":
        raise SondaError("estructura: la última línea no es end (invariante 5)")
    return raw, msgs


def check_no_temit(raw: bytes):
    n = raw.count(b"t_emit")
    if n:
        raise SondaError(f"invariante 2: t_emit aparece {n} veces en el log "
                         f"(debe ser 0; t_emit es solo del wire)")


def check_seq_grid(steps, dt: float):
    prev = None
    for s in steps:
        seq, t_sim = s["seq"], s["t_sim"]
        if prev is not None and seq != prev + 1:
            raise SondaError(f"invariante 1: seq {seq} tras {prev} — el log "
                             f"canónico debe ser monótono SIN huecos")
        prev = seq
        drift = abs(t_sim - seq * dt)
        if drift > TOL_TSIM_S:
            raise SondaError(
                f"invariante 1: rejilla rota en seq={seq}: |t_sim − seq·dt| = "
                f"{drift:.6f} s > TOL medida {TOL_TSIM_S} s (humo del stage)")


def check_end(msgs, steps):
    end = msgs[-1]
    last_seq = steps[-1]["seq"] if steps else -1
    if end.get("seq_final") != last_seq:
        raise SondaError(f"invariante 5: seq_final {end.get('seq_final')} != "
                         f"último seq {last_seq}")
    if "status" not in end.get("sonda", {}):
        raise SondaError("invariante 5: end sin sonda.status")
    if end.get("totals", {}).get("emit_errors", 0) != 0:
        raise SondaError(f"invariante 5: emit_errors = "
                         f"{end['totals']['emit_errors']} ≠ 0 — el log no es "
                         f"verdad completa")


# --------------------------------------------------------------------------- #
# Probe (present-frame-only record) and invariants 3-4
def load_probe(probe_path):
    """Player CSV: sim_time_s, veh, lane_sumo, var_speed, estado.
    Returns {t_str: {"elegibles": {veh: (lane, speed)}, "otros": {veh: estado}}}."""
    ticks: dict = {}
    with open(probe_path, newline="") as f:
        for row in csv.DictReader(f):
            slot = ticks.setdefault(row["sim_time_s"],
                                    {"elegibles": {}, "otros": {}})
            if row["estado"] == "elegible":
                slot["elegibles"][row["veh"]] = (row["lane_sumo"],
                                                 float(row["var_speed"]))
            else:
                slot["otros"][row["veh"]] = row["estado"]
    return ticks


def _tick_key(seq: int, dt: float) -> str:
    return f"{seq * dt:.1f}"      # same format the player writes


def check_ghosts(steps, probe, dt):
    for s in steps:
        key = _tick_key(s["seq"], dt)
        slot = probe.get(key, {"elegibles": {}, "otros": {}})
        emitted = {e["id"] for e in s["entities"]}
        eligible = set(slot["elegibles"])
        ghosts = emitted - eligible
        if ghosts:
            estados = {v: slot["otros"].get(v, "ausente_del_probe")
                       for v in ghosts}
            raise SondaError(f"invariante 3: tick {key} (seq {s['seq']}): "
                             f"FANTASMAS emitidos {estados}")
        missing = eligible - emitted
        if missing:
            raise SondaError(f"invariante 3: tick {key} (seq {s['seq']}): "
                             f"elegibles OMITIDOS {sorted(missing)}")


def _load_occ_cfg(occ_cfg_path):
    import yaml
    sys.path.insert(0, str(ROOT / "reconstruccion_causal" / "scripts"))
    import occupancy_core as occ_core   # SINGLE implementation, not a copy
    occ_cfg_path = os.path.abspath(occ_cfg_path)
    with open(occ_cfg_path) as f:
        cfg = yaml.safe_load(f)
    ref = cfg["lane_map_ref"]
    lm_path = ref if os.path.isabs(ref) else os.path.join(str(ROOT), ref)
    with open(lm_path) as f:
        approach_lanes = occ_core.load_approach_lanes(yaml.safe_load(f))
    return occ_core, approach_lanes, float(cfg["halting_speed_mps"])


def check_occupancy_oracle(steps, probe, dt, occ_cfg_path, sample=0):
    occ_core, approach_lanes, halt = _load_occ_cfg(occ_cfg_path)
    idx = range(len(steps))
    if sample and sample < len(steps):
        # uniform sampling that ALWAYS includes first and last
        stride = max(1, (len(steps) - 1) // max(1, sample - 1))
        idx = sorted(set(list(range(0, len(steps), stride)) + [len(steps) - 1]))
    checked = 0
    for k in idx:
        s = steps[k]
        key = _tick_key(s["seq"], dt)
        slot = probe.get(key, {"elegibles": {}, "otros": {}})
        states = [(v, lane, speed)
                  for v, (lane, speed) in sorted(slot["elegibles"].items())]
        expected = occ_core.occupancy_counts(states, approach_lanes, halt)
        exp_shape = {}
        for (brazo, carril), c in expected.items():
            exp_shape.setdefault(str(brazo), {})[str(carril)] = {
                "ocupacion": c["ocupacion"], "detenidos": c["detenidos"]}
        got = s["occupancy"]["per_lane"]
        if got != exp_shape:
            raise SondaError(
                f"invariante 4: tick {key} (seq {s['seq']}): ocupación emitida "
                f"≠ recomputada desde el probe del MISMO tick.\n  emitida:    "
                f"{got}\n  recomputada: {exp_shape}")
        checked += 1
    return checked


# --------------------------------------------------------------------------- #
# Helper for layer 3 (the end rule)
def canonical_digest(log_path, exclude_end_network=True) -> str:
    """BETWEEN-RUNS comparison md5: hello+steps as-is; from the end the network
    counters (END_NETWORK_KEYS) are excluded — thread state, not reconstruction."""
    h = hashlib.md5()
    lines = Path(log_path).read_bytes().splitlines()
    for line in lines:
        d = json.loads(line)
        if exclude_end_network and d.get("type") == "end":
            for k in END_NETWORK_KEYS:
                d.get("totals", {}).pop(k, None)
            h.update(canonical_bytes(d))
        else:
            h.update(line + b"\n")
    return h.hexdigest()


# --------------------------------------------------------------------------- #
def run_sonda(log_path, probe_path, occ_cfg_path, sample=0) -> dict:
    # invariant 2 FIRST and over raw bytes: a line with t_emit must
    # name its invariant, not die as a CanonicalError in the re-canonicalization
    check_no_temit(Path(log_path).read_bytes())
    raw, msgs = load_log(log_path)
    steps = [m for m in msgs if m["type"] == "step"]
    dt = float(msgs[0]["dt"])
    check_seq_grid(steps, dt)
    check_end(msgs, steps)
    if not probe_path or not os.path.exists(probe_path):
        raise SondaError(
            "corrida NO AUDITABLE: falta el probe de ocupación "
            "(--occupancy-probe-out del player). Los invariantes 3-4 no se "
            "verifican en silencio: relanzar la corrida auditable")
    probe = load_probe(probe_path)
    check_ghosts(steps, probe, dt)
    n_occ = check_occupancy_oracle(steps, probe, dt, occ_cfg_path, sample)
    return {"steps": len(steps), "occ_ticks_verificados": n_occ,
            "digest": canonical_digest(log_path)}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--log", required=True, help="log canónico JSONL del emisor")
    ap.add_argument("--probe", required=True,
                    help="CSV de --occupancy-probe-out (input del oráculo)")
    ap.add_argument("--occupancy-config", required=True,
                    help="occupancy.yaml del sitio (halting + lane_map_ref)")
    ap.add_argument("--sample", type=int, default=0,
                    help="ticks a muestrear en el oráculo (0 = todos)")
    args = ap.parse_args()
    try:
        rep = run_sonda(args.log, args.probe, args.occupancy_config, args.sample)
    except SondaError as e:
        print(f"[SondaEmisor] FAIL:\n{e}", file=sys.stderr)
        sys.exit(2)
    print(f"[SondaEmisor] PASS: {rep['steps']} steps; ocupación verificada en "
          f"{rep['occ_ticks_verificados']} ticks; digest {rep['digest'][:12]}…")
    sys.exit(0)


if __name__ == "__main__":
    main()
