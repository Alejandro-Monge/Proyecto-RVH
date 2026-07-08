"""Tests for SondaEmisor (`ws_emitter/sonda_emisor.py`) — STEP 2.4.

The probe PASSES on a valid log and FAILS on injected corrupt logs, naming
invariant and tick — a probe that can never fail proves nothing. The happy
path is generated with the REAL 2.0→2.3 pipeline (MessagesEmitter over
fabricated DecidedTickViews), not with hand-written JSON.

pytest tests/test_sonda_emisor.py from the implementacion/ root.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from ws_emitter.canonical import canonical_bytes  # noqa: E402
from ws_emitter.emitter import MessagesEmitter  # noqa: E402
from ws_emitter.sonda_emisor import (  # noqa: E402
    TOL_TSIM_S,
    SondaError,
    canonical_digest,
    run_sonda,
)
from ws_emitter.view import VAR_ANGLE, VAR_POSITION, DecidedTickView  # noqa: E402

DT = 0.1
HELLO = {
    "shadow": "messages", "mode": "replay", "dt": DT, "site": "rusia",
    "frame_geometry": None, "net_md5": "a" * 32,
    "artifact_md5": {"mensajes_csv": "b" * 32}, "occ_cfg_md5": {},
    "guard_mode": "assert", "zone_coverage": None, "layers": ["sumo"],
    "vclass_dims": {"car": {"length_m": 5.0, "width_m": 1.8}},
    "occupancy_semantics": {"kind": "sumo_lanes", "halting_speed_mps": 0.1},
}
SEQS = (100, 101, 102)


def _view(seq):
    return DecidedTickView.from_messages_tick(
        seq=seq, t_sim=seq * DT, active={"veh_1", "veh_ghost"},
        parked={"veh_ghost"},
        last_applied={"veh_1": {"provenance": "OBSERVED", "held_frame": False},
                      "veh_ghost": {"provenance": "OBSERVED"}},
        applied_ids={"veh_1"},
        subs={"veh_1": {VAR_POSITION: (10.0, 20.0), VAR_ANGLE: 90.0},
              "veh_ghost": {VAR_POSITION: (1.0, 1.0), VAR_ANGLE: 0.0}},
        generation={"veh_1": 0}, assigned_type={"veh_1": "shadow_car"},
        occupancy_counts={("norte", 1): {"ocupacion": 1, "detenidos": 0}},
        guard_report={"mode": "assert", "accesses": 3, "violations": 0})


@pytest.fixture()
def artefactos(tmp_path):
    """Valid log (real 2.0→2.3 pipeline) + coherent probe + occupancy config."""
    log = tmp_path / "ws_log.jsonl"
    em = MessagesEmitter(log_path=str(log), hello_fields=HELLO)
    em.emit_hello()
    for s in SEQS:
        em.emit_step(_view(s))
    em.emit_end(guard_final={"violations": 0}, status="PASS")
    em.close()
    probe = tmp_path / "occ_probe.csv"
    rows = ["sim_time_s,veh,lane_sumo,var_speed,estado"]
    for s in SEQS:
        t = f"{s * DT:.1f}"
        rows.append(f"{t},veh_1,E1_0,3.0000,elegible")
        rows.append(f"{t},veh_ghost,E1_0,0.0000,parked")
    probe.write_text("\n".join(rows) + "\n")
    lane_map = tmp_path / "lane_map.yaml"
    lane_map.write_text(
        "arms:\n  norte:\n    entry:\n      edge: E1\n      lanes:\n        1: 0\n")
    occ_cfg = tmp_path / "occupancy.yaml"
    occ_cfg.write_text(
        f"halting_speed_mps: 0.1\nlane_map_ref: {lane_map}\n")
    return log, probe, occ_cfg


def _rewrite(log: Path, fn):
    """Mutates the log parsing each line and re-canonicalizing (still canonical)."""
    msgs = [json.loads(l) for l in log.read_bytes().splitlines()]
    msgs = fn(msgs)
    log.write_bytes(b"".join(canonical_bytes(m) for m in msgs))


def _expect_fail(artefactos, fragmento):
    log, probe, occ = artefactos
    with pytest.raises(SondaError) as ei:
        run_sonda(str(log), str(probe), str(occ))
    assert fragmento in str(ei.value), str(ei.value)
    return str(ei.value)


# --------------------------------------------------------------------------- #
def test_pass_sobre_log_valido(artefactos):
    log, probe, occ = artefactos
    rep = run_sonda(str(log), str(probe), str(occ))
    assert rep["steps"] == 3 and rep["occ_ticks_verificados"] == 3


def test_falla_seq_duplicado(artefactos):
    log, _, _ = artefactos

    def fn(msgs):
        msgs[2]["seq"] = msgs[1]["seq"]          # duplicates the seq of step 2
        msgs[2]["t_sim"] = msgs[1]["t_sim"]
        return msgs
    _rewrite(log, fn)
    _expect_fail(artefactos, "invariante 1")


def test_falla_seq_con_hueco(artefactos):
    log, _, _ = artefactos

    def fn(msgs):
        return [m for m in msgs if not (m["type"] == "step" and m["seq"] == 101)]
    _rewrite(log, fn)
    msg = _expect_fail(artefactos, "invariante 1")
    assert "102" in msg and "100" in msg          # names the seqs of the gap


def test_tolerancia_muerde_en_el_margen(artefactos):
    """The measured TOL (2 ms, stage smoke) bites JUST above the bound,
    and does not fire inside it — it is not a decorative tolerance."""
    log, probe, occ = artefactos

    def desplaza(delta):
        def fn(msgs):
            for m in msgs:
                if m["type"] == "step" and m["seq"] == 101:
                    m["t_sim"] = 101 * DT + delta
            return msgs
        return fn

    _rewrite(log, desplaza(0.001))                # INSIDE the bound: passes
    run_sonda(str(log), str(probe), str(occ))
    _rewrite(log, desplaza(0.0025))               # just ABOVE: fails
    msg = _expect_fail(artefactos, "invariante 1: rejilla")
    assert "seq=101" in msg
    assert f"{TOL_TSIM_S}" in msg                 # the message cites the measured bound


def test_falla_t_emit_colado(artefactos):
    log, _, _ = artefactos
    with open(log, "ab") as f:
        f.write(b'{"t_emit":1.0,"type":"x"}\n')
    _expect_fail(artefactos, "invariante 2")


def test_falla_fantasma_emitido(artefactos):
    log, _, _ = artefactos

    def fn(msgs):
        for m in msgs:
            if m["type"] == "step" and m["seq"] == 101:
                m["entities"].append({"id": "veh_ghost", "source": "sumo",
                                      "provenance": "OBSERVED",
                                      "pos_plane": [1.0, 1.0], "heading": 0.0,
                                      "vclass": "car", "generation": 0})
        return msgs
    _rewrite(log, fn)
    msg = _expect_fail(artefactos, "invariante 3")
    assert "FANTASMAS" in msg and "veh_ghost" in msg and "parked" in msg


def test_falla_elegible_omitido(artefactos):
    log, _, _ = artefactos

    def fn(msgs):
        for m in msgs:
            if m["type"] == "step" and m["seq"] == 102:
                m["entities"] = []
        return msgs
    _rewrite(log, fn)
    msg = _expect_fail(artefactos, "invariante 3")
    assert "OMITIDOS" in msg and "veh_1" in msg and "10.2" in msg


def test_falla_ocupacion_alterada(artefactos):
    log, _, _ = artefactos

    def fn(msgs):
        for m in msgs:
            if m["type"] == "step" and m["seq"] == 101:
                m["occupancy"]["per_lane"]["norte"]["1"]["ocupacion"] = 2
        return msgs
    _rewrite(log, fn)
    msg = _expect_fail(artefactos, "invariante 4")
    assert "10.1" in msg                          # names the tick


def test_falla_sin_end_y_emit_errors(artefactos):
    log, _, _ = artefactos
    _rewrite(log, lambda msgs: msgs[:-1])         # no end
    _expect_fail(artefactos, "última línea no es end")
    # end with emit_errors > 0
    log2, probe, occ = artefactos
    em = MessagesEmitter(log_path=str(log2), hello_fields=HELLO)
    em.emit_hello()
    for s in SEQS:
        em.emit_step(_view(s))
    em.emit_end(guard_final={"violations": 0}, status="PASS")
    em.close()

    def fn(msgs):
        msgs[-1]["totals"]["emit_errors"] = 1
        return msgs
    _rewrite(log2, fn)
    _expect_fail(artefactos, "invariante 5")


def test_probe_ausente_no_es_pass_silencioso(artefactos):
    log, probe, occ = artefactos
    with pytest.raises(SondaError) as ei:
        run_sonda(str(log), str(probe) + ".no_existe", str(occ))
    assert "NO AUDITABLE" in str(ei.value)


def test_canonical_digest_regla_del_end(artefactos):
    """Rule 2.3: two runs that differ ONLY in end.totals.dropped share the
    digest; a difference in a step changes it."""
    log, _, _ = artefactos
    other = log.parent / "otro.jsonl"
    other.write_bytes(log.read_bytes())

    def mas_drops(msgs):
        msgs[-1]["totals"]["dropped"] = 999
        return msgs
    _rewrite(other, mas_drops)
    assert canonical_digest(str(log)) == canonical_digest(str(other))
    assert (canonical_digest(str(log), exclude_end_network=False)
            != canonical_digest(str(other), exclude_end_network=False))

    def toca_step(msgs):
        for m in msgs:
            if m["type"] == "step" and m["seq"] == 100:
                m["t_sim"] = 10.0001
        return msgs
    _rewrite(other, toca_step)
    assert canonical_digest(str(log)) != canonical_digest(str(other))
