"""Tests for the layer-3 detector (`ws_emitter/prefix_audit.py`) — no SUMO.

A deterministic TOY generator (messages → steps) in two variants: honest
(uses messages ≤ t+H) and mutant (looks beyond the horizon). The input is
truncated, logs are regenerated, and the comparator must: let the honest one
pass over the WHOLE window, catch the mutant naming tick+field, and respect the
window (a diff at t ≤ T_N−H is detected; the same diff outside the window does not count).

pytest tests/test_prefix_audit.py from the implementacion/ root.
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from ws_emitter.canonical import canonical_bytes  # noqa: E402
from ws_emitter.prefix_audit import (  # noqa: E402
    compare_prefix,
    prefix_lines,
    simtime_span,
    truncate_csv,
)

DT, H, T_END = 0.1, 1.0, 6.0
MSGS = [(round(0.3 * k + 0.05, 4), k + 1) for k in range(18)]   # (simTime, value)


# --------------------------------------------------------------------------- #
# Toy generator: deterministic messages -> steps reconstructor
def toy_log(path, msgs, horizon):
    """Step at t: value = sum of messages with ts <= t + horizon. The honest one
    uses horizon=H; a mutant uses horizon>H (looks beyond the legitimate one)."""
    lines = [canonical_bytes({"type": "hello", "schema_version": "3.0.0",
                              "dt": DT})]
    k = 0
    t = 0.0
    while t <= T_END + 1e-9:
        val = sum(v for ts, v in msgs if ts <= t + horizon + 1e-12)
        lines.append(canonical_bytes({"type": "step", "seq": k,
                                      "t_sim": round(t, 4),
                                      "guard": {"valor": val}}))
        k += 1
        t = round(t + DT, 4)
    lines.append(canonical_bytes({"type": "end", "seq_final": k - 1,
                                  "sonda": {"status": "PASS", "detail": ""},
                                  "guard_final": {}, "totals": {"steps": k}}))
    Path(path).write_bytes(b"".join(lines))


def _trunc_msgs(t_n):
    return [(ts, v) for ts, v in MSGS if ts <= t_n]


# --------------------------------------------------------------------------- #
def test_hello_excluye_solo_artifact_md5(tmp_path):
    """The truncated one's hello declares ITS artifact (different md5): it does not
    count as a leak. Any OTHER differing hello field DOES count."""
    import json
    ref, tr = tmp_path / "r.jsonl", tmp_path / "t.jsonl"
    toy_log(ref, MSGS, H)
    lines = [json.loads(l) for l in Path(ref).read_bytes().splitlines()]
    lines[0]["artifact_md5"] = {"mensajes_csv": "f" * 32}   # different input: OK
    Path(tr).write_bytes(b"".join(canonical_bytes(d) for d in lines))
    assert compare_prefix(str(ref), str(tr), 2.0) is None
    lines[0]["dt"] = 0.2                                    # this IS a diff
    Path(tr).write_bytes(b"".join(canonical_bytes(d) for d in lines))
    diff = compare_prefix(str(ref), str(tr), 2.0)
    assert diff is not None and diff["campos"] == ["dt"]


def test_truncado_determinista(tmp_path):
    csv = tmp_path / "in.csv"
    csv.write_text("rxTime,vehicleId,simTime,x\n"
                   "0.11,a,0.10,1.0\n0.21,b,0.20,2.0\n0.31,c,0.30,3.0\n")
    out1, out2 = tmp_path / "t1.csv", tmp_path / "t2.csv"
    k1 = truncate_csv(csv, 0.20, out1)
    k2 = truncate_csv(csv, 0.20, out2)
    assert k1 == k2 == 2
    assert out1.read_bytes() == out2.read_bytes()          # deterministic
    lines = out1.read_text().splitlines()
    assert lines[0] == "rxTime,vehicleId,simTime,x"        # header intact
    assert all(len(l.split(",")) == 4 for l in lines)      # no split rows
    assert simtime_span(csv) == (0.10, 0.30)
    # the cut is by simTime (column 3), NOT by rxTime: the row with
    # simTime=0.20 (rxTime=0.21 > 0.20) IS KEPT
    assert "0.21,b" in lines[1] or "0.21" in lines[2]


def test_honesto_pasa_en_toda_la_ventana(tmp_path):
    ref = tmp_path / "ref.jsonl"
    toy_log(ref, MSGS, H)
    for t_n in (2.0, 3.5, 5.0):
        tr = tmp_path / f"tr_{t_n}.jsonl"
        toy_log(tr, _trunc_msgs(t_n), H)
        assert compare_prefix(str(ref), str(tr), t_n - H) is None


def test_mutante_falla_nombrando_tick_y_campo(tmp_path):
    ref = tmp_path / "ref.jsonl"
    tr = tmp_path / "tr.jsonl"
    lookahead_extra = 3 * DT                     # looks 0.3 s BEYOND H
    t_n = 3.5
    toy_log(ref, MSGS, H + lookahead_extra)      # mutant: ref and trunc with the
    toy_log(tr, _trunc_msgs(t_n), H + lookahead_extra)   # same leak
    diff = compare_prefix(str(ref), str(tr), t_n - H)
    assert diff is not None
    assert diff["campos"] == ["guard.valor"]     # names the FIELD
    assert diff["seq"] is not None               # and the tick
    # the first diff falls inside the zone (T_N − H − extra, T_N − H]
    assert t_n - H - lookahead_extra < diff["t_sim"] <= t_n - H + 1e-9


def test_lookahead_dentro_de_H_no_se_detecta(tmp_path):
    """A lookahead ≤ H is a LEGITIMATE decision: the window must not flag it.
    (Asymmetry that validates where the window is placed.)"""
    ref = tmp_path / "ref.jsonl"
    tr = tmp_path / "tr.jsonl"
    t_n = 3.5
    toy_log(ref, MSGS, H - DT)                   # uses up to t + H − dt (legitimate)
    toy_log(tr, _trunc_msgs(t_n), H - DT)
    assert compare_prefix(str(ref), str(tr), t_n - H) is None


def test_ventana_correcta_borde(tmp_path):
    """A diff exactly at t = T_N − H (border INCLUDED) is detected; the same
    diff falls outside if the window ends one tick earlier."""
    ref = tmp_path / "ref.jsonl"
    tr = tmp_path / "tr.jsonl"
    t_n, t_borde = 3.5, 3.5 - H                  # border = 2.5
    toy_log(ref, MSGS, H)
    lines = []
    for raw in Path(ref).read_bytes().splitlines():
        import json
        d = json.loads(raw)
        if d.get("type") == "step" and abs(d["t_sim"] - t_borde) < 1e-9:
            d["guard"]["valor"] += 999           # diff ONLY at the border
        lines.append(canonical_bytes(d))
    Path(tr).write_bytes(b"".join(lines))
    diff = compare_prefix(str(ref), str(tr), t_n - H)
    assert diff is not None and abs(diff["t_sim"] - t_borde) < 1e-9
    assert compare_prefix(str(ref), str(tr), t_n - H - DT) is None  # outside: does not count
    # prefix_lines respects the window
    assert all(__import__("json").loads(l).get("t_sim", 0) <= t_n - H + 1e-9
               for l in prefix_lines(str(ref), t_n - H)
               if b'"step"' in l)
