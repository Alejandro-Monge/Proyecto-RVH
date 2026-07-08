"""Piece 1 (vision emitter) — causal rule of the HANDOFF registry.

Verifies the EXACT rule (only observed events; never delay/cross_t/inst_t):
it is born from boxes_prev∖boxes_now (extinguished box) with vid not alive; it dies
from vid∈live (success) / box reappears (cancellation) / handoff_max_hold_s (presentation cap).

.venv/bin/python -m pytest tests/test_vision_handoff.py -v
"""
from __future__ import annotations

import inspect
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "scripts" / "demo_causal"))

import run_harness_causal as rhc  # noqa: E402


def mk_box(tid, cx=1.0, cy=2.0, cls=2):
    return rhc.BoxState(tid, cx, cy, 0.0, "lane", 1, 4.5, 2.0, cls, "OBSERVED")


def test_nace_de_extincion_con_last_pos_observada():
    r = rhc.HandoffRegistry({10: 10}, max_hold_s=2.0)
    assert r.update(0.0, {10: mk_box(10, cx=5.0, cy=6.0)}, set()) == []   # box present
    marks = r.update(0.1, {}, set())                                      # extinguished box
    assert len(marks) == 1
    m = marks[0]
    assert (m.id, m.source, m.prov) == ("v10", "handoff", "HELD")
    assert m.last_pos == (5.0, 6.0) and m.last_t == 0.0 and m.vclass == "car"


def test_muere_cuando_vid_aparece_en_live():
    r = rhc.HandoffRegistry({10: 10}, 2.0)
    r.update(0.0, {10: mk_box(10)}, set())
    assert len(r.update(0.1, {}, set())) == 1          # is born
    assert r.update(0.2, {}, {"v10"}) == []            # success: vid took over


def test_muere_cuando_la_caja_reaparece():
    r = rhc.HandoffRegistry({10: 10}, 2.0)
    r.update(0.0, {10: mk_box(10)}, set())
    assert len(r.update(0.1, {}, set())) == 1          # is born
    assert r.update(0.2, {10: mk_box(10)}, set()) == []  # cancellation (transient extinction)


def test_muere_por_tope_de_presentacion():
    r = rhc.HandoffRegistry({10: 10}, max_hold_s=0.5)
    r.update(0.0, {10: mk_box(10)}, set())
    r.update(0.1, {}, set())                            # born at t=0.1
    assert len(r.update(0.5, {}, set())) == 1           # 0.4 ≤ 0.5: still there
    assert r.update(0.75, {}, set()) == []              # 0.65 > 0.5: removed


def test_no_nace_si_el_vid_ya_esta_vivo():
    r = rhc.HandoffRegistry({10: 10}, 2.0)
    r.update(0.0, {10: mk_box(10)}, {"v10"})
    assert r.update(0.1, {}, {"v10"}) == []             # no bridge to draw


def test_tid_root_colapsa_fragmentos_del_mismo_root():
    r = rhc.HandoffRegistry({10: 10, 11: 10}, 2.0)     # two tids, one root
    r.update(0.0, {10: mk_box(10)}, set())             # root 10 via tid 10
    assert r.update(0.1, {11: mk_box(11)}, set()) == []  # via tid 11 (re-id): still present
    marks = r.update(0.2, {}, set())                   # now the root does extinguish
    assert len(marks) == 1 and marks[0].id == "v10"


def test_estructural_no_alcanza_futuro():
    # the registry ONLY takes tid_root + max_hold_s; it cannot consult
    # delay/cross_t/inst_t (they are not in its signature)
    params = set(inspect.signature(rhc.HandoffRegistry.__init__).parameters) - {"self"}
    assert params == {"tid_root", "max_hold_s"}
