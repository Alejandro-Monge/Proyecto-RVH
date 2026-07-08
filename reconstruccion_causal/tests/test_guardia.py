"""
Tests of the causality guard.

The central piece is the NEGATIVE TEST: a guard that never fires proves
nothing; here it is verified that it fires exactly when it should.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from guardia_causal import GuardedMessageStore, CausalityViolation  # noqa: E402


def mk_rows(ts, veh="veh_1"):
    return [{"vehicleId": veh, "simTime": str(t), "rxTime": str(t),
             "x": str(10.0 * i), "y": "0", "headingDeg": "0", "speed": "1",
             "acceleration": "0", "edgeId": "E1", "laneId": "E1_0",
             "vType": "car"} for i, t in enumerate(ts)]


BUF = 1.0


def test_ventana_permite_dentro_del_buffer():
    store = GuardedMessageStore(mk_rows([0.0, 0.5, 1.0]), BUF)
    view = store.slot_view("veh_1", 0.0)          # decision at instant t=0
    m = view.next_within_window(0.0)              # next after t=0
    assert m is not None and m.t == 0.5           # 0.5 <= 0 + 1.0: allowed
    assert store.stats["violations"] == 0


def test_ventana_oculta_fuera_del_buffer_sin_disparar():
    # the next message exists but falls outside the window: the view
    # returns None without exposing it and WITHOUT counting it as a violation
    # (equivalent to closing the window by clock live)
    store = GuardedMessageStore(mk_rows([0.0, 2.0]), BUF)
    view = store.slot_view("veh_1", 0.0)
    assert view.next_within_window(0.0) is None
    assert store.stats["violations"] == 0


def test_negativa_acceso_futuro_dispara():
    # unit NEGATIVE TEST: touching a message beyond t+buffer must
    # raise CausalityViolation in assert mode
    store = GuardedMessageStore(mk_rows([0.0, 5.0]), BUF)
    view = store.slot_view("veh_1", 0.0)
    with pytest.raises(CausalityViolation):
        view.peek_absolute(-1)                    # last message (t=5.0 > 1.0)
    assert store.stats["violations"] == 1


def test_negativa_uso_transitivo_dispara():
    # a state that embeds a future message also cannot be used to compose
    # the state of an earlier instant (transitive accounting)
    store = GuardedMessageStore(mk_rows([0.0]), BUF)
    with pytest.raises(CausalityViolation):
        store.check_state_use(max_msg_ts=3.0, t_state=0.0, ctx="test")


def test_modo_audit_cuenta_sin_abortar():
    store = GuardedMessageStore(mk_rows([0.0, 5.0]), BUF, mode="audit")
    view = store.slot_view("veh_1", 0.0)
    m = view.peek_absolute(-1)                    # violation counted
    assert m.t == 5.0
    assert store.stats["violations"] == 1
    assert store.guard_report()["status"] == "FAIL"


def test_entrega_respeta_reloj():
    store = GuardedMessageStore(mk_rows([0.0, 0.5, 2.0]), BUF)
    store.set_clock(1.0)
    assert [m.t for m in store.deliver_until(1.0)] == [0.0, 0.5]
    with pytest.raises(ValueError):
        store.deliver_until(2.0)                  # ahead of the clock
    with pytest.raises(ValueError):
        store.set_clock(0.5)                      # the clock does not go back
    store.set_clock(2.0)
    assert [m.t for m in store.deliver_until(2.0)] == [2.0]
    assert store.exhausted()


def test_max_lookahead_registrado():
    store = GuardedMessageStore(mk_rows([0.0, 0.9]), BUF)
    view = store.slot_view("veh_1", 0.0)
    view.next_within_window(0.0)
    assert abs(store.stats["max_lookahead_s"] - 0.9) < 1e-12
    assert store.guard_report()["status"] == "PASS"
