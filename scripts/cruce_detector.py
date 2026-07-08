"""cruce_detector — COMMITTED CROSSING detector from the shared protocol Section 2
(gt_mensajes_sim.md): nose, net stop line, 3 m hysteresis, by TRANSITION.

SINGLE implementation (single-source-of-truth doctrine): factored VERBATIM from
generar_gt_mensajes_sim.py and consumed by
(a) the message GT generator and (b) the event adapter of the message shadow
(matching_mensajes.py). If GT and shadow used different detectors,
the metric would measure the detector difference, not shadow vs reality.
Re-verified when factoring: the regenerated GT is byte-identical
(be012d86 lane / 3785aecc occupancy).

Pure functions: no I/O, no state; `linea` receives the net as an argument.
"""
import numpy as np


def linea(NET, eid, extremo):
    """Transverse line of the edge: extremo='fin' (stop line, J5 side of the approach) or
    'inicio' (start of the departure, J5 side)."""
    lanes = NET.getEdge(eid).getLanes()
    k = -1 if extremo == "fin" else 0
    a = np.asarray(lanes[0].getShape(), float)[k]
    b = np.asarray(lanes[-1].getShape(), float)[k]
    if np.hypot(*(b - a)) < 0.5:
        u0 = np.asarray(lanes[0].getShape(), float)
        d = u0[-1] - u0[0]
        n = np.array([-d[1], d[0]]) / max(np.hypot(*d), 1e-9)
        b = a + 3.2 * n
    return a - 0.8 * (b - a), b + 0.8 * (b - a)


def cruce(traza, tiempos, la, lb, hacia, histeresis=3.0):
    """COMMITTED CROSSING: first crossing of the line after which the point advances
    >= hysteresis meters without going back behind it. Without hysteresis, the nose of a
    queue head (the clamp leaves it grazing the line, ±0.5-1.3 m) triggers the detector 30-100 s
    before the real crossing — CONTRACT point: the message shadow must use
    the same hysteresis. Returns (t, censored)."""
    nrm = np.array([-(lb - la)[1], (lb - la)[0]])
    nrm = nrm / max(np.hypot(*nrm), 1e-9)
    if (hacia - la) @ nrm < 0:
        nrm = -nrm
    lado = (traza - la) @ nrm
    if lado[0] > histeresis:
        return None, True
    i = 1
    while i < len(traza):
        if lado[i] > 0 >= lado[i - 1]:
            j = i
            comprometido = False
            while j < len(traza) and lado[j] > 0:
                if lado[j] >= histeresis:
                    comprometido = True
                    break
                j += 1
            if comprometido:
                f = -lado[i - 1] / max(lado[i] - lado[i - 1], 1e-9)
                return float(tiempos[i - 1] + f * (tiempos[i] - tiempos[i - 1])), False
            i = j
        i += 1
    return None, lado[0] > 0
