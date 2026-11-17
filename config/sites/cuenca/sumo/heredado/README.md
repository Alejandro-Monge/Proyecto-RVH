# Inherited from the previous work (NOT used by the digital shadow)

These files come from the previous work, which **did** have traffic-signal
control and a demand simulation in SUMO. The **digital shadow does NOT use
them**: SUMO never takes control; the vehicles are driven via TraCI according to
what perception infers, and the signal plan is not read. The only thing the
shadow consumes from the net is the lane and connection **geometry**.

They are set aside here to keep the official net clean, in a **reversible** and
traceable way (they are not deleted: they are evidence of the starting point).

## Contents

- `intersection_rusia_pre_rotacion.net.xml` — the **original** net as it was drawn
  in the previous work, **rotated 180° with respect to the real world** (it was drawn over an
  inverted base image). It is the starting point; it is kept as evidence. The good net
  (straightened by a 180° rotation + normalized south connections) is the canonical
  `../intersection_rusia.net.xml`.
- `rutas_rusia.rou.xml` — 12 O-D routes + 12 synthetic background `flow`s
  (60/120 veh/h). Demand for the control simulation, not for real traffic.
- `simulacion_rusia.sumocfg` — control scenario: ties together the net + the routes,
  `step-length=0.0333` (30 Hz), `end=86400`. It points to
  `intersection_rusia_pre_rotacion.net.xml` (the pre-rotation net, right here) and to
  `rutas_rusia.rou.xml`.

## If this scenario ever needed to be run

The routes reference edges by id (`E1`, `-E5`, …), which **do not change** with the
net rotation; topologically they are still valid. The `.sumocfg` and its pre-rotation
net live together here, so the scenario is internally coherent as is. (If it were to be
run over the straightened **canonical** net, just point `net-file` to
`../intersection_rusia.net.xml`.)

> Note for the far future: if this were to evolve into a **twin** with real
> traffic-signal control, the signal plan would have to be **re-synchronized with the
> net's `linkIndex`** (this does not apply today: the shadow ignores the plan).
