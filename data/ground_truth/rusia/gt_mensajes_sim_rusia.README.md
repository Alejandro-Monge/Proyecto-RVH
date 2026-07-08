# GT of the MESSAGES SHADOW (over the sim) — contract

**What they are**: the reference truth for the V2I MESSAGES shadow. Derived from the
SIMULATION (GT2 dataset `f712b74643f5687f94d79356ca5cabec`, 62 vehicles, direct
mobility + queue→nose offset + geometric nose), NOT from the video. The video GTs
(`ground_truth_rusia_v2_carril_processed.csv`, `ground_truth_occupancy_rusia.csv`)
remain the VIDEO truth and are not touched.

**Generator**: offline script (not included in this repository).

## Contract decisions

1. **REFERENCE POINT = NOSE** (the SUMO position after the queue→nose offset):
   (a) it is exactly what VEINS puts in the messages (`getPosition` = front);
   (b) it coincides with the "NOSE rule" of the video GT;
   (c) it is the physical semantics of a line detector.
2. **CROSSING detector = the arm's stop line** = the end of the approach edge
   (J5 side) of the net. **CROSSING COMMITTED with 3 m HYSTERESIS**: it counts the first
   crossing of the line after which the nose advances ≥3 m without going back. Without hysteresis,
   the nose of a queue head (the stop clamp leaves it grazing the line ±0.5–1.3 m)
   fires the detector 30–100 s before the real crossing. **The messages shadow must
   count with the same hysteresis** or its count will not be comparable with this GT.
3. **LANE detector = APPEARANCE** of the vehicle in the lane (insertion in SUMO;
   the SUMO position directly, without bands or fallback). In the sim there is no
   `sin_carril`: every vehicle is born in a concrete lane.
4. **Times in SIMULATION seconds** (the base the shadow sees). To cross-reference with
   the video GT: `t_sim = t_video + delay` (per-vehicle delay in
   `reconstruccion_veins/datos/manifest_gt2.json`).
5. **Censoring** (as in the video GT): v0, v1, v2 are born with the nose already past
   the line (the sim window cuts the event) → `cruce_censurado=si`, empty time.
   The unobserved event is not invented.

## Documented LINE offset (sim vs video)

Cross-validation against the video GT (56 matchers, same clock):
`cruce_sim − delay − entry_time_video`: **median −0.62 s** (p10 −1.73 / p90 −0.12) —
consistent with geometric nose vs annotated nose. **Explained exceptions (note in
the CSV)**: v9, v34, v76 (−25 to −36 s): the net line falls UPSTREAM of the video's
painted stop line on those arms, and the datum's wait falls BETWEEN the two lines. Since the
messages shadow measures over the SAME net line, GT and shadow stay aligned;
the offset is only against the video GT and is declared.

## `gt_mensajes_sim_rusia_carril.csv` (analogous to _v2_carril_processed)

| column | meaning |
|---|---|
| vehicle_id | sim id (vXX; the same as manifest/driver) |
| gt_video_event_id | event_id of the video GT (matchers only; empty if not) |
| vehicle_class | car / bus / truck |
| entry_arm / entry_lane | GT arm and lane (1=right) at APPEARANCE |
| entry_lane_sumo | SUMO lane of the insertion (e.g. E5_2) |
| exit_arm / exit_lane | GT exit arm and lane (nearest -E lane on leaving) |
| aparicion_sim_s | LANE detector: instant of insertion in SUMO |
| cruce_pare_sim_s | CROSSING detector: nose crosses the stop line (committed, 3 m hyst.) |
| cruce_censurado | si/no (si ⇒ empty time) |
| salida_junction_sim_s | nose crosses the start line of the exit edge |
| turn | left / right / through |

## `gt_mensajes_sim_rusia_occupancy.csv` (sibling, the video occupancy template)

Same columns as the video template with the sim semantics:
`zone_enter_time_s` = APPEARANCE in the lane (the sim lane detector);
`zone_exit_time_s` EMPTY in crossers (sibling rule: the zone exit IS the stop-line
crossing, which lives in the lane file); `cruce_pare_time_s_ref` = reference
(the same value from the lane file, not measured here);
`entrada_censurada` = the censored crossing of v0/v1/v2.
