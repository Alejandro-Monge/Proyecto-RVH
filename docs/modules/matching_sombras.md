# Module — Shadow↔GT matching and fidelity metrics (`scripts/matching_core.py`)

The single executable specification of the shadow↔GT matching of the metrics protocol (step 6),
consumed by the visual shadow and the messages shadow; and on top of it, the route and lane metrics
(7.1), crossing delay (7.2), and ghosts and omissions (7.3). The golden rule is to have the same
definitions character by character; the only thing that differs per shadow is its GT, its clock, and
its window.

## Pieces and single source

| Piece | File | Consumers |
|---|---|---|
| Core (step 6): lexicographic cost, Hungarian per approach, one-to-one, window; ghosts and omissions; P/R report per approach | `scripts/matching_core.py` | visual (`match_events_to_gt.py`, which re-exports for `reconnect_fragments.py`) and messages (`matching_mensajes.py`) |
| Crossing detector: front, net stop line, 3 m hysteresis, by transition | shared crossing detector (offline, not included) | messages GT generator and messages-shadow event adapter |

Equivalences verified on factoring: visual → `matches.csv` `abf7f5b5…` and `reconnected/matches.csv`
`198864d0…` **byte-identical to their pins**; messages GT regenerated **byte-identical**
(`be012d86…`/`3785aecc…`).

## Events per shadow

- **Visual**: `verify_out/events/rusia/track_events.csv` (event-per-track layer; `entry/exit_time_s`
  = stop-line crossing). GT: 73 video routes.
- **Messages**: derived from the dense dump of the causal reconstructor by
  `reconstruccion_causal/scripts/matching_mensajes.py` with the shared detector (the player's
  `crossing_times.csv` is not used: a different detector would measure a detector difference). Entry
  approach: first normal approach edge; if the vehicle is born inside the crossing, from the initial
  internal lane via `lane_map.junction_internal` (case v119). GT: 62 sim vehicles
  (`gt_mensajes_sim_rusia_carril.csv`), clock = simulation seconds.

## Tolerance window

Each shadow sets its window by sensitivity:

- Visual: 1.0 s (`site.yaml: matching_tolerance_s`; stable zone of its sweep).
- Messages: **1.5 s** (`config/sites/rusia/matching_mensajes.yaml`): at 2 Hz/P00 the stable zone is
  flat in [1.25, 3.0] (54 matched) and below 1.25 s the matching collapses into spurious crossings
  because the systematic channel delay (~+1.07 s) exceeds the window (identity control: 0/17 correct at
  1.0 s). The sweep is re-emitted on each run (`window_sweep_mensajes.csv`).

## Metrics

Same definition in both shadows:

- **Route and lane (7.1), three levels** per entry approach: approach (matched/GT), plus entry lane,
  full route (4 fields). Visual: `measure_lane_accuracy.py` (with declared resolution bands). Messages:
  `metricas_71_mensajes.csv` (exact lane: in the sim there are no bands). P/R/F1 in the per-approach
  report.
- **Delay (7.2)** = `t_cruce_pare_sombra − t_cruce_pare_GT` per matched vehicle (= matching's
  `dt_entry`; both sides anchor at the stop), with its distribution (median, p10/p90, extremes).
  Visual: `delay_cruce.csv` (total: median +0.219 s, n=66). Messages: in `resumen_matching.json` (2
  Hz/P00: median **+1.066 s**, p10 1.029, p90 1.112, n=54; includes the 1.0 s buffer, latency, and
  reconstruction by construction).
- **Ghosts and omissions (7.3)**: direct read of the matching, per approach, with a **cause per
  omission** in messages (`omisiones_mensajes.csv`): `cruce_en_primer_intervalo` /
  `no_casado_en_ventana` / `sin_sombra`.

## Censoring and the v119 failure mode

- **Symmetric censoring** (v0/v1/v2): they are born with the front past the line on both sides, they
  are discounted symmetrically, they are outside the universe and are not error. Verified with data,
  not assumed (2 Hz/P00: shadow censoring = {v0, v1, v2} plus v119).
- **v119 is an omission, not censoring**, and it is signal, not noise: it crosses the stop 0.21 s after
  appearing (GT: appearance 156.60, crossing 156.81) and its first message (t=157.13, 2 Hz) already
  comes from an internal lane; the shadow never sees a position behind the line and, by transition, it
  generates no event. It is the **characteristic V2C failure mode: omission from a crossing within the
  first interval between messages**, dependent on the cadence (1 Hz > 2 Hz > 5 Hz is expected; that it
  appears at P00 isolates it as a pure cadence effect). Traceable per vehicle. Mechanics nuance: the
  cost can reassign the neighboring shadow (at 2 Hz/P00, the GT v119 matched with veh_117 by dt=0.52 <
  typical delay, and the omission surfaces as v117); the identity control makes it visible, and the
  aggregate count (matched/omissions) does not change.

## Protocol boundary zone: waiting on the line

3/62 vehicles at 2 Hz/P00 (v9, v45, v60, queue heads stopped at the line; v9 belongs to the "wait past
the net line" trio already declared in `gt_mensajes_sim.md`) produce ghost pairs plus spurious
omissions with Δt of 10–92 s: the reconstructed shadow stays millimeters on the positive side for tens
of seconds and the 3 m commitment retroactively validates a millimetric transition (the layer-0 GT is
protected by the stop clamp; the generator itself documents "v45 −100 s" without hysteresis). Verified
with data that the obvious fix does not work: a dead band δ=0.05 m in the transition changes 53/62 GT
crossings (up to 22 s for those who wait on the line), which would alter the semantics of the shared
protocol.

**Interpretation**: at 2 Hz/P00 (no loss) the effect is isolated and does not come from the channel: it
is **geometric ambiguity of the crossing detector, which does not distinguish "crossing slowly" from
"being stopped on the line"**. They are declared a protocol boundary zone: vehicles stopped on the stop
line (v9, v45, v60, with fixed mobility throughout the MC sweep), traceable (same physical id in ghost
and omission), interpreted with that caveat and not corrected. Every downstream aggregation (the
degradation curve) reports them separately, never mixed with the channel omissions.

## Artifacts per run

Messages artifacts per run: `matches_mensajes.csv` (with `id_ok`, identity control; verification,
never mechanism), `fantasmas_mensajes.csv`, `omisiones_mensajes.csv` (with cause),
`censura_sombra.csv`, `baseline_por_brazo_mensajes.csv` (with F1), `metricas_71_mensajes.csv`,
`window_sweep_mensajes.csv`, `resumen_matching.json` (universe, delay 7.2, identity, input md5s).
Reference run: `reconstruccion_causal/runs/censura_2Hz_P00/matching/`.

## Entry lane accuracy

Tabular metric generated by `scripts/paper_figures_resultados/carril_fidelidad_71.py` (persists the
logic of `measure_lane_accuracy.py`) → `verify_out/tablas_resultados/carril_fidelidad_{vision,mensajes}.csv`,
`carril_confusion_vision.csv`, and a seal. Pinned inputs.

**Entry is the result, for two reasons:** (1) it **discriminates** — 3 entry lanes; the exit converges
to ~82 % in lane 1, with low discriminating power; (2) **by purpose** — the input for traffic-signal
control is what approaches the intersection, not what has already exited; the exit is trivial twice
over (it converges to 1 lane and contributes nothing to the control decision). That is why only the
entry is reported.

**Measurement decisions:** the denominator is always over GT (the 7.1 convention, which folds in
recall); **per-lane resolution forced in the 4 approaches** (grouped/arm_only removed: grouping lanes
to raise accuracy contradicts spatial fidelity); **south enters with its real sample** (n=4/3/4) with a
sparse-sample flag, neither excluded nor grouped; **by (approach, lane)**, not aggregated by approach
(the aggregate would cover the failing lane). The matcher is **lane-blind**, so the comparison is not
inflated.

**Entry lane accuracy (over GT, reconnected system), by (approach, lane):**

| approach | c1 | c2 | c3 |
|---|---|---|---|
| east | 3/3=1.00 | 13/14=0.93 | 4/5=0.80 |
| north | 13/13=1.00 | 3/4=0.75 | 2/2=1.00 |
| west | 2/2=1.00 | 7/8=0.88 | **5/7=0.71** |
| south* | 4/5=0.80 | **3/5=0.60** | 4/5=0.80 |

(*south = sparse sample n=4/3/4, not validated with a long video.) Approach level (recall) → plus lane:
total 68/73=93 % → 63/73=86 %. Wilson 95% CI per cell in the CSV; the per-approach LOO (east/west 0/·,
north 1/19) measures the classifier's separability, a different caveat from the swap.

**Recall and lane accuracy degrade at the same physical point.** The worst cell is **south lane 2**
(far field, worse camera geometry): accuracy 0.60 **and** with an omission. The omissions (5 in the
reconnected system) and the lowest-accuracy cells fall in the same places: far approaches (south, west)
and far lanes (c3): east·c3, west·c3, south·c2, south·c3. The cause is a single one: a far vehicle is at
once **harder to detect** (it does not cross the stop → generates no event → lowers recall and
underrepresents that lane) **and harder to classify by lane** (compressed geometry in the far field).
That two independent metrics —detection (7.3) and lane (7.1)— degrade in the same cell characterizes
where and why vision fails.

**Messages (reference, not result):** lane over `id_ok=True` = **1.000** at entry in 1/2/5 Hz P00
(discarded by swap 8/3/10). Exact by construction (the message carries the lane and the reconstruction
preserves it); it is the reference against which the vision number is read, not a result of its own.

**Swap bound per approach** (matching error, distinct from LOO/CI): without `id_ok` in vision,
co-temporal GTs of the same entry approach plus exit within 1.0 s — east 5, north 2, **west 6**, south
0. Where the bound is high (west) the lane number carries that caveat; where it is 0 (south) the low
accuracy is not swap, it is geometry.

## Monte Carlo sweep and degradation curve

- **Windows per frequency** (rule: left edge of the common stable zone P00∩P_max + 0.2 s; the sweep's
  P_max is **P50**, there is no P90): 1 Hz → 2.3 s, 2 Hz → 2.0 s, 5 Hz → 2.1 s
  (`matching_mensajes.yaml:windows`; curves in `runs/ventana_*_{P00,P50}/matching/`). The stable zone
  shifts to the right under loss at all three frequencies: calibrating only on P00 would have left all
  three short.
- **Execution**: `barrido_matching_mc.sh` — 3003/3003, 0 failures, ~12 workers, ephemeral dense
  (deterministically regenerable), file-based census, `--no-sweep`.
- **Aggregation**: `agregar_curva_mc.py` → `runs/matching_mc/{curva_degradacion.csv,
  tabla_respaldo.csv, alertas_borde.csv}`: p10/p50/p90 band over ~100 reps, three separate channels
  (channel / boundary zone {v9,v45,v60} / censoring edge {v0,v1,v2}); clean universe = 56 GT.
- **Edge-proximity alert** (watches the window without a per-condition sweep): **0 alerts in 33
  conditions** (maximum observed in an isolated rep: 5.9 % of pairs at <0.2 s from the cut; median 0 in
  all).
- **v119 signal measured as a direct metric** (`censura_cadencia`: non-symmetric shadow censorings,
  robust to the cost swaps that re-label the omission): median per condition — 1 Hz: 2 (P00) → 5 (P50);
  2 Hz: 1 → 2; 5 Hz: 0 → 0. **The predicted order 1 Hz > 2 Hz > 5 Hz, with P00 isolating the pure
  cadence effect (2/1/0)**: at 5 Hz the first interval (0.2 s) almost always gets to see the back side
  of the line; at 1 Hz (1.0 s) it does not.
- **Main curve (p50)**: full route (7.1 level 3) — 1 Hz 0.82→0.73; 2 Hz 0.91→0.86; 5 Hz ~0.80–0.82
  flat. Delay (7.2) stable ~1.06–1.07 s in all conditions (dominated by the 1.0 s buffer, as designed).
  The degradation figure is rendered in the results-figures task from these CSVs.

## Messages buffer sensitivity

Offline sweep of `--delay-s` (without re-running VEINS): the 1.0 s buffer is justified by
**continuity** (it minimizes reacquisition jumps with recall at its maximum), not by recall (which is
flat up to ~1.2 s and breaks ≥1.8 s). Table and analysis in
`verify_out/tablas_resultados/buffer_sensibilidad.{csv,md}`.

## Known limitations

**Vision delay-vs-accuracy curve.** The delay-vs-accuracy trade-off is conceptually inherent to vision
(inferring the destination early with little trajectory gives low delay and lower accuracy; waiting
gives more accuracy and more delay). V2C does not have it: it receives the datum, it does not infer it.
Its measurement is not available for a technical reason:

- Vision's destination inference is a fixed lane→turn rule that commits only once, at the stop crossing
  (`route.py:136-142`, `commit_destination_predicted` → `predicted_exit`/`predict_turn` from
  `lane_turn_rule.yaml`; `orchestrator.py:119`, at `cross_t`). It has no horizon parameter (`emit_at_s`
  or another does not exist in the repo; the only timing is a fixed 0.1 s look-ahead in
  `orchestrator.py:111`, not of destination inference).
- Tracing the curve would require building a destination-inference model parameterized by horizon
  (predict the destination from a partial trajectory at horizon X) that does not exist: it is new
  modeling, not a sweep of an existing parameter.
- The measurement side would be cheap (persisted per-frame trajectories plus matching plus GT already
  measure the accuracy of any predicted destination); the main limitation is the inference model, not
  the instrumentation.

## References

- `docs/modules/track_events.md`: event-per-track layer (visual-shadow events).
- `docs/modules/gt_mensajes_sim.md`: GT and protocol of the messages shadow.
- `reconstruccion_causal/docs/reconstructor_causal.md`: causal reconstructor and Monte Carlo sweep.
