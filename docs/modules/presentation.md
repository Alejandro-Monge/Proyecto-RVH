# Module — Presentation smoothing (`digital_shadow/sumo/presentation.py`)

Smooths the jitter of the trajectory drawn in the SUMO demo (stationary vehicles that vibrate back and forth), without touching the position measured for fidelity. It is the presentation layer, separate from the measured shadow.

## Separation between presentation and measurement

The fidelity shadow is measured by reading `getPosition` after `simulationStep` in separate loops that use the raw recon position. `render()` (the demo) emits no metric (it does not call `getPosition`, does not write CSV): it only takes screenshots. That is why smoothing the position that `render()` sends to `moveToXY` cannot alter any fidelity number. There are two paths from the same recon:

- Measured: `run_core` → raw recon → moveToXY → getPosition (fidelity shadow). Untouched.
- Drawn: recon → `presentation_smooth` → moveToXY → screenshot (render only).

Mechanical proof: `draw_xy` is only created inside `render()`; the `veh` of `run_core` (the source of the metrics) does not contain `draw_xy`.

## Cause of the jitter

The detector box of a stationary vehicle flickers, and its ground contact oscillates ~6–14 Hz (peak to peak 12–26 px in the image). After the homography (which scales and amplifies in the far field) and the recon, the drawn trajectory vibrates ~5–14 px. `moveToXY` adds no jitter (it places exactly). The root cause is the detection.

## Filter: causal dead-band with anchor recentered by moving median

`presentation_smooth(recon_xy, recon_t, H, deadband_px, median_window_s)`:

- If the new position is less than `deadband_px` (image px) from the current center, it stays frozen; the center is re-estimated as the moving median of the recon positions of the frozen stretch in a window of the past (`median_window_s`). Since the jitter is symmetric, the median is the real center, so the stopped vehicle is drawn centered (not at the point of entry into the dead-band). In a slow creep the window follows the advance.
- If it jumps outside `deadband_px`, the vehicle started: the center jumps and the stretch (the window) is reset.

The filter is causal (past only, no accumulated lag, without needing the future), suitable for live video (a design requirement), unlike a zero-phase filter. The threshold is in image px for perceptual uniformity.

## Parameters

In `config/sites/rusia/zones.yaml` → `presentation:` (not set by hand):

- `smooth_deadband_px` (8): dead-band threshold. Trade-off: the sweep showed that there is no threshold that cleanly separates stopped-jitter from slow-movement (a larger threshold kills more jitter but stair-steps the creep more).
- `smooth_median_window_s` (0.5): window of the moving median that recenters the frozen one (it covers several periods of the 6–14 Hz jitter, so the median is stable and coincides with the real center).

## Known limitations

There is no clean threshold. Suppressing all the jitter (threshold ≥14) freezes ~0.37 s for one that moves clearly; a creep (a queue advancing slowly) faster than ~threshold/median_window_s keeps stair-stepping (inherent to the dead-band). The offset of the fixed anchor (≤threshold) is resolved by the moving median (measured offset +4.8 px → 0.0 px in v104).

## Input and output

- Input: `recon_xy (N,2)` world, `recon_t (N,)` s, `H (3,3)` image→world, `deadband_px`, `median_window_s`.
- Output: `draw_xy (N,2)` world to draw. It does not mutate `recon_xy`. Causal: `draw_xy[i]` depends only on `recon_xy[:i+1]`.
- `load_presentation(site_id)`: reads `(deadband_px, median_window_s)` from the site config.

## Verifications

Suite 77/77, net `5fbb6a28`, exit_carril west exit = 100 %; the fidelity metrics do not change with the smoothing (`draw_xy` absent in `run_core`), which verifies that the presentation does not touch the measurement. Before and after the smoothing: `verify_out/jitter_antes_despues.png` (simple) and `verify_out/jitter_refinamiento_antes_despues.png` (refinement: simple with offset +4.8 px → refined 0.0 px centered; the stopped one does not vibrate and the one that really moves keeps advancing).

## References

- `docs/modules/sumo_stage.md`: the SUMO stage where the trajectory is drawn.
- Site config: `config/sites/rusia/zones.yaml` (`presentation` block).
