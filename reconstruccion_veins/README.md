# reconstruccion_veins — GT2 dataset (shadow mobility for VEINS)

Isolated, self-contained copy. The original mobility reconstruction
(f5048e02) is NOT touched; there are no symlinks or imports of it here.

- `datos/gt2_base/` — trajectories drawn by the original reconstruction, materialized (70 npz: recon_xy,
  reconstruction by lane geometry; exporting the raw homography xy produced
  drift/cars at the edge during replay).
  Generated once by a base exporter (offline, not included; the single point of contact
  with the original reconstruction, deterministic). `datos/manifest_base.json` with hash.
- `arreglos.py` — applies the 14 fixes on top of the base → `datos/gt2/`
  (62 npz with per-sample provenance) + `datos/manifest_gt2.json` (hash, metadata,
  detail of each fix). Deterministic: two runs → same hash.
- `red/intersection_rusia.net.xml` — physical copy of the 19-link network.
- `datos/gt_ref.csv` — physical copy of the GT (time anchors of v3/v12/v44).

Per-sample provenance (`prov`): 0 OBSERVED(recon) · 1 SMOOTHED · 2 INTERP_ROUTE ·
3 EXTENDED_GT · 4 SYNTH_HOLD · 5 DESOLAPADO. Overall: 81.6% observed,
0.8% declared synthetic, 17.6% shifted by the minimal de-overlap (queues re-spaced
to real gap; times intact). In addition: turn path smoothed (spline) and lateral of the
raw restored (±0.7 m).

GT2 = **62 vehicles** (manifest_gt2.json: 70 recovered chains − 8 declared
discards due to perception limits, case by case in `manifest.exclusiones`;
the 3 southern GT (22, 37, 49)
have no trace and remain a **declared limit**, they are not fabricated.
Do not mix denominators: **62 = MESSAGES (GT2)**; the 73 that cross in VIDEO
are GT1.

Current dataset hash: see `datos/manifest_gt2.json` (`hash_md5`).
NOTE: the MESSAGES carry x,y from recon.

## Edge removal

The driver (`driver_replay.py`) includes removal at the
crop edge: it cuts the suffix of samples beyond the physical end of the
exit edge (margin 0) and hardens the release (snap to route keepRoute=1 +
speed floor). Cars DISAPPEAR when they leave the frame; it removes the
11 zombies that emitted an empty laneId + invalid heading (24% of the messages
in the MC sweep). The DATASET does not change: hash `f712b746...` still valid (it affects
the replay, not the data).
