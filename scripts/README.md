# scripts — entry points and utilities

Demo entry points and utilities of the vision shadow pipeline.

- `run_harness_demo_causal.py` — entry point of the vision shadow demo:
  it replays the precomputed artifacts and brings up the view (SUMO-GUI or
  the web 3D viewer).
- `demo_causal/` — orchestration and recording of the paired demo (reality
  and shadow panels), with its composition and verification scripts.
- The rest are pipeline utilities: homography calibration, track event
  derivation, fragment reconnection, fidelity and occupancy measurement, and
  supporting video rendering.

`paper_figures/` is a separate layer that only reads the production state;
`paper_figures/common.py` provides the provenance guard (`check_provenance`):
it compares the md5 of each input against fixed pins and aborts if any datum
does not match, so that no output is generated over stale or accidentally
altered data.
