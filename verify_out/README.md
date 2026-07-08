# verify_out — precomputed artifacts

Precomputed outputs that the demo reproduces without recomputing them. By default the
vision demo does not re-run perception over the video (that requires a GPU and
the weights in `models/`): it reproduces these already-generated artifacts.

Main contents:

- `tracks_rusia.csv` — per-frame tracking tracks of the demo segment.
- `events/rusia/` — derived track events (entries, crossings and exits per
  vehicle).
- `homography_gcp/homography_img2ground.npy` — image→road-plane homography
  matrix used to project the tracks.

Other intermediate derivatives of the verification runs also live here
(for example the long generalization segment and the 3D viewer logs). They are
reproducible outputs from the repo's code and data; they are not inputs
to be edited by hand.
