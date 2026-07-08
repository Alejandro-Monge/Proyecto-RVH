# data — inputs and ground truth

Data inputs for the demo, separated into two layers.

- `sources/` — input video of the crossing (Rusia site). It is the material the
  vision shadow starts from. Only the 180 s demo segment
  (`sources/rusia/rusia_tramo_000-180.mp4`) is versioned here, which is all the
  local demo needs. The long generalization segment
  (`sources/rusia_long/`) is not versioned (available on request). The default
  demo does not re-run detection: it replays the precomputed artifacts in
  `verify_out/`.
- `ground_truth/` — ground truth used to contrast the shadows: the ground truth
  annotated over the video (lane and occupancy per vehicle) and the ground truth
  of the messages shadow derived from the simulation. Each set carries its own
  README with the column-by-column contract.
