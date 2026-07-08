# Module — Ground contact (`digital_shadow.geometry.ground_contact`)

Defines the **vehicle reference point**: the **ground contact**
`(gx, gy) = ((x1 + x2) / 2, y2)`, the center of the **bottom edge** of the detection box, in
image pixels. It is the single, stable piece that derives that point from the raw corners of
tracking. Its main consumer is the **zones layer** (classify lane and approach) and the
**projection to SUMO** with the homography.

## Rationale

The ground contact lies in the plane of the **asphalt**, the same plane for all vehicles
regardless of their height. That:

- **Eliminates the height parallax** that the centroid introduces: a bus and a car with the same
  ground contact have centroids at different heights, and the centroid would project to different
  ground positions.
- Is **consistent with the single-plane assumption** of the image→ground homography (calibrated with
  ground-plane points).

The centroid `(cx, cy)` is **not** the reference point: it exists in the tracking CSV only as a
visualization convenience. Using it to classify lane or project to SUMO reintroduces the
parallax that the ground contact avoids. The module resolves that ambiguity between the two points: the
system contract is the ground contact.

The ground contact's **drift** is worse in the **near** field (large boxes, the bottom edge
sweeps laterally). It is noise, not bias, and it does not reach the position in SUMO, because the zones
layer places **by lane** (SUMO centers the vehicle in the lane).

## Files

```
digital_shadow/geometry/
  __init__.py        re-exports the public API
  ground_contact.py  ground_contact(), add_ground_contact(), GROUND_X, GROUND_Y
```

## Input and output

- `ground_contact(x1, x2, y2) -> (gx, gy)`
  A **vectorized** core of pure arithmetic. `x1, x2, y2` can be scalars, `np.ndarray`, or
  `pd.Series` (the operation broadcasts element by element) and the output preserves the type.
  `gx = (x1 + x2) / 2`, `gy = y2` (exactly the bottom edge; not `cy`).

- `add_ground_contact(df, x_col=GROUND_X, y_col=GROUND_Y) -> df`
  Adds to a DataFrame with raw columns `x1, x2, y2` (the `run_tracker.py` CSV format) the
  canonical ground-contact columns. It mutates `df` in place and returns it. It does not touch
  `cx`/`cy`.

- Canonical names: `GROUND_X = "gx"`, `GROUND_Y = "gy"`. A **single** name throughout the repo (no
  duplicate variants like `ground_x/ground_y`).

## Consumers

The analysis and calibration scripts derive the ground contact through this module, no longer
inline: `project_tracks_birdseye.py`, `plot_camera_trajectories.py`, `compute_homography_improved.py`,
`plot_birdseye_hires.py`, `analyze_south_trajectories.py`. The main consumer is the zones layer.

## Verifications

`add_ground_contact` produces `gx/gy` columns **byte-for-byte identical** to the inline formula,
verified with `np.array_equal` over the real CSV. The homography and the figures do not change by a
single pixel.

## Limits

- It does not read the CSV or do I/O: it only derives the point. The caller loads the data.
- It does not project to meters (that is the homography) or classify lane (that is the zones layer): it
  only delivers the reference point in pixels.
- It does not correct the ground-contact drift: it is absorbed downstream, in the zones layer.

## References

- `docs/modules/tracking.md`: origin of the tracks CSV with the raw corners.
- `docs/modules/zones.md`: the zones layer, the point's main consumer.
- `docs/modules/track_events.md`: uses the ground contact as the reference point.
