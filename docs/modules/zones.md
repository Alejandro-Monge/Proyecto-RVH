# Module — Site zones: classification in the image plane

The **zones** are polygons in the **image plane** (pixels) that define, for each site, the regions
on which the **ground contact point** of a vehicle falls along its trajectory, and from which the
following is decided:

- **Exclusion**: traffic that is not from the intersection (for example, a parking lot) and is discarded.
- **Route** (entry/exit per approach): origin and destination of the vehicle for the route metric. The
  eight route zones (entry + exit per approach) are asymmetric, with the Option B design.

## Choice of the image plane

The approach classification is **coarse** and must be robust exactly where the homography is worst
(south/west horizon, breakdown of the single plane). A polygon in pixels does not inherit the projection
error of the far field. The homography is reserved for **positioning in SUMO** in the mid and
near field.

## Polygon format

- Coordinates in **image pixels at the site's full resolution** (`frame_width`×`frame_height`
  from `site.yaml` = 1200×646). Not from the matplotlib PNG nor from any scaled view.
- Vertices **ordered**; the polygon **closes implicitly** from the last vertex to the first
  (the first is not repeated at the end).
- Membership by the **ground contact** `(gx, gy) = ((x1+x2)/2, y2)`
  (`digital_shadow/geometry/ground_contact.py`), **not** the centroid.

## Configuration

`config/sites/rusia/zones.yaml` (a file separate from `site.yaml` so as not to overload it; the
route zones live there too). Header fields: `site_id`, `reference_frame` (frame on which it was marked),
`frame_width`/`frame_height` (sanity validation against `site.yaml`). Each zone carries
`description`, `arm`, `marked_by`, `marked_date`, `source` and `polygon_px`.

### West exclusion zone (`exclusion_zones.oeste_parqueadero`)

Real parking lot of the west mouth (upper right in the image): vehicles parked off
the roadway. It is **not** the static cluster of the south (upper left), which is traffic **stopped at
red** and is **not** excluded. Marked with `scripts/select_exclusion_zone.py`. 7 vertices.

**Quantitative verification** (over `verify_out/tracks_rusia.csv`, ground contact): inside the
polygon fall `tid12` (4669 obs, 100%), `tid116` (2444, 100%), `tid36` (445, 100%) = parked,
and `tid128` (106, parking maneuver); **0 through-traffic tracks** with any point inside
(it does not bite the roadway); the south cluster stays outside.

## Marking tool

`scripts/select_exclusion_zone.py` — polygon marker by clicks (OpenCV), with the same mechanics
of zoom/pan as the homography selector. The background is a clean frame in pixel space (1:1) with the
accumulated trajectories (ground contact) and the long-static tracks highlighted, plus cardinal labels
per corner as an orientation reference. The capture output goes to
`verify_out/zones/rusia/` (CSV + preview PNG); the promotion to `zones.yaml` is a separate step after
reviewing the preview. `--selftest` validates the `win↔src` mapping without a display.

## Limits

This module **only stores geometry**. The **application** of the zones in the pipeline —exclusion
after the detector and before tracking/inference, keeping the detector generic and
reusable; route classification by sequence of zones with *dwell*— corresponds to a later
layer.

## References

- `docs/modules/ground_contact.md`: the ground contact point used as reference.
- `docs/modules/detector.md`: the detector prior to the exclusion.
- `docs/modules/occupancy_zones.md`: the approach zones layer, which shares the polygon format.
- Site config: `config/sites/rusia/zones.yaml`, `config/sites/rusia/site.yaml`.
