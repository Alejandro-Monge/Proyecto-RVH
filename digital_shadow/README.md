# digital_shadow — vision shadow package

Python package for the vision shadow: it reconstructs the vehicular mobility of the
crossing from a traffic camera video and projects it onto a shared SUMO runtime.
The flow is one-directional physical→virtual (monitoring without actuation): SUMO
does not take control; the shadow vehicles are driven through TraCI according to
what perception infers.

Subpackages:

- `perception/` — vehicle detection (YOLO detector, COCO classes, and detection
  types).
- `tracking/` — multi-object tracking, re-identification (`reid/`), and causal
  reconnection of trace fragments.
- `geometry/` — ground contact point and box correction; the basis of the
  homography projection onto the road plane.
- `sumo/` — synchronization with SUMO: lane map, position, route, state
  reconstruction, staging, and the presentation layer.
- `occupancy/` — approach zones and waiting boxes for the occupancy metric.
- `sources/` — video sources (reading from file).
- `config/` — loading of the site, detector, and tracker configuration.
- `causal/` — causal contract that perception must respect.
- `harness/` — pipeline orchestration and output collectors.
