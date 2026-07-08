# config — pipeline and site configuration

Configuration for the demo, in two levels.

At the root, the pipeline parameters shared across sites: `detector.yaml`
(detection), `tracker.yaml` (tracking and re-identification) and
`positioning.yaml` (positioning on the road plane).

In `sites/rusia/`, the site-specific configuration: the homography and the
approach zones, the SUMO network and its lane map (`sumo/`), the 3D scene
overlays (`web3d.yaml`), the turn derivation table (`turns.csv` with its
README), and the parameters for occupancy, matching and the other pipeline
pieces. `site.yaml` gathers the core site settings. The files set aside as
inherited from the previous work live in `sumo/heredado/` with their own README.
