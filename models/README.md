# models — perception model weights

Weights of the models used by the vision shadow. They are large binaries; they are
downloaded and normalized here (their content is not versioned).

- `yolov8x.pt` — YOLOv8x object detector, used to detect vehicles in
  the video.
- `osnet_x1_0_msmt17.pt`, `osnet_x0_25_msmt17.pt` — OSNet re-identification
  models, used by the tracking to associate and reconnect tracks.

These weights are only needed to regenerate the tracks from the video. The demo
by default reproduces the precomputed artifacts in `verify_out/` and does not
re-run perception.

## Downloading the weights

The weights are not versioned. The libraries pinned in `requirements.txt` fetch
them automatically on first use, so no manual step is normally required:

- `yolov8x.pt` — downloaded by **ultralytics** the first time the detector runs
  (e.g. `python -c "from ultralytics import YOLO; YOLO('yolov8x.pt')"`), and also
  available from the Ultralytics GitHub *assets* releases. Leave it in this folder.
- `osnet_x1_0_msmt17.pt`, `osnet_x0_25_msmt17.pt` — OSNet re-identification
  weights (MSMT17-pretrained), downloaded by **boxmot** on first use into its
  weights cache.
