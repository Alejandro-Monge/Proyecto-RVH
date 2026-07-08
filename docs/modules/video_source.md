# Module — Video source (`digital_shadow.sources`)

Makes any video source behave the same to the pipeline. The current
implementation is a prerecorded file that behaves like a camera (real-time mode). Replacing
that implementation with a live camera touches nothing above the boundary. It delivers
**timed frames** and nothing else: it does not detect, does not draw and does not know about vehicles; that
corresponds to the consumer (the detector and what follows).

## Boundary and files

```
digital_shadow/
  config/site.py        SiteConfig + load_site_config (reads site.yaml)
  sources/
    base.py             Frame, VideoSource (ABC), Clock + MonotonicClock,
                        FrameReader (ABC)   ← the contract and its dependencies
    file_source.py      FileVideoSource (real-time + drop), OpenCVFrameReader
    __init__.py         file_source_from_site(site)  ← wires config → source
```

The pipeline depends **only** on `VideoSource` and `Frame`. Everything else is internal or
replaceable.

## Input and output

`VideoSource` exposes three operations (plus the `with` context manager):

| Operation | Input | Output | Semantics |
|---|---|---|---|
| `open()` | — | — | Leaves the source ready. Sets the session `t0` (= "the camera turns on"). |
| `read()` | — | `Frame(image, timestamp_s, end_of_source)` | Returns the frame corresponding to the clock's current instant. |
| `close()` | — | — | Releases the source. After closing, `read()` returns `end_of_source=True`. |

It also exposes two session counters: `delivered_frames` and `discarded_frames` (the latter
is the main performance metric).

### `Frame` (the only thing that crosses the boundary)

- `image`: BGR `numpy.ndarray` of the delivered frame, or `None` if `end_of_source`.
- `timestamp_s` (float): **seconds since the session `t0`**. In file mode it is the **media
  time** of the delivered frame = `index / fps_ref` (its real index, not wall-clock). That is why
  it remains comparable with the ground truth regardless of which frames are dropped.
  Exact computation: `t = index · 12/355` with `fps_ref = 355/12`.
- `end_of_source` (bool): explicit and terminal signal. Once `True`, subsequent reads keep
  returning `True` (file exhausted or source closed; the pipeline does not distinguish file
  from camera).

`Frame` carries no resolution, fps or any other property of the medium: those live in the
site config and the pipeline reads them from there. Thus no property of the concrete video
leaks to the pipeline through this interface.

## Clock and dropping

The media instant of frame `k` is `k / fps_ref`. On each `read()`:

1. Elapsed clock time: `t_now = clock.now() - t0`. The frame corresponding to
   "now" is the one with the highest index whose instant has already passed: `n_due = floor(t_now · fps_ref)`.
2. **Pacing:**
   - If `n_due < next_to_deliver` → consumer **ahead**: the source **waits**
     (`clock.sleep`) until the exact instant of the next frame and delivers it. It does not duplicate
     frames or busy-spin.
   - If `n_due ≥ next_to_deliver` → consumer **on time or behind**: the frame at
     `n_due` is delivered and the intermediate ones are **dropped**.
3. **Advance** of the decoder to the target frame with `grab()` (advances without decoding,
   cheap); only the delivered frame is decoded with `retrieve()`. That is where the cheapness of the
   dropping lies.
4. **Real EOF**: signaled by the decoder (`grab()` returns `False`), not by a config count. The
   last available frame is delivered and the next read ends.
5. **Counters**: `discarded += gap`, `delivered += 1`.

**Guaranteed invariant** (verified by test): when signaling `end_of_source`,
`delivered_frames + discarded_frames == total frames of the segment`. Each media index was
delivered or dropped **exactly once**. This is what makes the drop rate
(`discarded / total`) reliable.

**Why `grab/retrieve` and not `CAP_PROP_POS_FRAMES`:** random seeking jumps to the nearest keyframe
in codecs with inter-frame compression and returns the wrong frame or is extremely slow.
Sequential advance is exact and models a camera (frames forward, never backward).

**Why `Fraction` for the fps:** `355/12` is exact; with `Fraction`, both
`floor(t_now · fps)` and `t = index · 12/355` do not accumulate error over 5325 frames.

**`t0` is set in `open()`**, not in the first `read()`: the camera has been running since it is
turned on. If between opening and the first read more than one period passes, those first frames are
dropped. It is the correct behavior.

## Injectable dependencies

`FileVideoSource` receives two dependencies, with a real implementation by default:

- **`Clock`** (`MonotonicClock` in production: `time.monotonic`/`time.sleep`).
- **`FrameReader`** (`OpenCVFrameReader` in production: `cv2.VideoCapture`).

They are injectable so that the clock/dropping —the part most prone to subtle bugs (off-by-one in
the count, EOF edges)— is verified deterministically and in milliseconds, without tying itself to
a 180 s run and reproducing the same drop pattern as many times as needed. See
`tests/test_file_source.py`.

## Parameters (from config)

Read from `config/sites/<id>/site.yaml` via `load_site_config`:

| Parameter | Source | Use in the module |
|---|---|---|
| `video_ref` | site.yaml | media path (resolved to absolute against the root) |
| `fps_ref` | site.yaml (`"355/12"`) | pacing rhythm and frame↔second mapping |
| `frame_width/height` | site.yaml | resolution sanity warning at open |
| `t0_s`, `t1_s` | site.yaml | `expected_total_frames` (drop denominator) |

The config resolution only triggers a **warning** if it does not match the real one (it is not corrected nor
aborted; the homography is calibrated on the image as is). The resolution is not exposed by the
contract.

## Usage

```python
from digital_shadow.config import load_site_config
from digital_shadow.sources import file_source_from_site

site = load_site_config("rusia")
with file_source_from_site(site) as source:   # open() in __enter__
    while True:
        frame = source.read()
        if frame.end_of_source:
            break
        # ... the consumer (detector) processes frame.image at frame.timestamp_s
    print(source.delivered_frames, source.discarded_frames)
```

## Known limitations

- **Dropping introduces non-determinism in the fidelity**: which frames are dropped depends on
  the machine load. That is why the drop rate is **always reported** and, if it is high, stability is
  verified by repeating the run.
- **A real-time pass of the 180 s segment takes ~180 s of clock** by construction (the source
  is anchored to real time; a fast consumer waits, a slow one drops, but neither
  finishes before 180 s). It is not accidental slowness.
- **`time.sleep`/`monotonic` are not hard real time** (~1 ms of scheduler granularity,
  negligible against the 33.8 ms period).

## Verification

- **Deterministic test** (`tests/test_file_source.py`, 8 cases): invariant, exact count of
  dropped with a simulated slow consumer, EOF at the correct frame, idempotent terminality,
  empty source, timestamp = media time, `read()` before `open()`, release of the reader on
  close. Runs in milliseconds.
- **Verification script** (`scripts/verify_video_source.py`) over the real clip: reports
  delivered/dropped, invariant, drop rate, timestamp of first/last frame, EOF,
  resolution. `--work-ms` simulates the consumer cost; `--max-seconds` bounds the window;
  `--dump-frames` dumps the first/last frame to PNG.

## References

- `digital_shadow.perception` (`docs/modules/detector.md`): the consumer that processes
  `frame.image` at its `frame.timestamp_s`.
- `config/sites/rusia/site.yaml`: site parameters.
- Environment and pinned dependencies: `requirements.txt`.
