# Ground truth — Rusia (files and versioning)

> **Current official GT:** **`ground_truth_rusia_v2_carril_processed.csv`** (**v2 by
> lane**, 73 events, with `entry_lane`/`exit_lane`). The pipeline and the metric consume
> this file. The v1 by arm (`_v1_por-brazo/ground_truth_rusia_processed.csv`, 72
> events) is superseded and is kept archived in `_v1_por-brazo/` as historical.

## Why the cardinals are corrected

The GT was labeled against the **pre-rotation** net frame: the arm cardinals
came out **flipped 180°** with respect to the real world (verified visually against the
video). The correction is a swap of cardinals: **NORTE↔SUR** and **ESTE↔OESTE**, applied
only to `entry_arm`/`exit_arm`. The turn (`turn`) is invariant under a 180° rotation
(a proper rotation) and stays identical; `movement_status` is re-derived against the net and
matches; `inclusion` and the times are not cardinals and stay intact.

## Files and where each thing is

**In the root `data/ground_truth/rusia/` (the current material):**

| File | Version | Content | Role |
|---|---|---|---|
| `ground_truth_rusia_v2_carril_processed.csv` | **v2 — by lane** | seconds + **`entry_lane`/`exit_lane`** + correct cardinals + `turn`/`movement_status`/`inclusion` derived; canonical order | **★ Current official.** Consumed by the metric (Phase 3). **73 events.** |
| `ground_russia_tnew.csv` | **v2 — raw** | original delivery: lane + times in seconds; `turn`/`status`/`inclusion` empty | **raw v2, intact** (evidence, original delivery). Source of the official one. Do not consume directly. |
| `README.md` | — | this catalog | — |

**Archived in `_v1_por-brazo/` (v1 by arm — historical, DO NOT consume):**

| File (inside `_v1_por-brazo/`) | Version / convention | Content | Role |
|---|---|---|---|
| `ground_truth_rusia_processed.csv` | v1 — by arm (corrected) | seconds + corrected cardinals + derived; **no lane** | **Superseded by v2.** Kept. 72 events. Do not consume. |
| `ground_truth_rusia.csv` | v1 — raw working (corrected) | raw working (timecode), no lane | v1 historical. Kept. |
| `ground_truth_rusia_raw.csv` | v1 — pristine corrected | pristine corrected (timecode), no lane | v1 historical. Kept. |
| `ground_truth_rusia_raw_VIEJA-convencion-pre-rotacion.csv` | v1 — **OLD (flipped)** | original labeling as is (timecode, flipped convention) | **historical — NOT field truth.** Do not consume. |

The v1 `raw` and the working one share **the same md5**. The four v1 files reside in
`_v1_por-brazo/` as historical; none of them is consumed.

## How it was generated (reproducible)

**v1 — orientation correction:** `scripts/correct_ground_truth_orientation.py` applies the
per-field swap, re-derives `turn` (by geometry **and** by the net connections,
independently) and `movement_status` (against the net), requires them to match the original and
**aborts if not**. It writes only if all checks pass (0 discrepancies). It is an
artifact already run, with a guard that **prevents re-running it** (re-running it would re-flip
already-corrected cardinals; its paths point to `_v1_por-brazo/`).

**v2 — derivation by lane:** `scripts/derive_ground_truth_v2_lanes.py` takes the raw
`ground_russia_tnew.csv` (**already in the correct convention** — this is why it does **NOT** go through the
orientation-correction script, which would re-flip it), derives
`turn`/`movement_status`/`inclusion` with the **same method** (turns.csv ↔ geometry ↔ net) and
writes the official one in canonical order. Fail-fast checks: triple coherence of `turn` +
**cross-check against v1** (0 discrepancies in the shared pairs). The count rises to 73
events: the additional event (`norte→sur`) is real. The raw `ground_russia_tnew.csv` stays
**intact** (md5 verified before/after).

## Lane convention

For the GT that carries lane (`entry_lane` / `exit_lane`), the numbering is: **lane 1 = the
rightmost one for the vehicle in its direction of travel**, increasing toward the left;
`entry_lane` (approaching) and `exit_lane` (moving away) are **opposite frames** and **are not
compared as 1 = 1**. It is the numbering of the **raw GT** (not SUMO, not image bands: those
are later stages). **Read it before touching any lane field.**
