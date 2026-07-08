"""Vehicle reference point: the GROUND CONTACT.

CONTRACT (single, stable piece; main consumer = the downstream zones layer):

  The reference point of a vehicle for classifying lane/arm and for projecting to SUMO
  with the homography is the GROUND CONTACT = (gx, gy) = ((x1 + x2) / 2, y2): the center
  of the BOTTOM EDGE of the detection box, in image pixels.

  - x = (x1 + x2) / 2  : horizontal center of the box (same as `cx`).
  - y = y2             : BOTTOM edge of the box (NOT the vertical center `cy`).

  Why the bottom edge and not the center: the ground contact is on the asphalt plane,
  the SAME plane for all vehicles regardless of their height. That removes the height
  parallax introduced by the centroid (a bus and a car with the same ground contact have
  centroids at different heights) and is consistent with the single-plane assumption of
  the image→ground homography.

  THE CENTROID (cx, cy) IS NOT THE REFERENCE POINT. It exists in the tracking CSV only as
  a visualization convenience; using it to classify lane or project to SUMO introduces the
  parallax this point avoids. This ambiguity (two possible points) is settled here: the
  system contract is the ground contact.

Input/output:
  - `ground_contact(x1, x2, y2)` is VECTORIZED: it accepts scalars, `np.ndarray`, or
    `pd.Series` (the arithmetic broadcasts) and returns `(gx, gy)` of the same type.
  - `add_ground_contact(df)` adds the canonical columns `GROUND_X`/`GROUND_Y`
    (= "gx"/"gy") to a DataFrame that has the raw corners `x1, x2, y2`.

Canonical column names: `GROUND_X = "gx"`, `GROUND_Y = "gy"` (a UNIQUE name across the
whole repo; before there was `gx/gy` in some scripts and `ground_x/ground_y` in another).

It does not depend on pandas at import time (only `add_ground_contact` uses it, via the
columns); the core function is pure arithmetic.
"""

from __future__ import annotations

GROUND_X = "gx"
GROUND_Y = "gy"


def ground_contact(x1, x2, y2):
    """Ground contact (gx, gy) = ((x1 + x2) / 2, y2) from the raw corners.

    Vectorized: `x1, x2, y2` can be scalars, `np.ndarray`, or `pd.Series`. Returns the
    tuple `(gx, gy)` preserving the input type (the division and the sum broadcast
    element-wise). `gy` is exactly `y2` (bottom edge).
    """
    gx = (x1 + x2) / 2.0
    gy = y2
    return gx, gy


def add_ground_contact(df, x_col: str = GROUND_X, y_col: str = GROUND_Y):
    """Add the ground-contact columns (`gx`, `gy`) to `df` and return it.

    `df` must have the raw columns `x1`, `x2`, `y2` (format of the tracking CSV from
    `run_tracker.py`). Mutates `df` in place (adds columns) and returns it for chaining
    convenience. It does not touch `cx`/`cy` (centroid), which is NOT the reference point
    (see the contract above).
    """
    gx, gy = ground_contact(df["x1"], df["x2"], df["y2"])
    df[x_col] = gx
    df[y_col] = gy
    return df
