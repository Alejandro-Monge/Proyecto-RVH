"""COCO class name↔id mapping, derived from the model itself (not hardcoded).

The detector emits COCO ids; the site defines its active filter by NAMES (`class_set`
in `site.yaml`). This module translates names → ids using the `names` map that the
loaded model brings (authoritative), so that no COCO id is written by hand in the logic.
If a site names a class the model does not know, it fails explicitly (better than
silently filtering by the wrong class).
"""

from __future__ import annotations

from typing import Mapping


def names_to_ids(class_names: list[str], model_names: Mapping[int, str]) -> list[int]:
    """Translate a list of class names to their COCO ids according to `model_names`.

    `model_names` is the id→name map the model exposes (e.g. `YOLO(...).names`). Returns
    the ids in the same order, without duplicates. Raises `ValueError` if some name does
    not exist in the model's vocabulary.
    """
    name_to_id = {name: idx for idx, name in model_names.items()}
    unknown = [n for n in class_names if n not in name_to_id]
    if unknown:
        raise ValueError(
            f"Clases del sitio ausentes en el vocabulario del modelo: {unknown}. "
            f"Vocabulario disponible: {sorted(name_to_id)}"
        )
    ids: list[int] = []
    for name in class_names:
        cid = name_to_id[name]
        if cid not in ids:
            ids.append(cid)
    return ids
