"""CANONICAL serializer of the WS v3 contract — shared infrastructure.

Converts a contract MESSAGE (flat dict: hello/step/end) into the canonical bytes
consumed by the emitter log and by layers 2-3 of the anti-leak probe. It operates on
the message dict and NOT on the DecidedTickView: layer 3 (prefix audit) canonicalizes
LOGS, not views.

GUARANTEE: DETERMINISM, not exact decimal precision. Quantization uses Python's fixed
format (f"{x:.Nf}"), which rounds over the BINARY REPRESENTATION of the double: at the
decimal boundaries the result depends on the nearest binary value (evidence pinned in
tests/test_canonical.py: 0.0005->"0.001" and 0.0015->"0.002" round up, 1.0005->"1.000"
rounds down, 2.675 at 2 decimals gives "2.67"). That is correct for reproducibility
(same double -> same bytes, always); "3 decimals = millimeters" is NOT literally exact
at the edges and is not claimed. Fixed format was chosen over round()+repr with
evidence: stable text width ("0.300", not "0.3"), no scientific notation, and
idempotent re-formatting (0 failures over 100k uniform samples).

Pinned rules (full doc in docs/modules/ws_canonical.md):
  1. JSON UTF-8, ensure_ascii (ASCII escapes only), separators ",", ":" without spaces.
  2. Keys ordered (ascending str byte order) at all levels.
  3. Every list under key "entities" is ordered by (id, source).
  4. MEASUREMENT floats: fixed format per field (QUANT_*); "-0.000..." is normalized
     to positive.
  5. CONFIG/constant floats and unlisted fields: exact round-trip repr
     (-0.0 -> "0.0").
  6. NaN/±Inf: forbidden (CanonicalError with the field path).
  7. "t_emit": FORBIDDEN in the canonical bytes (CanonicalError). It is injected later
     with wire_frame(), OVER THE BYTES, without re-serializing: the network frame
     differs from the canonical one exactly in the inserted segment.
  8. Terminator b"\\n": the canonical log is JSONL; the pin of a run is the file's md5.

The module is TIED to the frozen contract: CONTRACT_MD5 pins the md5 of
docs/modules/ws_payload_contract.md; the pin test fails if the contract mutates without
bumping CONTRACT_VERSION (silent mutation = visible breakage, not a surprise).
"""
from __future__ import annotations

import json
import math

CONTRACT_VERSION = "3.0.0"
CONTRACT_DOC = "docs/modules/ws_payload_contract.md"
CONTRACT_MD5 = "e25a04befcc8ef2529ead8e87e444952"   # md5 of the frozen contract

# Quantization by key CONTEXT (contract Section 3.2/Section 3.3). The generic keys
# "x"/"y"/"t" are interpreted ONLY inside "last_pos"; outside that branch an unlisted
# float field falls under rule 5 (exact repr). The table is extended explicitly when
# the contract grows (new version), never by a silent default.
QUANT_SCALAR = {"heading": 4, "t_sim": 4}     # key -> decimals, at any level
QUANT_POS_LIST = {"pos_plane": 3}             # list [x, y] -> decimals per element
QUANT_LAST_POS = {"x": 3, "y": 3, "t": 4}     # only inside "last_pos"

FORBIDDEN_KEYS = frozenset({"t_emit"})


class CanonicalError(ValueError):
    """Violation of the canonical rules (t_emit, NaN/Inf, illegal type or key)."""


def _check_finite(x: float, path: str) -> None:
    if math.isnan(x) or math.isinf(x):
        raise CanonicalError(f"NaN/Inf prohibido en canónico: {path} = {x!r}")


def _fmt_fixed(x: float, nd: int, path: str) -> str:
    """Fixed format .{nd}f (rule 4). Rounding over the binary (half-even of the C
    formatter over the real double): deterministic, not decimal-exact."""
    _check_finite(float(x), path)
    s = f"{float(x):.{nd}f}"
    if s.startswith("-") and float(s) == 0.0:
        s = s[1:]                      # "-0.000" -> "0.000" (unsigned zero)
    return s


def _fmt_free(x: float, path: str) -> str:
    """exact round-trip repr (rule 5) for config/constants and floats with no rule."""
    _check_finite(float(x), path)
    if x == 0.0:
        return "0.0"                   # normalizes -0.0
    return repr(float(x))


def _emit_str(s: str) -> str:
    return json.dumps(s, ensure_ascii=True)


def _entity_sort_key(e, path: str):
    if not isinstance(e, dict):
        raise CanonicalError(f"elemento de entities no es objeto: {path}")
    return (str(e.get("id", "")), str(e.get("source", "")))


def _emit(value, out: list, path: str, quant: int | None = None,
          in_last_pos: bool = False) -> None:
    """Recursive traversal -> canonical tokens in `out`.

    quant: decimals imposed by the CONTEXT of the parent key (or None).
    in_last_pos: enables the QUANT_LAST_POS table for x/y/t of that sub-object.
    """
    if value is None:
        out.append("null")
    elif isinstance(value, bool):              # before int: bool is a subclass
        out.append("true" if value else "false")
    elif isinstance(value, int):
        out.append(str(value))
    elif isinstance(value, float):
        out.append(_fmt_fixed(value, quant, path) if quant is not None
                   else _fmt_free(value, path))
    elif isinstance(value, str):
        out.append(_emit_str(value))
    elif isinstance(value, (list, tuple)):
        out.append("[")
        for i, item in enumerate(value):
            if i:
                out.append(",")
            _emit(item, out, f"{path}[{i}]", quant=quant)
        out.append("]")
    elif isinstance(value, dict):
        out.append("{")
        first = True
        for k in sorted(value.keys()):
            if not isinstance(k, str):
                raise CanonicalError(f"clave no-str en {path}: {k!r}")
            if k in FORBIDDEN_KEYS:
                raise CanonicalError(
                    f"'{k}' prohibido en bytes canónicos ({path}) — se inyecta "
                    f"con wire_frame(), nunca antes (regla 7)")
            if not first:
                out.append(",")
            first = False
            out.append(_emit_str(k))
            out.append(":")
            v = value[k]
            kpath = f"{path}.{k}"
            if k == "entities" and isinstance(v, (list, tuple)):
                v = sorted(v, key=lambda e: _entity_sort_key(e, kpath))
                _emit(v, out, kpath)
            elif k in QUANT_POS_LIST:
                _emit(v, out, kpath, quant=QUANT_POS_LIST[k])
            elif k == "last_pos" and isinstance(v, dict):
                _emit_last_pos(v, out, kpath)
            elif k in QUANT_SCALAR and isinstance(v, float):
                _emit(v, out, kpath, quant=QUANT_SCALAR[k])
            else:
                _emit(v, out, kpath)
        out.append("}")
    else:
        raise CanonicalError(f"tipo no serializable en {path}: {type(value).__name__}")


def _emit_last_pos(d: dict, out: list, path: str) -> None:
    out.append("{")
    first = True
    for k in sorted(d.keys()):
        if k in FORBIDDEN_KEYS:
            raise CanonicalError(f"'{k}' prohibido en bytes canónicos ({path})")
        if not first:
            out.append(",")
        first = False
        out.append(_emit_str(k))
        out.append(":")
        v = d[k]
        nd = QUANT_LAST_POS.get(k)
        _emit(v, out, f"{path}.{k}", quant=nd if isinstance(v, float) else None)
    out.append("}")


def canonical_bytes(msg: dict) -> bytes:
    """v3 contract message (dict) -> canonical bytes (one JSONL line with '\\n').

    Deterministic: same content -> same bytes, in any process and with any
    construction order of the dict. Raises CanonicalError on t_emit, NaN/Inf,
    non-str keys or non-serializable types.
    """
    if not isinstance(msg, dict) or not msg:
        raise CanonicalError("el mensaje canónico debe ser un dict no vacío")
    out: list = []
    _emit(msg, out, "$")
    return ("".join(out) + "\n").encode("utf-8")


def wire_frame(canonical: bytes, t_emit: float) -> bytes:
    """EMITTER thread ONLY: network frame = canonical + t_emit injected OVER THE
    BYTES (without re-serializing): guarantees by construction that the frame differs
    from the canonical one EXACTLY in the inserted segment. t_emit with 6 decimals
    (µs) — it is a wall clock, it never enters the canonical log."""
    _check_finite(float(t_emit), "t_emit")
    if not canonical.startswith(b'{"'):
        raise CanonicalError("wire_frame espera bytes canónicos de un objeto no vacío")
    return b'{"t_emit":' + f"{float(t_emit):.6f}".encode("ascii") + b"," + canonical[1:]
