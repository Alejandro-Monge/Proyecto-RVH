# Module — Canonical serializer of the WS v3 contract (`ws_emitter/canonical.py`)

Serializes the 3D line of the WS v3 contract. The module is shared by the message emitter and
the layers that audit the canonical log: if the canonicalization diverged between consumers, the
audit would give false positives, which is why it lives in its own module and not as an internal
convention of the emitter.

## Method

`canonical_bytes(msg) → bytes`: converts a v3 contract message (flat dict:
`hello`/`step`/`end`) into a canonical JSONL line (UTF-8 bytes plus `\n`). It is deterministic:
the same content produces the same bytes, in any process and with any order of
construction of the dict.

`wire_frame(canonical, t_emit) → bytes`: only for the emitter thread; produces the network frame
injecting `t_emit` over the canonical bytes, without reserializing.

It operates on the message dict, not on the `DecidedTickView`: the audit layer canonicalizes
logs, not views. The view→message mapping is provided by the upper layers and composes with this
module.

## Input and output

- **Input:** a non-empty dict with `str` keys and `None|bool|int|float|str|list|dict` values.
  Any other type produces `CanonicalError` with the field path.
- **Output:** bytes of a valid JSON line ending in `\n`. The canonical log of a
  run is the concatenation of lines; its pin is the md5 of the file.
- **Errors (`CanonicalError`):** `t_emit` at any level; `NaN`/`±Inf`; non-str key;
  non-serializable type; empty message.

## Canonicalization rules

1. UTF-8 JSON, `ensure_ascii` (only ASCII escapes), separators `","`/`":"` without spaces.
2. Keys sorted (ascending byte order) at all levels.
3. Every list under the key `entities` is sorted by `(id, source)`.
4. Measurement floats: fixed format `f"{x:.Nf}"` per field (table below); a result
   `-0.000…` is normalized to positive.
5. Config or constant floats and fields with no rule: exact round-trip `repr` (`-0.0` →
   `"0.0"`). Normative example: `dt` is not quantized (it is an exact config constant, not a
   measurement).
6. `NaN`/`±Inf` forbidden.
7. `t_emit` forbidden in the canonical (enforced by exception, not by convention); it is
   injected with `wire_frame`.
8. Terminator `\n` (JSONL).

### Quantization table (rule 4)

| Context | Decimals | Note |
|---|---|---|
| `pos_plane[*]` | 3 | matches the rounding of the repo readouts (`round(c,3)`, dense `%.3f`) |
| `last_pos.x`, `last_pos.y` | 3 | the keys `x`/`y`/`t` are only interpreted inside `last_pos` |
| `last_pos.t`, `t_sim` | 4 | the player grid rounds to 4 (`causal_player.py:440`) |
| `heading` | 4 | 1e-4 rad ≈ 0.006° |

A float field not listed falls under rule 5. The table is extended explicitly with each
version of the contract, never by a silent default.

## Guarantee: determinism, not exact decimal precision

The fixed format rounds over the binary representation of the double. At the decimal
boundaries the direction depends on the nearest binary, not on the decimal rule learned in
school. The behavior is fixed in
`tests/test_canonical.py::test_fronteras_redondeo_pineadas`:

```
0.0005 → "0.001"  (up: binary > half)          1.0005 → "1.000"  (DOWN: binary < half)
0.0015 → "0.002"  (up)                          2.675 to 2 dec → "2.67" (the classic)
-0.0, -0.0004 → "0.000" (zero without sign)
```

"3 decimals = millimeters" is not literally exact at the edges and it is not claimed. For
reproducibility it is correct: the same double produces the same bytes, always.

Fixed format instead of `round()`+`repr` (justified with evidence, not by default): same
boundary behavior (both operate on the binary), but the fixed format gives a stable textual
width (`"0.300"`, not `"0.3"`), never scientific notation (`repr(round(1.2e-05,4))`
= `"0.0"` versus small cases that in repr give `1e-05`), and verified idempotent
re-formatting (0 failures in 100 000 uniform samples: a re-formatted `float(f"{x:.3f}")`
reproduces the same string, a property that sustains the round-trip of `wire_frame`).

## wire_frame: injection over bytes, without reserializing

```
wire = b'{"t_emit":<%.6f>,' + canonical[1:]
```

Injection over bytes was chosen over `loads`→add→`dumps` with canonical rules
because it guarantees by construction that the network frame differs from the canonical exactly in the
inserted segment: there is no reserialization that could diverge if the rules evolve, and the
emitter thread does not pay an extra parse plus dump per frame. The `test_wire_frame_round_trip` test
also verifies the required strong property: `canonical_bytes(loads(wire) without t_emit) ==
canonical_bytes(m)` byte for byte (sustained by the re-formatting idempotence of rule 4 and
the `repr` round-trip of rule 5).

## Pin to the frozen contract

`CONTRACT_VERSION = "3.0.0"` and `CONTRACT_MD5 = e25a04be…` are the freeze pins of the
contract. `test_pin_contrato_congelado` fails if `ws_payload_contract.md` changes without bumping the
version and re-pinning: a silent mutation of the contract breaks the test instead of going
unnoticed.

## Relation with the canonical log

The emitter writes `canonical_bytes()` to the JSONL log inside the player loop (before
enqueueing): the log is the reproducible truth; the wire is best-effort presentation. The audit
of the log compares logs of truncated runs, so everything excluded from the canonical
(`t_emit`) stays out of the hash, as the payload contract fixes.

## Verifications

`pytest tests/test_canonical.py`: 7/7 pass. It covers direct idempotence; equivalence between
different constructions (insertion order, inverted entities, floats by different arithmetic
paths); fixed boundaries; rejections; wire round-trip byte for byte; stability
across processes with `PYTHONHASHSEED` crossed (1 versus 20260704); and the contract pin.

## References

- `docs/modules/ws_payload_contract.md`: the v3 contract that this module serializes.
- `docs/modules/ws_emitter_impl.md`: the emitter that produces the canonical log and the wire.
- `docs/modules/decided_tick_view.md`: the view that the upper layers map to a message.
