"""Smoke test of the web client IMPORT GRAPH (3D client front-end).

Guards against vendoring failures: if three r180 omits three.core.js, the build
is SPLIT and three.module.js imports it at runtime.
The previous smoke test ("N known resources return 200" + node --check) did not
catch it because it validated a hand-written list and syntax, not import
RESOLUTION.

This test starts from each client HTML, extracts the import map and the modules
(src and inline), and follows the `import/export ... from '...'` RECURSIVELY,
resolving bare specifiers against the import map and relative ones against the
importing file. Every resolved file MUST exist. Declared limit: STATIC
resolution (a dynamic import() is not followed; three r180 and our modules do
not use them — re-evaluate if any module introduces them).

pytest tests/test_web_imports.py from the implementacion/ root.
"""
from __future__ import annotations

import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WEB = ROOT / "ws_emitter" / "web"

# import/export ... from '<spec>'  |  import '<spec>'  (side-effect)
_IMPORT_RE = re.compile(
    r"""(?:^|\n)\s*(?:import|export)\s+(?:[^'"]*?\sfrom\s+)?['"]([^'"]+)['"]""")
_IMPORTMAP_RE = re.compile(
    r'<script\s+type="importmap">(.*?)</script>', re.S)
_MODULE_SRC_RE = re.compile(
    r'<script\s+type="module"\s+src="([^"]+)"')
_MODULE_INLINE_RE = re.compile(
    r'<script\s+type="module">(.*?)</script>', re.S)
# data resources: any .json literal in the scripts (covers fetch('x.json')
# and also loadScene('./x.json') — the URL travels as an argument)
_FETCH_RE = re.compile(r"""['"]([^'"]+\.json)['"]""")
# VIRTUAL resources: they do not exist on disk because the webserver GENERATES them
# (config.json comes from web3d.yaml — single source; ws_emitter/webserver.py)
_VIRTUAL = {"config.json", "./config.json"}


def _resolve(spec: str, importer_dir: Path, importmap: dict) -> Path:
    if spec.startswith("./") or spec.startswith("../"):
        return (importer_dir / spec).resolve()
    for prefix, target in importmap.items():
        if prefix.endswith("/") and spec.startswith(prefix):
            return (WEB / (target.lstrip("./") + spec[len(prefix):])).resolve()
        if spec == prefix:
            return (WEB / target.lstrip("./")).resolve()
    raise AssertionError(f"especificador sin resolución: {spec!r} "
                         f"(¿falta en el import map?)")


def _walk_module(path: Path, importmap: dict, seen: set, missing: list):
    if path in seen:
        return
    seen.add(path)
    if not path.exists():
        missing.append(str(path.relative_to(WEB)) if WEB in path.parents
                       else str(path))
        return
    for spec in _IMPORT_RE.findall(path.read_text(encoding="utf-8")):
        _walk_module(_resolve(spec, path.parent, importmap), importmap,
                     seen, missing)


def _graph_from_html(html: Path):
    text = html.read_text(encoding="utf-8")
    m = _IMPORTMAP_RE.search(text)
    importmap = json.loads(m.group(1))["imports"] if m else {}
    seen, missing = set(), []
    for src in _MODULE_SRC_RE.findall(text):
        _walk_module((html.parent / src).resolve(), importmap, seen, missing)
    for inline in _MODULE_INLINE_RE.findall(text):
        for spec in _IMPORT_RE.findall(inline):
            _walk_module(_resolve(spec, html.parent, importmap), importmap,
                         seen, missing)
        for url in _FETCH_RE.findall(inline):
            if url in _VIRTUAL:
                continue
            f = (html.parent / url).resolve()
            seen.add(f)
            if not f.exists():
                missing.append(url)
    return seen, missing


import pytest

_HTMLS = sorted(WEB.glob("*.html"))


@pytest.mark.parametrize("html", _HTMLS, ids=lambda p: p.name)
def test_grafo_imports_completo(html):
    """ALL client HTMLs: every file in the import graph exists."""
    seen, missing = _graph_from_html(html)
    assert not missing, (
        f"{html.name}: archivos del grafo AUSENTES: {missing}")


@pytest.mark.parametrize("html", _HTMLS, ids=lambda p: p.name)
def test_todo_el_grafo_es_servible_por_rutas_relativas(html):
    """No file in the graph can fall outside ws_emitter/web (the static server
    only serves that directory)."""
    seen, _ = _graph_from_html(html)
    fuera = [str(p) for p in seen if WEB not in p.parents]
    assert not fuera, f"{html.name}: recursos fuera del directorio servido: {fuera}"


def test_dev_scene_llega_al_build_partido():
    """The dev_scene graph must cover the SPLIT build of three r180."""
    seen, _ = _graph_from_html(WEB / "dev_scene.html")
    nombres = {p.name for p in seen}
    assert {"three.core.js", "three.module.js", "OrbitControls.js",
            "net_scene.js", "scene_rusia.json"} <= nombres


def test_dev_stream_llega_a_stream_js():
    seen, _ = _graph_from_html(WEB / "dev_stream.html")
    assert "stream.js" in {p.name for p in seen}
