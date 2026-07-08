# Vendored Three.js — r180 (npm three@0.180.0)

Single local copy (no CDN, no bundler; ES modules + import map — the path
documented by the official installation manual). Chosen for the STABILITY of the
API used, not for its date: the official migration guide records no changes in
WebGLRenderer or OrbitControls between r178 and r185 (the only deprecation in the
range: PCFSoftShadowMap in r182 — the client does not use shadow maps);
WebGLRenderer has no announced removal toward WebGPURenderer; r180 has five later
releases of public field use with the same API surface. Path verification done
against the real npm tarball: `build/three.module.js`,
`examples/jsm/controls/OrbitControls.js` (imports `'three'` bare → the import map
resolves it), `LICENSE` (MIT).

Source: https://registry.npmjs.org/three/-/three-0.180.0.tgz

RUNTIME IMPORT GRAPH: the r180 build is SPLIT — `three.module.js` imports and
re-exports from `./three.core.js` (lines 6-7 of the build; verified in the
tarball). `three.core.js` is a LEAF (zero import/export-from). The vendor must
cover the FULL graph; `tests/test_web_imports.py` watches over it (static
resolution of the graph from each HTML).

md5 of each vendored file (full graph):

```
e3a10386281e96895ecedc84238261eb  three.module.js
22184b6508211a30dd3ec752297fa999  three.core.js
c9adb791476ceba9aa02f33205031df7  addons/controls/OrbitControls.js
d0c607a5dd111060bd8feb787f5cde4d  LICENSE
```

Rule: do NOT mix files from another version or source (official warning of the
manual); any update replaces the entire vendor, updates these md5 and the module
doc (docs/modules/ws_net_scene.md), and is re-validated visually.
