# ws_emitter — 3D web viewer

A 3D web viewer common to both shadows (vision and V2I messages): the same page
animates either of them; only the scene and the port change. There are two processes:
an HTTP webserver that serves the static page and generates `/config.json`, and a
WebSocket emitter that publishes the shadow's state per tick so the client renders it.

The front-end lives in `web/` (`index.html` plus the JS modules in `web/js/`: network
scene, entities, HUD and stream client) and uses **Three.js vendored offline** in
`web/vendor/three/` (no CDN, suitable for offline use). The per-site scenes
(`scene_rusia.json`, `scene_vision_rusia.json`) are derived from the `.net.xml` and
served static. The package's Python modules cover the webserver, the WebSocket emitter,
the payload canonicalization and the stream verification probes.
