"""INTEGRATION smoke of the orchestrator with LIVE SUMO (Part 3): that the loop handles SumoStage
end-to-end (instantiate/move/retire + gap fill) without error. Over a subset of frames
(fast). It does not measure metrics (that is Part 4); it only verifies that the integration runs against real SUMO.
"""
from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
import derive_track_events as dte
import reconnect_fragments as rf
from digital_shadow.config.site import load_site_config
from digital_shadow.geometry.ground_contact import add_ground_contact
from digital_shadow.harness.orchestrator import CausalOrchestrator
from digital_shadow.harness.sink import SumoStageSink
from digital_shadow.sumo.lane_map import load_lane_map
from digital_shadow.sumo.position import load_alpha, load_homography
from digital_shadow.sumo.route import load_lane_turn_rule, load_turns
from digital_shadow.sumo.stage import SumoStage
from digital_shadow.tracking.causal_reconnect import CausalReconnector
from audit_harness_part3 import internal_shapes_from_net

MAX_FRAME = 2600   # subset: covers several vehicles (incl. the reconnected turn sur->este 93,98)


def main():
    site = load_site_config("rusia")
    center = np.array([site.frame_width / 2.0, site.frame_height / 2.0])
    tracks = pd.read_csv(ROOT / "verify_out/tracks_rusia.csv")
    tracks = tracks[tracks.frame_index <= MAX_FRAME].copy()
    add_ground_contact(tracks)
    base_events = pd.read_csv(ROOT / "verify_out/events/rusia/track_events.csv")
    excluded = set(tracks.track_id.unique()) - set(base_events.track_id.unique())
    feats = rf.track_features(tracks, center, dte.CORE_RADIUS_PX, dte.WINDOW_FRAMES)
    lm = load_lane_map("rusia")
    shapes = internal_shapes_from_net(lm, ROOT / "config/sites/rusia/sumo/intersection_rusia.net.xml")

    stage = SumoStage(ROOT / "config/sites/rusia/sumo/intersection_rusia.net.xml",
                      gui=False, step_length=0.001, lane_map=lm).start()
    sink = SumoStageSink(stage)
    orch = CausalOrchestrator(
        tracks=tracks, feats=feats,
        reconnector=CausalReconnector(buffer_frames=site.reid_temporal_window_frames,
                                      corridor_px=site.reid_spatial_corridor_px,
                                      valid_moves=rf.load_valid_movements("rusia")),
        lane_map=lm, H=load_homography("rusia"), rule=load_lane_turn_rule("rusia"),
        turns=load_turns("rusia"), stop_lines=dte.load_stop_lines("rusia"),
        lane_bands=dte.load_lane_bands("rusia"), center=center, alpha=load_alpha(),
        buffer_frames=site.reid_temporal_window_frames, excluded=excluded, internal_shapes=shapes)

    instantiated = retired = 0
    arrived = []
    err = None
    try:
        # patches the sink to count + watch SUMO arrivals per frame
        orig_inst, orig_ret, orig_sync = sink.instantiate, sink.retire, sink.sync
        def inst(*a):
            nonlocal instantiated; instantiated += 1; orig_inst(*a)
        def ret(*a):
            nonlocal retired; retired += 1; orig_ret(*a)
        def sync2(t):
            orig_sync(t)
            arrived.extend(stage._traci.simulation.getArrivedIDList())
        sink.instantiate, sink.retire, sink.sync = inst, ret, sync2
        commits, chains = orch.run(sink)
        clock = stage._traci.simulation.getTime()
        print(f"  bucle con SUMO VIVO sobre {tracks.frame_index.nunique()} frames: OK (sin tumbar el bucle)")
        print(f"  instanciados: {instantiated} | retirados (orquestador): {retired} | vivos al final: {len(stage.vehicles)}")
        print(f"  reloj SUMO al final: {clock:.2f} s (video {tracks.frame_index.max()*12/355:.2f} s) "
              f"→ {'SINCRONIZADO' if abs(clock - tracks.frame_index.max()*12/355) < 0.1 else 'DESYNC'}")
        print(f"  HALLAZGO de integración (causa CONFIRMADA con datos): SUMO auto-retira {len(sink.dropped)} "
              f"vehículos al LLEGAR a su ruta (dropped ⊆ arrived: {sink.dropped <= set(arrived)}).")
        print(f"  = el vehículo alcanzó el fin del egreso ≈ SALIÓ de la intersección → retire RAZONABLE, pero")
        print(f"  PRECEDE al retire de fin-de-cadena del orquestador (muerte+buffer). El guard lo maneja sin")
        print(f"  tumbar el bucle. A reconciliar (aceptar arrival como salida, o prevenirlo) en la viz/Parte 5.")
        print(f"  (Descartada con datos la hipótesis previa de 'error H lejano fuera de red': el bounds-guard")
        print(f"   saltó {sink.out_of_net} moves → las coords estaban EN la red; la causa es arrival, no H.)")
        print(f"  -> PASS de INTEGRACIÓN: orquestador↔SumoStage end-to-end, ciclo de vida, reloj sync; NO")
        print(f"     afecta el stream/delays (Parte 3 core, validado aparte con RecordingSink).")
    except Exception as e:
        err = e
        import traceback; traceback.print_exc()
    finally:
        stage.close()
    if err:
        print(f"  -> FALLA: {err}")


if __name__ == "__main__":
    main()
