import { coherenceVerdict } from './hud_core.js';

// hud — DOM overlay that DRAWS, does not compute (contract: docs/modules/ws_hud.md).
// Each number originates in the payload or in the ready-made counters of
// stream/entities; the only aggregates are those of hud_core (its own network/render
// metrics). If a number is not in the contract table, it is missing from the CONTRACT
// — it is not computed here. The provenance legend LIVES here (a single place; before
// inline in dev_entities).

export class Hud {
  constructor(rootEl) {
    this.el = rootEl;
    this.el.innerHTML = `
      <div><b>estado:</b> <span id="hud-estado">WAIT_HELLO</span> <span id="hud-razon"></span></div>
      <div id="hud-sesion">sesión: —</div>
      <div id="hud-step">t_sim: — | seq: — | vehiculos: —</div>
      <div id="hud-perdidas">wireGaps(servidor): 0 | skipped(cliente): 0 | ignoradas(layers): 0 | vclass?: 0</div>
      <div id="hud-latencia">WS→render: — <i>(hasta render_apply; no hay callback de píxeles en WebGL)</i></div>
      <div id="hud-guard">guardia: —</div>
      <table id="hud-occ"><caption id="hud-occ-cap">ocupación (server-side): —</caption></table>
      <div id="hud-veredicto"></div>
      <div id="hud-leyenda">
        <span class="sw" style="background:#ffd633"></span>car
        <span class="sw" style="background:#3396ff"></span>bus
        <span class="sw" style="background:#ff7033"></span>truck |
        provenance: sólido=OBSERVED · normal=INTERPOLATED · naranja=EXTRAPOLATED ·
        azulado=INFERRED · translúcido=UNCERTAIN · gris=HELD · magenta=REACQUIRED*
      </div>
      <div id="hud-leyenda-vision"></div>
      <button id="hud-export">exportar métricas (JSON)</button>`;
    this._last = { hello: null, step: null, end: null, stats: null,
                   metrics: null, counters: null };
    this.el.querySelector('#hud-export').addEventListener('click',
      () => this._download());
  }

  _q(id) { return this.el.querySelector(id); }

  setSession(hello) {
    this._last.hello = hello;
    const os = hello.occupancy_semantics || {};
    this._q('#hud-sesion').textContent =
      `sesión: site=${hello.site} dt=${hello.dt}s layers=${JSON.stringify(hello.layers)} ` +
      `schema=${hello.schema_version} | ocupación: ${os.kind}` +
      (os.halting_speed_mps !== undefined ? ` (halting<${os.halting_speed_mps} m/s)` : '');
    // VISION legend (only if its sources are declared): the anchor encoding
    // (Section 3.7) is visual validation, not decoration; the ghost is the bridge.
    const layers = hello.layers || [];
    const vis = this._q('#hud-leyenda-vision');
    if (layers.includes('vision_box') || layers.includes('handoff')) {
      vis.innerHTML =
        'cola (vision_box): huella llena=<b>anclada a carril</b> · ' +
        'contorno rojo=<b>raw_homography</b> (anclaje CRUDO, error lateral conocido — ' +
        'el sur; NO es métrica) | fantasma gris tenue=<b>handoff</b> ' +
        '(puente caja→vehículo en el hueco; solo pasado)';
    } else {
      vis.innerHTML = '';
    }
  }

  setState(state, reason) {
    const e = this._q('#hud-estado');
    e.textContent = state; e.className = state;
    this._q('#hud-razon').textContent = reason ? `— ${reason}` : '';
    if (state === 'SIN_VEREDICTO') {
      this._q('#hud-veredicto').textContent =
        'CORRIDA SIN VEREDICTO (el stream cerró sin end — nunca PASS implícito)';
    }
  }

  // step from the payload + ready-made counters from stream/entities + hud_core
  // snapshot. The HUD computes none of this: it places it.
  setStep(step, streamStats, layerCounters, vehicleCount, metrics) {
    this._last.step = { seq: step.seq, t_sim: step.t_sim };
    this._last.stats = streamStats;
    this._last.metrics = metrics;
    this._last.counters = {
      ignored: layerCounters.ignored,
      unknownVclass: layerCounters.unknownVclass,
      ignoredSources: [...layerCounters.ignoredSources],
    };
    this._q('#hud-step').textContent =
      `t_sim: ${step.t_sim.toFixed(1)} s | seq: ${step.seq} | vehiculos: ${vehicleCount}`;
    this._q('#hud-perdidas').textContent =
      `wireGaps(servidor): ${streamStats.wireGaps} | skipped(cliente): ${metrics.skippedTotal} | ` +
      `ignoradas(layers): ${layerCounters.ignored} | vclass?: ${layerCounters.unknownVclass}`;
    this._q('#hud-latencia').innerHTML =
      `WS→render: n=${metrics.n} mediana=${fmt(metrics.median_ms)} ` +
      `p95=${fmt(metrics.p95_ms)} max=${fmt(metrics.max_ms)} ms ` +
      `<i>(hasta render_apply; no hay callback de píxeles en WebGL)</i>`;
    const g = step.guard || {};
    this._q('#hud-guard').textContent =
      `guardia: accesos=${g.accesses ?? '—'} violaciones=${g.violations ?? '—'} ` +
      `max_lookahead=${g.max_lookahead_s ?? '—'} s`;
    this._occTable(step.occupancy);
  }

  _occTable(occ) {
    // discriminated union by kind (Section 3.4): sumo_lanes (messages) =
    // occupancy/stopped; pixel_zones (vision) = RAW COUNT per zone/lane, NOT the paper
    // metric.
    const t = this._q('#hud-occ');
    const cap = this._q('#hud-occ-cap');
    const rows = [];
    if (occ.kind === 'pixel_zones') {
      cap.textContent =
        `ocupación (server-side, pixel_zones) — CONTEO CRUDO por zona/carril ` +
        `(${occ.semantics}); NO es la métrica del paper (§3.4/§3.7)`;
      for (const zona of Object.keys(occ.per_zone).sort()) {
        const lanes = occ.per_zone[zona];
        const celdas = Object.keys(lanes).sort().map((c) =>
          `<td>c${c}: ${lanes[c]}</td>`).join('');
        rows.push(`<tr><th>${zona}</th>${celdas}</tr>`);
      }
    } else {   // sumo_lanes (messages) — INTACT; separator "/" (fix 2026-07-05)
      cap.textContent =
        `ocupación (server-side, ${occ.kind}) — celda = ocupación/detenidos`;
      for (const brazo of Object.keys(occ.per_lane).sort()) {
        const lanes = occ.per_lane[brazo];
        const celdas = Object.keys(lanes).sort().map((c) =>
          `<td>c${c}: ${lanes[c].ocupacion}/${lanes[c].detenidos}</td>`).join('');
        rows.push(`<tr><th>${brazo}</th>${celdas}</tr>`);
      }
    }
    t.innerHTML = t.querySelector('caption').outerHTML + rows.join('');
  }

  setEnd(end, streamStats) {
    this._last.end = end;
    // TWO-CASE coherence (hud_core.coherenceVerdict): MISMATCH only from REAL loss in
    // the observed window; entering late is TARDIA, not an error
    const v = coherenceVerdict(streamStats, end);
    const etiqueta = v.caso === 'COMPLETA' ? 'completa desde el inicio'
      : v.caso === 'TARDIA'
        ? `conexión tardía (entró en seq ${streamStats.firstSeq}; ` +
          `${v.lateMiss} steps antes de entrar — no cuentan como pérdida)`
        : 'sin steps recibidos';
    this._q('#hud-veredicto').textContent =
      `END: sonda=${end.sonda.status} | steps=${end.totals.steps} ` +
      `dropped=${end.totals.dropped} emit_errors=${end.totals.emit_errors} | ` +
      `recepción ${etiqueta}: received=${streamStats.received}, faltantes en ` +
      `ventana=${v.missing ?? '—'} (≤ dropped ${end.totals.dropped}) ` +
      `→ ${v.ok ? 'OK' : 'DESAJUSTE'}`;
  }

  exportData() {
    return { exported_at_perf_ms: performance.now(), ...this._last };
  }

  _download() {
    const blob = new Blob([JSON.stringify(this.exportData(), null, 2)],
                          { type: 'application/json' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = 'hud_metricas.json';
    a.click();
    URL.revokeObjectURL(a.href);
  }
}

function fmt(v) { return v === null || v === undefined ? '—' : v.toFixed(1); }
