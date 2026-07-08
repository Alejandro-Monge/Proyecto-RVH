// stream.js — WebSocket consumer of the v3.0.0 contract (messages profile).
// PROTOCOL state machine, not rendering: it does not touch net_scene, does not create
// meshes, does not compute domain, does not reorder (WS is FIFO per connection,
// RFC 6455), does not interpolate. Doc: docs/modules/ws_stream.md.
//
// States: WAIT_HELLO -> STREAMING -> ENDED | SIN_VEREDICTO | RECHAZADO.
//   - step/end before hello -> RECHAZADO (contract Section 3).
//   - schema_version != 3.0.0 -> RECHAZADO.
//   - close without end -> SIN_VEREDICTO (never an implicit PASS).
//   - net_md5: it is EXPOSED in hello; the comparison lives in main.js, which on
//     failure pushes the machine with reject() — the only external path to RECHAZADO.
//   - Terminals: later messages are ignored and COUNTED (postTerminal).
//
// Loss accounting, SEPARATE and honest:
//   - wireGaps: seq gaps in what is RECEIVED = SERVER drops (the dropped that the end
//     declares; the canonical log does not have them, the wire does).
//   - skipped (in takeLatest): received steps that the painting did not consume =
//     CLIENT coalescing. Two distinct losses, two counters.
//   Coherence identity (case 7, Israel's precision): when hello and end arrive,
//   received == end.totals.steps − end.totals.dropped, EXACT — TCP delivers everything
//   sent before the orderly close; dropped counts what was never sent. The
//   scaffolding/hud verifies it, here only the data is exposed.

const SCHEMA = '3.0.0';
const TERMINAL = new Set(['ENDED', 'SIN_VEREDICTO', 'RECHAZADO']);

export class StreamClient {
  constructor(url) {
    this.url = url;                  // from config (web3d.yaml) via main.js — no hardcode here
    this.state = 'WAIT_HELLO';
    this.hello = null;
    this.end = null;
    this.reason = null;
    this._ws = null;
    this._buf = null;                // "latest received": last-writer-wins
    this._skippedSinceTake = 0;
    this._received = 0;
    this._firstSeq = null;
    this._lastSeq = null;
    this._wireGaps = 0;
    this._postTerminal = 0;
    this._cbs = { state: [], hello: [], end: [] };
  }

  connect() {
    this._ws = new WebSocket(this.url);
    // the emitter sends BINARY frames (canonical bytes + injected t_emit):
    // arraybuffer to decode without promises (Blob would force async)
    this._ws.binaryType = 'arraybuffer';
    this._ws.onmessage = (ev) => this._onMessage(ev);
    this._ws.onclose = () => this._onClose();
    // an error also terminates: in some runtimes an aborted handshake (server dead
    // mid-way) may not reach close — _onClose is idempotent
    this._ws.onerror = () => this._onClose();
    return this;
  }

  reject(reason) {                   // external path (main.js: net_md5 mismatch)
    this._set('RECHAZADO', reason);
    try { this._ws && this._ws.close(); } catch (e) { /* already closed */ }
  }

  takeLatest() {                     // consumed by the paint loop
    if (this._buf === null) return null;
    const out = { ...this._buf, skipped: this._skippedSinceTake };
    this._buf = null;
    this._skippedSinceTake = 0;
    return out;
  }

  stats() {
    return { received: this._received, firstSeq: this._firstSeq,
             lastSeq: this._lastSeq, wireGaps: this._wireGaps,
             postTerminal: this._postTerminal };
  }

  onStateChange(cb) { this._cbs.state.push(cb); }
  onHello(cb) { this._cbs.hello.push(cb); }
  onEnd(cb) { this._cbs.end.push(cb); }

  // ------------------------------------------------------------------ internal
  _set(state, reason = null) {
    if (TERMINAL.has(this.state)) return;
    this.state = state;
    this.reason = reason;
    for (const cb of this._cbs.state) cb(state, reason);
  }

  _onMessage(ev) {
    const tRecv = performance.now();          // WS->render mark, reception side
    if (TERMINAL.has(this.state)) { this._postTerminal++; return; }
    let msg;
    try {
      const text = (ev.data instanceof ArrayBuffer)
        ? new TextDecoder().decode(ev.data) : ev.data;
      msg = JSON.parse(text);
    } catch (e) {
      this._set('RECHAZADO', 'JSON inválido en el wire');
      return;
    }
    const type = msg && msg.type;

    if (this.state === 'WAIT_HELLO') {
      if (type !== 'hello') {
        this._set('RECHAZADO', `${type} antes de hello (contrato §3)`);
        return;
      }
      if (msg.schema_version !== SCHEMA) {
        this._set('RECHAZADO',
                  `schema_version ${msg.schema_version} != ${SCHEMA}`);
        return;
      }
      this.hello = Object.freeze(msg);
      this._set('STREAMING');
      for (const cb of this._cbs.hello) cb(this.hello);
      return;
    }

    // STREAMING
    if (type === 'hello') { this._set('RECHAZADO', 'hello duplicado'); return; }
    if (type === 'step') {
      if (typeof msg.seq !== 'number' || typeof msg.t_sim !== 'number') {
        this._set('RECHAZADO', 'step malformado (sin seq/t_sim)');
        return;
      }
      this._received++;
      if (this._firstSeq === null) this._firstSeq = msg.seq;
      if (this._lastSeq !== null && msg.seq > this._lastSeq + 1) {
        this._wireGaps += msg.seq - this._lastSeq - 1;   // server drops
      }
      this._lastSeq = msg.seq;
      if (this._buf !== null) this._skippedSinceTake++;  // client coalescing
      this._buf = { step: msg, tRecv };
      return;
    }
    if (type === 'end') {
      this.end = Object.freeze(msg);
      this._set('ENDED');
      for (const cb of this._cbs.end) cb(this.end);
      return;
    }
    this._set('RECHAZADO', `type desconocido: ${type}`);
  }

  _onClose() {
    if (TERMINAL.has(this.state)) return;
    this._set('SIN_VEREDICTO',
              this.state === 'WAIT_HELLO' ? 'conexión cerrada sin hello'
                                          : 'conexión cerrada sin end');
  }
}
