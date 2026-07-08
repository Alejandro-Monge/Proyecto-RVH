// TEST SCAFFOLDING (node >= 22, undici global WebSocket): exercises the
// REAL STATE MACHINE of stream.js outside the browser — the same file
// the client serves, no copy or port. Orchestrated by tests/test_stream_protocol.py.
//
// Usage: node stream_harness.mjs url=ws://127.0.0.1:PORT [expect=<md5>]
//        [waitPortMs=20000] [timeoutMs=300000]
// Waits for the port to accept TCP (the server may take a while to come up),
// connects, consumes takeLatest() at 100 Hz (observable coalescing) and on reaching
// a terminal state prints ONE JSON line with state/reason/stats/hello/end.
import net from 'node:net';
import { StreamClient } from '../ws_emitter/web/js/stream.js';

const args = Object.fromEntries(
  process.argv.slice(2).map((a) => [a.slice(0, a.indexOf('=')), a.slice(a.indexOf('=') + 1)]));
const url = new URL(args.url);
const deadline = Date.now() + Number(args.waitPortMs ?? 20000);

function tryPort() {
  return new Promise((res) => {
    const s = net.connect({ host: url.hostname, port: Number(url.port) },
      () => { s.destroy(); res(true); });
    s.on('error', () => { s.destroy(); res(false); });
  });
}
while (!(await tryPort())) {
  if (Date.now() > deadline) {
    console.log(JSON.stringify({ state: 'TIMEOUT', reason: 'puerto nunca abrió' }));
    process.exit(1);
  }
  await new Promise((r) => setTimeout(r, 100));
}

const sc = new StreamClient(args.url);
if (args.expect) {
  sc.onHello((h) => {
    if (h.net_md5 !== args.expect) {
      sc.reject(`net_md5 ${h.net_md5.slice(0, 8)}… != esperado ${args.expect.slice(0, 8)}…`);
    }
  });
}
let consumed = 0, skippedTotal = 0;
const pump = setInterval(() => {
  const g = sc.takeLatest();
  if (g) { consumed++; skippedTotal += g.skipped; }
}, 10);

const TERMINAL = new Set(['ENDED', 'SIN_VEREDICTO', 'RECHAZADO']);
sc.onStateChange((st) => {
  if (!TERMINAL.has(st)) return;
  clearInterval(pump);
  const g = sc.takeLatest();
  if (g) { consumed++; skippedTotal += g.skipped; }
  console.log(JSON.stringify({
    state: sc.state, reason: sc.reason, stats: sc.stats(),
    hello: sc.hello ? { net_md5: sc.hello.net_md5, dt: sc.hello.dt,
                        layers: sc.hello.layers } : null,
    end: sc.end ? { status: sc.end.sonda.status, seq_final: sc.end.seq_final,
                    totals: sc.end.totals } : null,
    consumed, skippedTotal,
  }));
  process.exit(0);
});
sc.connect();
setTimeout(() => {
  console.log(JSON.stringify({ state: 'TIMEOUT', reason: 'sin estado terminal',
                               stats: sc.stats() }));
  process.exit(1);
}, Number(args.timeoutMs ?? 300000));
