// replay_ws3d.mjs — HEADLESS replay of the ws_log → one PNG per frame (3D panel).
// Applies the ws_log steps IN ORDER on dev_replay.html (a replica of the index.html
// render) and captures. Deterministic: swiftshader + fixed camera + discrete render.
//
// The frame→step mapping is decided by the caller (VISION 1:1; MESSAGES last-step ≤
// t_r−delay_s). Here: --steps "i,j,k" renders those step indices (spot-check), or
// --all renders one per step. Reconcile is stateful → ALL steps are applied
// in order up to the target (no skipping), as the client would do when walking the log.
//
//   node replay_ws3d.mjs --ws-log <log> --url <dev_replay.html> --out <dir> \
//        --width 1280 --height 720 [--steps 100,200,300 | --all]
import fs from 'node:fs';
import path from 'node:path';
import puppeteer from 'puppeteer';

function arg(name, def) {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && i + 1 < process.argv.length ? process.argv[i + 1] : def;
}
const WSLOG = arg('ws-log');
const URL = arg('url');
const OUT = arg('out');
const W = parseInt(arg('width', '1280'), 10);
const H = parseInt(arg('height', '720'), 10);
const ALL = process.argv.includes('--all');
const STEPS = arg('steps', '');

if (!WSLOG || !URL || !OUT) { console.error('faltan --ws-log/--url/--out'); process.exit(2); }
fs.mkdirSync(OUT, { recursive: true });

// ---- load ws_log: hello (1st type=hello line) + steps in order ----
const lines = fs.readFileSync(WSLOG, 'utf8').split('\n').filter(Boolean).map(JSON.parse);
const hello = lines.find((m) => m.type === 'hello');
const steps = lines.filter((m) => m.type === 'step');       // already in emission order (seq)
if (!hello) { console.error('ws_log sin hello'); process.exit(2); }
console.log(`ws_log: hello + ${steps.length} steps (seq ${steps[0]?.seq}..${steps.at(-1)?.seq})`);

// targets: indices into the steps array (not seq). spot-check = subset; demo = all.
const targets = ALL
  ? steps.map((_, i) => i)
  : STEPS.split(',').filter((s) => s !== '').map((s) => parseInt(s, 10));
if (!targets.length) { console.error('nada que renderizar (--all o --steps)'); process.exit(2); }
const targetSet = new Set(targets);
const lastTarget = Math.max(...targets);

const browser = await puppeteer.launch({
  headless: true,
  args: ['--no-sandbox', '--enable-unsafe-swiftshader',
         '--use-gl=angle', '--use-angle=swiftshader', '--disable-gpu',
         '--force-device-scale-factor=1'],
});
const page = await browser.newPage();
await page.setViewport({ width: W, height: H, deviceScaleFactor: 1 });
await page.goto(URL, { waitUntil: 'networkidle0' });
await page.waitForFunction('window.__ready === true', { timeout: 20000 });

// net_md5: the served scene must derive from the same net as the hello (like the client)
const netMd5 = await page.evaluate(() => window.__netMd5);
if (netMd5 !== hello.net_md5) {
  console.error(`net_md5 escena ${netMd5?.slice(0, 8)} != hello ${hello.net_md5?.slice(0, 8)} — escena equivocada`);
  process.exit(3);
}
await page.evaluate((h) => window.__setHello(h), hello);

// apply ALL steps in order; capture at the targets
let shot = 0;
for (let i = 0; i <= lastTarget; i++) {
  await page.evaluate((s) => window.__applyStep(s), steps[i]);
  if (targetSet.has(i)) {
    await page.evaluate(() => window.__render());
    const f = path.join(OUT, `step_${String(steps[i].seq).padStart(5, '0')}.png`);
    await page.screenshot({ path: f });
    const n = await page.evaluate(() => window.__size());
    console.log(`  step idx=${i} seq=${steps[i].seq} t_sim=${steps[i].t_sim.toFixed(3)} veh=${n} -> ${path.basename(f)}`);
    shot++;
  }
}
console.log(`OK: ${shot} frames en ${OUT}`);
await browser.close();
