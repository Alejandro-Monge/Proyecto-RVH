// spotcheck_live.mjs — captures the LIVE client (index.html) fed by
// ws_replay_server, when its HUD reaches seq==--seq. Used to compare against the
// offline frame of the SAME step (proves that the reconstruction shows what the live
// client would show). Same viewport as the offline replay → pixel-by-pixel comparison.
//
//   node spotcheck_live.mjs --url http://127.0.0.1:PORT/index.html --seq 2124 \
//        --out live_02124.png --width 1280 --height 720
import puppeteer from 'puppeteer';

function arg(name, def) {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && i + 1 < process.argv.length ? process.argv[i + 1] : def;
}
const URL = arg('url'), OUT = arg('out');
const SEQ = parseInt(arg('seq'), 10);
const W = parseInt(arg('width', '1280'), 10), H = parseInt(arg('height', '720'), 10);

const browser = await puppeteer.launch({
  headless: true,
  args: ['--no-sandbox', '--enable-unsafe-swiftshader', '--use-gl=angle',
         '--use-angle=swiftshader', '--disable-gpu', '--force-device-scale-factor=1'],
});
const page = await browser.newPage();
await page.setViewport({ width: W, height: H, deviceScaleFactor: 1 });
await page.goto(URL, { waitUntil: 'networkidle0' });

// wait for the HUD (REAL client, takeLatest) to reach the target seq
const readSeq = () => {
  const t = document.getElementById('hud')?.textContent || '';
  const m = t.match(/seq:\s*(\d+)/);
  return m ? parseInt(m[1], 10) : -1;
};
await page.waitForFunction(
  (target, fn) => {
    const t = document.getElementById('hud')?.textContent || '';
    const m = t.match(/seq:\s*(\d+)/);
    return m && parseInt(m[1], 10) >= target;
  },
  { timeout: 30000, polling: 100 }, SEQ, readSeq.toString());
await new Promise((r) => setTimeout(r, 400));      // settle: a couple of rAF after the last step
const got = await page.evaluate(() => {
  const t = document.getElementById('hud')?.textContent || '';
  const m = t.match(/seq:\s*(\d+)/);
  const seq = m ? parseInt(m[1], 10) : -1;
  document.getElementById('hud').style.display = 'none';   // hide HUD: compare only the 3D
  return seq;
});
await page.screenshot({ path: OUT });
console.log(`live seq=${got} (objetivo ${SEQ}) -> ${OUT}`);
await browser.close();
