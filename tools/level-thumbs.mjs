// tools/level-thumbs.mjs — regenerate the start-screen level teasers (levels/thumbs/<id>.jpg, 256x240, scenery only).
// Needs /usr/bin/chromium + the game served on :8765 (python3 -m http.server 8765; PORT=<n> for another). Run: node tools/level-thumbs.mjs
// Thumbs are colour-graded for the menu (saturation / contrast, warm-cool light sweep, vignette).
// Per level: [vehicle (only for physics settle / camera), view] — x/y = camera spot (truck hidden), vh = metres tall, atmo, ax/ay = anchor.
import { spawn } from 'node:child_process';
import fs from 'node:fs';
const OUT = new URL('../levels/thumbs', import.meta.url).pathname;
const PICK = { level01: { vh: 8.5, atmo: 'day' }, level02: { x: 108.5, y: 8.45, vh: 9.5, atmo: 'dusk', ay: 0.5 },
  level03: { x: 143, vh: 10, atmo: 'day', ax: 0.42 }, level05: { x: 61, vh: 8.5, atmo: 'dusk' } };
fs.mkdirSync(OUT, { recursive: true });
const port = 9333 + Math.floor(Math.random() * 500);
const ch = spawn('/usr/bin/chromium', ['--headless=new', '--no-sandbox', '--disable-gpu', `--remote-debugging-port=${port}`, '--user-data-dir=/tmp/o2d-thumbs-' + port, 'about:blank'], { stdio: 'ignore' });
const sleep = ms => new Promise(r => setTimeout(r, ms));
let list; for (let i = 0; i < 50; i++) { try { list = await (await fetch(`http://127.0.0.1:${port}/json/list`)).json(); break; } catch { await sleep(200); } }
const ws = new WebSocket(list.find(t => t.type === 'page').webSocketDebuggerUrl);
await new Promise(r => ws.onopen = r);
let id = 0; const pend = new Map();
ws.onmessage = m => { const d = JSON.parse(m.data); if (d.id && pend.has(d.id)) { pend.get(d.id)(d); pend.delete(d.id); } };
const send = (method, params = {}) => new Promise(r => { const i = ++id; pend.set(i, r); ws.send(JSON.stringify({ id: i, method, params })); });
await send('Page.enable'); await send('Page.navigate', { url: `http://127.0.0.1:${process.env.PORT || 8765}/__thumb404` }); await sleep(1500);
const expr = `(async () => {
  const { PhysicsWorld } = await import('/engine/physics.js'); const { Vehicle } = await import('/engine/vehicle.js'); const { Renderer } = await import('/engine/renderer.js');
  const get = u => fetch(u).then(r => r.json());
  const PICK = ${JSON.stringify(PICK)}, out = {};
  const cv = document.createElement('canvas'); cv.style.cssText = 'position:fixed;left:0;top:0;width:256px;height:240px'; document.body.appendChild(cv);
  for (const [lid, vid, fx, ax] of [['level01','crawler01',1,0.5],['level02','crawler02',1,0.5],['level03','crawler03',1,0.5],['level05','crawler04',1,0.5]]) {
    const L = await get('/levels/' + lid + '.json'), C = await get('/vehicles/' + vid + '.json');
    const p = PICK[lid] || {}, R = new Renderer(cv, { viewHeight: p.vh || 7.5, minViewWidth: 8, shake: 0 });
    R.setAtmosphere(p.atmo || 'day');
    const phys0 = new PhysicsWorld(L); R.setLevel({ ...phys0.getStatic(), logDefs: L.logs || [], bridges: L.bridges || [] });
    const x = p.x ?? (L.stages && L.stages[1] ? L.stages[1].x + 3 : 20), y = p.y ?? R.groundY(x) + 0.05;
    const phys = new PhysicsWorld(L), car = new Vehicle(phys, C, { x, y, angle: p.angle || 0 });
    R.setLevel({ ...phys.getStatic(), logDefs: L.logs || [], bridges: L.bridges || [] });
    R.setVehicle({ ...car.getShape(), paint: C.paint, look: C.look });
    const it3 = R._items3d; R._items3d = function (...a) { return it3.apply(this, a).filter(it => it.ord == null); };   // scenery only: drop truck parts
    car.controls.brake = 1;
    for (let i = 0; i < 150; i++) phys.stepFixed();
    for (let i = 0; i < 10; i++) R.draw({ vehicle: car.getState(), dyn: phys.getDynamic(), cam: '2.5D', dt: 1/60, anchor: [p.ax ?? 0.5, p.ay ?? 0.62], info: {} });
    // pop: graded copy (saturation / contrast lift, warm light sweep from the top left, soft dark vignette)
    const pc = document.createElement('canvas'); pc.width = cv.width; pc.height = cv.height;
    const g = pc.getContext('2d'), W = pc.width, H = pc.height;
    g.filter = 'saturate(1.55) contrast(1.16) brightness(1.06)'; g.drawImage(cv, 0, 0); g.filter = 'none';
    g.globalCompositeOperation = 'soft-light';
    let gr = g.createLinearGradient(0, 0, W, H); gr.addColorStop(0, 'rgba(255,200,120,0.55)'); gr.addColorStop(0.55, 'rgba(255,255,255,0)'); gr.addColorStop(1, 'rgba(40,90,160,0.45)');
    g.fillStyle = gr; g.fillRect(0, 0, W, H);
    g.globalCompositeOperation = 'source-over';
    gr = g.createRadialGradient(W * 0.5, H * 0.45, Math.min(W, H) * 0.35, W * 0.5, H * 0.5, Math.max(W, H) * 0.75);
    gr.addColorStop(0, 'rgba(0,0,0,0)'); gr.addColorStop(1, 'rgba(0,0,0,0.42)'); g.fillStyle = gr; g.fillRect(0, 0, W, H);
    out[lid] = pc.toDataURL('image/jpeg', 0.86);
  }
  return out; })()`;
const r = await send('Runtime.evaluate', { expression: expr, awaitPromise: true, returnByValue: true });
const v = r.result?.result?.value;
if (!v) { console.log(JSON.stringify(r).slice(0, 800)); } else for (const k in v) { fs.writeFileSync(`${OUT}/${k}.jpg`, Buffer.from(v[k].split(',')[1], 'base64')); console.log(k, fs.statSync(`${OUT}/${k}.jpg`).size); }
ch.kill(); process.exit(0);
