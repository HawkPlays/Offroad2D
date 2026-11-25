// engine/icons.js — one shared pictogram set (no text needed to play). 24×24 grid, line style: stroke = currentColor, width 2,
// round caps / joins; parts marked `f` are filled. `svg(name)` -> inline SVG markup for the DOM HUD; `paintIcon(ctx, name, x, y,
// size, style)` draws the same paths on a canvas (start screen: Samsung "dark sites" mode recolours CSS / SVG colours, never canvas).
// Each icon = list of [d, mode?] (mode 'f' = fill only, 'fs' = fill + stroke, 't' = two-tone: translucent fill (.3) + stroke,
// 'w' = heavy stroke (2.8), default stroke only, 'd' = dim stroke: opacity .4). Default stroke 2.3: solid objects, not hairlines.

const I = {
  menu: [['M4 7h16M4 12h16M4 17h16']],
  reset: [['M19.5 12A7.5 7.5 0 1 1 17 6.4'], ['M19.5 3.5v4.4h-4.4']],
  play: [['M8 5.5l11 6.5-11 6.5z', 'fs']],
  pause: [['M8 5.5v13M16 5.5v13']],
  close: [['M6.5 6.5l11 11M17.5 6.5l-11 11']],
  next: [['M5 6l7 6-7 6zM13 6l7 6-7 6z', 'fs']],
  back: [['M15 5l-7 7 7 7']],
  map: [['M3.5 6.5L9 4.5l6 2 5.5-2v13l-5.5 2-6-2-5.5 2z', 't'], ['M9 4.5v13M15 6.5v13']],
  wheel: [['M12 3.5a8.5 8.5 0 1 1 0 17a8.5 8.5 0 1 1 0-17z', 't'], ['M12 10a2 2 0 1 1 0 4a2 2 0 1 1 0-4z'], ['M3.8 11.2l6.3 0.4M20.2 11.2l-6.3 0.4M12 14v6.3']],
  btns: [['M10 7l-6 5 6 5z', 'fs'], ['M14 7l6 5-6 5z', 'fs']],
  tilt: [['M5.2 8.6l12.4-3.3 2.6 9.8-12.4 3.3z', 't'], ['M3 6.5A10 10 0 0 1 8 3M21 17.5a10 10 0 0 1-5 3.5']],
  center: [['M12 4a8 8 0 1 1 0 16a8 8 0 1 1 0-16z'], ['M12 10.5a1.5 1.5 0 1 1 0 3a1.5 1.5 0 1 1 0-3z', 'f'], ['M12 1.5v4M12 18.5v4M1.5 12h4M18.5 12h4']],
  resize: [['M4 9.5V4h5.5M20 14.5V20h-5.5M4 4l6.5 6.5M20 20l-6.5-6.5']],
  gfx: [['M3.5 5.5h17v13h-17z', 't'], ['M3.5 16l5-5 4 4 2.5-2.5 5.5 5.5'], ['M16 8.2a1.5 1.5 0 1 1 0 3a1.5 1.5 0 1 1 0-3z', 'f']],
  cam3: [['M4 8.5l8-4 8 4v8l-8 4-8-4z', 't'], ['M4 8.5l8 4 8-4M12 12.5v8']],
  cam2: [['M5 5h14v14H5z', 't']],
  bug: [['M8.5 9h7v6.5a3.5 3.5 0 0 1-7 0z', 't'], ['M9.5 9a2.5 2.5 0 0 1 5 0M4 11.5h4.5M15.5 11.5H20M4.5 17l4-1M19.5 17l-4-1M12 11v7']],
  speaker: [['M4 9.5h3.5L12 5.5v13l-4.5-4H4z', 'fs'], ['M15.5 9a4 4 0 0 1 0 6M18.2 6.3a8 8 0 0 1 0 11.4']],
  mute: [['M4 9.5h3.5L12 5.5v13l-4.5-4H4z', 'fs'], ['M15.5 9.5l5 5M20.5 9.5l-5 5']],
  eye: [['M2.5 12S6 5.5 12 5.5 21.5 12 21.5 12 18 18.5 12 18.5 2.5 12 2.5 12z'], ['M12 9a3 3 0 1 1 0 6a3 3 0 1 1 0-6z', 'f']],
  eyeOff: [['M2.5 12S6 5.5 12 5.5 21.5 12 21.5 12 18 18.5 12 18.5 2.5 12 2.5 12z', 'd'], ['M4 4l16 16']],
  auto: [['M17.5 8.5A6.5 6.5 0 0 0 5.7 9.6M6.5 15.5a6.5 6.5 0 0 0 11.8-1.1'], ['M17.8 4.6v4.2h-4.2M6.2 19.4v-4.2h4.2']],
  engine: [['M3.5 10.5h2.5v-2h3v-2h6v2h2.5l2 2.5h1v5h-1l-2 2.5H9l-3-2.5H3.5z', 't'], ['M1.5 11.5v4']],
  world: [['M2.5 18.5l6-9 4 5.5 2.5-3 6.5 6.5z', 't'], ['M15 5.5a1.8 1.8 0 1 1 0 .1']],
  bell: [['M6 16.5v-5a6 6 0 0 1 12 0v5l1.5 1.5h-15z', 't'], ['M10 20.5a2 2 0 0 0 4 0']],
  vibe: [['M8.5 4h7v16h-7z', 't'], ['M5 8.5v7M19 8.5v7M2.5 10.5v3M21.5 10.5v3']],
  vibeOff: [['M8.5 4h7v16h-7z', 'd'], ['M4 4l16 16']],
  vibe1: [['M8.5 4h7v16h-7z', 't'], ['M5 9.5v5M19 9.5v5']],
  // winch hook (centred on the grid): eye, shank, deep curve, barb + latch
  hook: [['M15 2.6a2 2 0 1 1 0 4a2 2 0 1 1 0-4z', 't'], ['M15 6.6V14.5a4.6 4.6 0 0 1-9.2 0v-2.2l2.6 2.1', 'w'], ['M15 9.5l-4.2 3', 'd']],
  // dash warning lamps (gauge): battery, oil can, check engine
  battery: [['M3.5 7.5h17v12h-17z', 't'], ['M6.5 7.5v-2h3v2M14.5 7.5v-2h3v2'], ['M6.5 13.5h4M8.5 11.5v4M13.5 13.5h4']],
  oil: [['M3 11.5h3l2-2h7l5-2.5 1.5 1-6 6.5H7.5l-1.5-1.5H3z', 't'], ['M10 9.5v-2.5h3', 'd'], ['M21 15.5c0 1.2-.6 2-1.3 2s-1.2-.8-1.2-2c0-1 1.2-2.5 1.2-2.5s1.3 1.5 1.3 2.5z', 'f']],
  padlock: [['M6.5 11h11v9.5h-11z', 'fs'], ['M8.8 11V8a3.2 3.2 0 0 1 6.4 0v3']],
  // drivetrain pictograms: top-down truck (front = right), lit axle + padlock on it
  lockF: [['M7 7h10v10H7z', 'd'], ['M2.5 3.5h4v4h-4zM2.5 16.5h4v4h-4z', 'd'], ['M17.5 3.5h4v4h-4zM17.5 16.5h4v4h-4z', 'fs'], ['M19.5 7.5v9'], ['M9.5 11.8h5v4h-5z', 'f'], ['M10.5 11.8v-1.3a1.5 1.5 0 0 1 3 0v1.3']],
  lockR: [['M7 7h10v10H7z', 'd'], ['M17.5 3.5h4v4h-4zM17.5 16.5h4v4h-4z', 'd'], ['M2.5 3.5h4v4h-4zM2.5 16.5h4v4h-4z', 'fs'], ['M4.5 7.5v9'], ['M9.5 11.8h5v4h-5z', 'f'], ['M10.5 11.8v-1.3a1.5 1.5 0 0 1 3 0v1.3']],
  pedal: [['M8.5 3.5h7l1 13h-9z', 't'], ['M10 7h4M9.8 10h4.4M9.6 13h4.8'], ['M12 16.5v4']],
  brakePedal: [['M5.5 7.5h13l-1 8h-11z', 't'], ['M8 10.5h8M8.5 13h7'], ['M12 15.5v5']],
  // toasts / status
  impact: [['M12 2.5l1.9 5.6 5.5-2.6-2.6 5.4 5.7 1.6-5.7 1.9 2.8 5.3-5.5-2.6L12 21.5l-2.1-5.4-5.5 2.6 2.8-5.3L1.5 11.5l5.7-1.6-2.6-5.4 5.5 2.6z', 'fs']],
  stall: [['M3.5 10.5h2.5v-2h3v-2h6v2h2.5l2 2.5h1v5h-1l-2 2.5H9l-3-2.5H3.5z', 't'], ['M3 3l18 18']],
  thermo: [['M10 4.5a2 2 0 0 1 4 0v9.3a4 4 0 1 1-4 0z', 't'], ['M12 15.3a1.8 1.8 0 1 1 0 3.6a1.8 1.8 0 1 1 0-3.6z', 'f'], ['M12 15.5V8.5']],
  cool: [['M12 3v18M4.2 7.5l15.6 9M4.2 16.5l15.6-9'], ['M9.5 4.5L12 6.5l2.5-2M9.5 19.5L12 17.5l2.5 2']],
  rope: [['M2 12h7.5M14.5 12H22'], ['M9.5 12l1.6-2.2M9.5 12l1.6 2.2M14.5 12l-1.6-2.2M14.5 12l-1.6 2.2'], ['M12 4.5v2.5M12 17v2.5', 'd']],
  bridge: [['M2 9.5h7.5M14.5 9.5H22'], ['M4 9.5v8M20 9.5v8'], ['M10.5 11.5l3 5.5'], ['M2 13.5c1 .5 2 .5 2 .5M20 14s1 0 2-.5', 'd']],
  axle: [['M3 7.5v9M21 7.5v9'], ['M3 12h6.5l1.2-2.2 2.6 4.4 1.2-2.2H21']],
  roll: [['M3.5 11h17v4h-5l-1.5 3h-6l-1.5-3h-3z', 't'], ['M7.5 6.2a2.2 2.2 0 1 1 0 .1M16.5 6.2a2.2 2.2 0 1 1 0 .1', 'fs'], ['M2 21.5h20', 'd']],
  recover: [['M3.5 15.5h17v2.5h-17z', 't'], ['M7.5 18a1.8 1.8 0 1 1 0 .1M16.5 18a1.8 1.8 0 1 1 0 .1'], ['M12 12.5V3.5M8.5 7l3.5-3.5L15.5 7']],
  wrench: [['M14.5 3.5a5 5 0 0 0-4.6 6.9L3.5 16.8l3.7 3.7 6.4-6.4a5 5 0 0 0 6.9-4.6l-2.9 2.9-3-1-1-3z', 't']],
  plus: [['M12 5v14M5 12h14']],
  wait: [['M7 3.5h10M7 20.5h10'], ['M8 3.5v2.5l4 6-4 6v2.5M16 3.5v2.5l-4 6 4 6v2.5']],
  sun: [['M12 7.5a4.5 4.5 0 1 1 0 9a4.5 4.5 0 1 1 0-9z', 'fs'], ['M12 1.5v2.5M12 20v2.5M1.5 12H4M20 12h2.5M4.6 4.6l1.8 1.8M17.6 17.6l1.8 1.8M4.6 19.4l1.8-1.8M17.6 6.4l1.8-1.8']],
  moon: [['M15.5 3.5a8.5 8.5 0 1 0 5 13.6A7 7 0 0 1 15.5 3.5z', 'fs']],
  cloud: [['M7 18.5a4.5 4.5 0 0 1-.6-9 6 6 0 0 1 11.4 1.6 3.8 3.8 0 0 1-.3 7.4z', 'fs']],
  dusk: [['M6.5 16a5.5 5.5 0 0 1 11 0z', 'fs'], ['M2 16h20M2 19.5h20', 'd'], ['M12 4v3M4.2 8.2l2 2M19.8 8.2l-2 2']],
  phone: [['M4 7.5h16v9H4z', 't'], ['M18 10.5v3']],
  check: [['M5 12.5l4.5 4.5L19 7.5']],
  dots: [['M6 12h.1M12 12h.1M18 12h.1']],
  tap: [['M12 9a3 3 0 1 1 0 6a3 3 0 1 1 0-6z', 'f'], ['M12 4.5a7.5 7.5 0 1 1 0 15a7.5 7.5 0 1 1 0-15z', 'd']],
  // finish / fail / menu
  clock: [['M12 5.5a7.5 7.5 0 1 1 0 15a7.5 7.5 0 1 1 0-15z', 't'], ['M12 13V9.2M10 2.5h4M12 2.5v3M18.2 5.6l1.3 1.3']],
  pump: [['M4.5 20.5v-15a1.5 1.5 0 0 1 1.5-1.5h6a1.5 1.5 0 0 1 1.5 1.5v15z', 't'], ['M3 20.5h12'], ['M6.5 6.5h5v4h-5z', 'f'], ['M13.5 11h2a1.5 1.5 0 0 1 1.5 1.5v4a1.3 1.3 0 0 0 2.6 0V8.5L17 5.5']],
  can: [['M6 6.5h9.5l3 3v11H6z', 't'], ['M8 4h4v2.5'], ['M9 11.5l6 6M15 11.5l-6 6']],
  flag: [['M5.5 21V3.5'], ['M5.5 4h12l-2.5 4 2.5 4h-12', 'fs']],
  chequer: [['M5.5 21V3.5'], ['M5.5 4h13v8.5h-13z'], ['M5.5 4h3.25v2.83H5.5zM12 4h3.25v2.83H12zM8.75 6.83H12v2.84H8.75zM15.25 6.83h3.25v2.84h-3.25zM5.5 9.67h3.25v2.83H5.5zM12 9.67h3.25v2.83H12z', 'f']],
  trophy: [['M7.5 3.5h9v5a4.5 4.5 0 0 1-9 0z', 'fs'], ['M7.5 5.5H4.5a3 3 0 0 0 3 4M16.5 5.5h3a3 3 0 0 1-3 4M12 13v4M8 20.5h8M9.5 17h5v3.5h-5z']],
  cliff: [['M2.5 8.5h8v13'], ['M14 9.5l5.5 3.2-2 3.4-5.5-3.2z', 'fs'], ['M15 18.5l1.5 2.5M19.5 17.5l1 2.5', 'd']],
  power: [['M6.5 8.5h11v9h-11z', 't'], ['M9.5 8.5V5h5v3.5M3.5 11v4M20.5 11v4M6.5 13h-3M17.5 13h3']],
  speed: [['M3.5 16.5a8.5 8.5 0 1 1 17 0'], ['M12 16.5l4.5-5.5'], ['M6 13.5h.1M8 9.5h.1M12 7.8h.1', 'd']],
  grip: [['M12 3.5a8.5 8.5 0 1 1 0 17a8.5 8.5 0 1 1 0-17z', 't'], ['M12 8.5a3.5 3.5 0 1 1 0 7a3.5 3.5 0 1 1 0-7z'], ['M12 3.5v2.5M12 18v2.5M3.5 12H6M18 12h2.5M6 6l1.8 1.8M16.2 16.2L18 18M6 18l1.8-1.8M16.2 7.8L18 6']],
  truck: [['M2.5 15.5v-4l2.5-4h7.5v4h7.5l1.5 2v2z', 't'], ['M6.5 14.5a2.2 2.2 0 1 1 0 4.4a2.2 2.2 0 1 1 0-4.4zM17 14.5a2.2 2.2 0 1 1 0 4.4a2.2 2.2 0 1 1 0-4.4z', 'fs'], ['M6 8.5h5v3H4', 'd']],
  weight: [['M9 7.5a3 3 0 1 1 6 0'], ['M6.5 7.5h11l2.5 13h-16z', 'fs']],
};
export const ICONS = I;

/** Inline SVG for the DOM (class = extra classes). Dim parts carry .ic-dim. */
export function svg(name, cls = '') {
  const parts = I[name]; if (!parts) return '';
  const body = parts.map(([d, m]) => m === 'f' ? `<path d="${d}" fill="currentColor" stroke="none"/>`
    : m === 'fs' ? `<path d="${d}" fill="currentColor"/>`
    : m === 't' ? `<path class="ic-tt" d="${d}" fill="currentColor" fill-opacity="0.3"/>`
    : m === 'w' ? `<path d="${d}" stroke-width="2.8"/>`
    : m === 'd' ? `<path class="ic-dim" d="${d}"/>` : `<path d="${d}"/>`).join('');
  return `<svg class="ic ${cls}" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.3" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">${body}</svg>`;
}

/** Canvas paint of an icon at (x, y) top-left, size px. style: fill / stroke colour (string or gradient), lw (default size / 12). */
export function paintIcon(ctx, name, x, y, size, { color = '#fff', lw = 2, dim = 0.4 } = {}) {
  const parts = I[name]; if (!parts || typeof Path2D !== 'function') return;
  ctx.save(); ctx.translate(x, y); ctx.scale(size / 24, size / 24);
  ctx.lineCap = 'round'; ctx.lineJoin = 'round'; ctx.lineWidth = lw; ctx.strokeStyle = ctx.fillStyle = color;
  for (const [d, m] of parts) {
    const p = new Path2D(d);
    ctx.globalAlpha = m === 'd' ? dim : 1;
    if (m === 'f' || m === 'fs') ctx.fill(p);
    if (m === 't') { ctx.globalAlpha = 0.3; ctx.fill(p); ctx.globalAlpha = 1; }
    ctx.lineWidth = m === 'w' ? lw * 1.22 : lw;
    if (m !== 'f') ctx.stroke(p);
  }
  ctx.restore();
}
