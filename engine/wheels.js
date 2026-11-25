// engine/wheels.js — per-rig HUD steering wheels (pure strings, no DOM). Each rig gets its own wheel: rim material, spoke
// shape, top-centre marker and hub finish carry the identity; the centre badge is tinted with the truck's paint.
// Layers (hud.js): BODY (rim + spokes, rotates), CAP (hub, rotates, own compositor layer: the horn press scales it) and a
// fixed GLOSS overlay (hub dome highlight + rim sheen that stay put while the wheel turns = light from one direction).
// Shading is baked as gradients across each part (tube-shaded rim, cylindrical / machined gold + gunmetal bars), so the
// SVG is rasterised once and only ever rotated (transform): no per-frame repaint, no filters.
// Coordinates: viewBox -50..50, rim centreline r 40.5, angles in degrees, 0 = top, clockwise.

const R = Math.PI / 180;
const pt = (r, a) => `${(r * Math.sin(a * R)).toFixed(2)} ${(-r * Math.cos(a * R)).toFixed(2)}`;
/** Arc path along radius r from angle a0 to a1 (clockwise). */
const arc = (r, a0, a1) => `M${pt(r, a0)}A${r} ${r} 0 ${a1 - a0 > 180 ? 1 : 0} 1 ${pt(r, a1)}`;
/** Mirror a path's x coordinates (absolute M L Q C Z commands only: every number pair is x y). */
const mirror = d => { let i = 0; return d.replace(/-?\d*\.?\d+/g, n => ' ' + (i++ % 2 ? n : String(-n))); };

const hexRgb = h => { const n = parseInt(String(h || '#777').replace('#', '').slice(0, 6), 16) || 0x777777; return [n >> 16 & 255, n >> 8 & 255, n & 255]; };
const shade = (h, k) => { const [r, g, b] = hexRgb(h); return `rgb(${Math.round(r * k)},${Math.round(g * k)},${Math.round(b * k)})`; };

// gradient stops: machined gold across a bar (dark edge, warm body, bright specular line, falloff), gunmetal likewise
const GOLD = [[0, '#4a3107'], [0.16, '#a87a22'], [0.36, '#f0d07a'], [0.45, '#fff5cc'], [0.56, '#e0b148'], [0.8, '#8a6316'], [1, '#3a2705']];
const GUN = [[0, '#101216'], [0.22, '#454c56'], [0.4, '#9aa3ae'], [0.48, '#d2d9e0'], [0.6, '#606873'], [1, '#14161a']];
const stops = s => s.map(([o, c]) => `<stop offset="${o}" stop-color="${c}"/>`).join('');
const lin = (id, s, x2, y2, x1 = 0, y1 = 0) => `<linearGradient id="${id}" x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}">${stops(s)}</linearGradient>`;
// tube shading across the rim band (radial in user space: rotation-agnostic). band = [inner, outer] radius
const tube = (id, band, c, rr = 46) => {
  const [a, b] = band, o = r => (r / rr).toFixed(3), m = (a + b) / 2, w = b - a;
  return `<radialGradient id="${id}" gradientUnits="userSpaceOnUse" cx="0" cy="0" r="${rr}">`
    + `<stop offset="${o(a)}" stop-color="${c[0]}"/><stop offset="${o(a + w * 0.22)}" stop-color="${c[1]}"/>`
    + `<stop offset="${o(m - w * 0.06)}" stop-color="${c[2]}"/><stop offset="${o(m + w * 0.12)}" stop-color="${c[3]}"/>`
    + `<stop offset="${o(b - w * 0.12)}" stop-color="${c[4]}"/><stop offset="${o(b)}" stop-color="${c[5]}"/></radialGradient>`;
};
const LEATHER = ['#07080a', '#1d1f23', '#3a3d43', '#2a2c31', '#141518', '#060607'];
const SUEDE = ['#0b0b0d', '#1f2023', '#2e3034', '#292a2e', '#17181b', '#08080a'];
const PAD = ['#08090b', '#25272c', '#4a4d54', '#34363c', '#17181b', '#070708'];
const GOLDT = ['#3a2705', '#94691b', '#ffefb8', '#e3b54c', '#8a6316', '#3a2705'];

const defs = paint => '<defs>'
  + lin('swGh', GOLD, 0, 1) + lin('swGv', GOLD, 1, 0) + lin('swGd', GOLD, 1, 1)
  + lin('swMh', GUN, 0, 1) + lin('swMv', GUN, 1, 0)
  + tube('swRim', [36, 45], LEATHER) + tube('swSue', [35.5, 45.5], SUEDE) + tube('swPad', [35.3, 45.7], PAD)
  + tube('swGt', [35.5, 45.5], GOLDT)
  + '<linearGradient id="swFlat" gradientUnits="userSpaceOnUse" x1="0" y1="29" x2="0" y2="38">'
  + `<stop offset="0" stop-color="${LEATHER[0]}"/><stop offset=".22" stop-color="${LEATHER[1]}"/><stop offset=".45" stop-color="${LEATHER[2]}"/>`
  + `<stop offset=".62" stop-color="${LEATHER[3]}"/><stop offset=".88" stop-color="${LEATHER[4]}"/><stop offset="1" stop-color="${LEATHER[5]}"/></linearGradient>`
  + '<pattern id="swCarb" width="2.6" height="2.6" patternUnits="userSpaceOnUse" patternTransform="rotate(45)">'
  + '<rect width="2.6" height="2.6" fill="#141518"/><rect width="1.3" height="1.3" fill="#2c2e33"/><rect x="1.3" y="1.3" width="1.3" height="1.3" fill="#24262a"/></pattern>'
  + tube('swCsh', [36, 45], ['rgba(0,0,0,.75)', 'rgba(0,0,0,.25)', 'rgba(255,255,255,.12)', 'rgba(255,255,255,0)', 'rgba(0,0,0,.35)', 'rgba(0,0,0,.8)'])
  + `<radialGradient id="swBadge" cx=".42" cy=".38" r=".7"><stop offset="0" stop-color="${shade(paint, 0.62)}"/><stop offset=".7" stop-color="${shade(paint, 0.3)}"/><stop offset="1" stop-color="${shade(paint, 0.14)}"/></radialGradient>`
  + '</defs>';

// leather grain: two faint dotted rings (cheap texture that survives rotation)
const grain = (r0 = 39, r1 = 42.2, c = 'rgba(255,255,255,.09)') => `<circle r="${r0}" fill="none" stroke="${c}" stroke-width=".55" stroke-dasharray=".25 1.5" stroke-linecap="round"/>`
  + `<circle r="${r1}" fill="none" stroke="rgba(0,0,0,.35)" stroke-width=".55" stroke-dasharray=".25 1.7" stroke-linecap="round"/>`;
// stitched seams along both edges of a grip section
const stitch = (a0, a1, c = 'rgba(230,200,140,.55)', r0 = 36.6, r1 = 44.4) => `<path d="${arc(r0, a0, a1)} ${arc(r1, a0, a1)}" fill="none" stroke="${c}" stroke-width=".55" stroke-dasharray="1.3 .9"/>`;
// padded grip where the hands sit: wider band, brighter crown, perforation, seams
const pad = (a0, a1, sc) => `<path d="${arc(40.5, a0, a1)}" fill="none" stroke="url(#swPad)" stroke-width="10.4" stroke-linecap="round"/>`
  + `<path d="${arc(40.5, a0 + 2, a1 - 2)}" fill="none" stroke="rgba(0,0,0,.4)" stroke-width="5" stroke-dasharray=".3 1.25" stroke-linecap="round"/>`
  + stitch(a0 + 1, a1 - 1, sc, 35.7, 45.3);
// a bar with a dark edge line (spokes)
const bar = (d, fill, edge = 'rgba(0,0,0,.55)') => `<path d="${d}" fill="${fill}" stroke="${edge}" stroke-width=".6" stroke-linejoin="round"/>`;
const both = (d, fill, edge) => bar(d, fill, edge) + bar(mirror(d), fill, edge);
// recessed cut-out: dark hollow with a lit lower lip
const cut = d => `<path d="${d}" fill="#1c1404" opacity=".82"/><path d="${d}" fill="none" stroke="rgba(255,236,170,.35)" stroke-width=".45"/>`;

const LOGO = '<path class="sw-logo" d="M-7.2 4.6L-1.2-4.6 2.3 0 4.6-3.4 8.2 4.6Z" fill="url(#swGh)" stroke="#2a1c05" stroke-width=".5" stroke-linejoin="round"/>';
const HORN = '<path class="sw-horn" d="M-7.5-2.6h3.2l5-4.2v13.6l-5-4.2h-3.2z M3.6-3.6a5 5 0 0 1 0 7.2 M6.2-6.2a8.6 8.6 0 0 1 0 12.4"/>';
/** Hub cap: shadow, gold ring (finish per rig), tinted badge, emblem; ring = {w, knurl?, bolts?, rivets?, polish?}. */
const cap = ring => {
  const ringR = 12.4, n = ring.bolts || 0;
  let s = '<circle r="15.6" fill="rgba(0,0,0,.45)"/>'
    + `<circle r="${ringR}" fill="none" stroke="url(#swGd)" stroke-width="${ring.w}"/>`
    + `<circle r="${ringR + ring.w / 2}" fill="none" stroke="#2a1c05" stroke-width=".6"/>`
    + `<circle r="${ringR - ring.w / 2}" fill="none" stroke="rgba(0,0,0,.6)" stroke-width=".7"/>`;
  if (ring.knurl) s += `<circle r="${ringR}" fill="none" stroke="rgba(40,26,4,.55)" stroke-width="${ring.w * 0.7}" stroke-dasharray=".5 .8"/>`;
  if (ring.polish) s += `<circle r="${ringR + ring.w * 0.18}" fill="none" stroke="rgba(255,250,225,.55)" stroke-width=".45"/>`;
  s += `<circle r="${ringR - ring.w / 2}" fill="url(#swBadge)"/>`;
  for (let i = 0; i < n; i++) {
    const a = i * 360 / n + (ring.off || 0), [x, y] = pt(ringR, a).split(' ');
    s += `<circle cx="${x}" cy="${y}" r=".95" fill="#3a2705"/><circle cx="${(+x - 0.25).toFixed(2)}" cy="${(+y - 0.3).toFixed(2)}" r=".5" fill="#fff0c0"/>`;
  }
  if (ring.rivets) s += `<circle r="${ringR - ring.w / 2 - 1.3}" fill="none" stroke="#e3b54c" stroke-width=".9" stroke-dasharray=".01 2.05" stroke-linecap="round"/>`;
  return s + LOGO + HORN;
};
const rimGloss = '<path d="' + arc(42.6, -78, -18) + '" fill="none" stroke="url(#swGl)" stroke-width="2.2" stroke-linecap="round"/>';

/** Rig wheels keyed by vehicle look.style. body / cap = SVG inner markup (viewBox -50 -50 100 100); gloss: rim sheen ok (round rim). */
export const WHEELS = {
  // JEEP · B: gold spokes with recessed cut-outs + gold ribbed grips at 3 / 9; marker = clean dark-slotted gold bar
  jeep: {
    body: `<circle r="40.5" fill="none" stroke="url(#swRim)" stroke-width="9"/>${grain()}`
      + [[62, 118], [242, 298]].map(([a, b]) => `<path d="${arc(40.5, a, b)}" fill="none" stroke="url(#swGt)" stroke-width="9.8" stroke-linecap="butt"/>`
        + `<path d="${arc(40.5, a + 1.5, b - 1.5)}" fill="none" stroke="rgba(40,26,4,.55)" stroke-width="9.8" stroke-dasharray=".8 2"/>`).join('')
      + `<path d="${arc(40.5, -8, 8)}" fill="none" stroke="url(#swGt)" stroke-width="10.4"/><path d="M0-36.4V-44.6" stroke="#1b1205" stroke-width="2.1"/>`
      + both('M-12.5-6L-37-3.2L-37 8.8L-12.5 7Z', 'url(#swGh)') + cut('M-17-1.6L-32.5 0.4L-32.5 4.6L-17 3.6Z') + cut(mirror('M-17-1.6L-32.5 0.4L-32.5 4.6L-17 3.6Z'))
      + bar('M-7 12L7 12L6.2 37L-6.2 37Z', 'url(#swGv)') + cut('M-2.3 16L2.3 16L1.9 32.5L-1.9 32.5Z'),
    cap: cap({ w: 4.6, bolts: 3, off: 0 }), gloss: true,
  },
  // BOGGER · D: carbon rim, split gold spokes, inner gold ring; marker = ribbed gold sleeve with a dark notch
  bogger: {
    body: '<circle r="40.5" fill="none" stroke="url(#swCarb)" stroke-width="9"/><circle r="40.5" fill="none" stroke="url(#swCsh)" stroke-width="9"/>'
      + pad(64, 112, 'rgba(240,190,90,.6)') + pad(248, 296, 'rgba(240,190,90,.6)')
      + '<circle r="35.4" fill="none" stroke="url(#swGd)" stroke-width="1.3"/>'
      + `<path d="${arc(40.5, -15, 15)}" fill="none" stroke="url(#swGt)" stroke-width="11"/><path d="${arc(40.5, -13, 13)}" fill="none" stroke="rgba(40,26,4,.6)" stroke-width="11" stroke-dasharray="1 1.55"/>`
      + '<path d="M-1.4-46L1.4-46L0.9-41.6L-0.9-41.6Z" fill="#120c02"/>'
      + both('M-13-7.5L-36-9.5L-36-4.6L-13-3.2Z', 'url(#swGh)') + both('M-13 3.4L-36 6.2L-35.4 11L-12.4 7.4Z', 'url(#swGh)')
      + both('M-6.4 12L-2.6 12L-3.6 37.2L-8.8 36.6Z', 'url(#swGv)')
      + '<circle cx="-36" cy="1" r="1" fill="#3a2705"/><circle cx="36" cy="1" r="1" fill="#3a2705"/>',
    cap: cap({ w: 4.4, bolts: 6, off: 30 }), gloss: true,
  },
  // BUGGY · C: deep-dish suede rim, gold rivet ring, gunmetal spokes with gold inlays; marker = dual racing ticks
  buggy: {
    body: '<circle r="40.5" fill="none" stroke="url(#swSue)" stroke-width="10"/>' + grain(40, 43, 'rgba(255,255,255,.06)')
      + '<circle r="37.4" fill="none" stroke="#3a2705" stroke-width="1.7" stroke-dasharray=".01 3.25" stroke-linecap="round"/>'
      + '<circle r="37.4" fill="none" stroke="#f2cf72" stroke-width="1" stroke-dasharray=".01 3.25" stroke-linecap="round"/>'
      + stitch(55, 125) + stitch(235, 305)
      + '<path d="M-2.1-35.6V-45.4M2.1-35.6V-45.4" stroke="url(#swGv)" stroke-width="2"/><path d="M0-35.6V-45.4" stroke="#120c02" stroke-width="2.2"/>'
      + both('M-13-5L-36 0L-36 7L-13 6Z', 'url(#swMh)')
      + '<path d="M-16.5 0.8L-33 2.9M16.5 0.8L33 2.9" stroke="url(#swGv)" stroke-width="1.4" stroke-linecap="round"/>'
      + bar('M-6 12L6 12L5 36.6L-5 36.6Z', 'url(#swMv)') + '<path d="M-1.7 15V33.5M1.7 15V33.5" stroke="url(#swGv)" stroke-width="1.2" stroke-linecap="round"/>',
    cap: cap({ w: 3.8, rivets: true }), gloss: true,
  },
  // TRUGGY · D: flat-bottom rim with padded grips, gunmetal spokes with gold inserts, gold horseshoe braces, split gold
  // lower spoke; marker = one wide gold band
  truggy: {
    body: '<path d="M22.76 33.5A40.5 40.5 0 1 0-22.76 33.5" fill="none" stroke="url(#swRim)" stroke-width="9" stroke-linecap="round"/>'
      + '<path d="M-22.76 33.5H22.76" fill="none" stroke="url(#swFlat)" stroke-width="9" stroke-linecap="round"/>' + grain(39, 42.2).replace(/<circle/g, '<circle clip-path="url(#swTop)"')
      + pad(-78, -32, 'rgba(255,255,255,.28)') + pad(32, 78, 'rgba(255,255,255,.28)')
      + `<path d="${arc(40.5, -6.5, 6.5)}" fill="none" stroke="url(#swGt)" stroke-width="10.4"/>`
      + both('M-31 7Q-31 24-14 30.4L-12.2 26.6Q-26.6 21.4-26.8 7Z', 'url(#swGd)')
      + both('M-13-5L-35.5-2L-35.5 6.5L-13 6.5Z', 'url(#swMh)') + both('M-29-1.6L-20-2.6L-20 4.4L-29 4.8Z', 'url(#swGh)')
      + both('M-7.2 12L-2.2 12L-3 29.4L-9.2 29.4Z', 'url(#swGv)') + bar('M-12.5 29L12.5 29L12.5 32.4L-12.5 32.4Z', 'url(#swGh)'),
    cap: cap({ w: 4.8, knurl: true }), gloss: false,
  },
  // FANGO · C: polished gold outer rim over dark leather, gold stitching, gold paddles behind the spokes, gunmetal spokes
  // with gold edges; marker = a subtle notch (thin gold line + a V cut in the outer ring)
  wedge: {
    body: both('M-27.8-21C-33-12-33 6-28 16.5L-24.8 14.8C-28.6 5.4-28.6-11.2-24.8-19.4Z', 'url(#swGv)')
      + '<path d="M-30.4-11.2H-27.4M27.4-11.2H30.4M28.9-12.7V-9.7" stroke="#1b1205" stroke-width=".9" stroke-linecap="round"/>'
      + '<circle r="39.6" fill="none" stroke="url(#swRim)" stroke-width="8.4"/>' + grain(38.4, 41.4)
      + '<circle r="44.5" fill="none" stroke="url(#swGt)" stroke-width="2.4"/><circle r="45.7" fill="none" stroke="rgba(30,20,3,.8)" stroke-width=".5"/>'
      + '<circle r="36.1" fill="none" stroke="rgba(240,200,110,.7)" stroke-width=".55" stroke-dasharray="1.3 .9"/><circle r="42.9" fill="none" stroke="rgba(240,200,110,.7)" stroke-width=".55" stroke-dasharray="1.3 .9"/>'
      + '<path d="M0-35.4V-43.2" stroke="url(#swGv)" stroke-width="1.3"/><path d="M-1.9-46.2L1.9-46.2L0-43.4Z" fill="#120c02"/>'
      + both('M-13-5L-35.5-1L-35.5 6L-13 6Z', 'url(#swMh)', '#c99a35')
      + bar('M-6 12L6 12L4.5 35.6L-4.5 35.6Z', 'url(#swMv)', '#c99a35') + '<path d="M0 15V33" stroke="url(#swGv)" stroke-width="1.1"/>',
    cap: cap({ w: 4.2, bolts: 5, off: 0, polish: true }), gloss: true,
  },
};
const STYLE = { jeep: 'jeep', bogger: 'bogger', buggy: 'buggy', truggy: 'truggy', wedge: 'wedge' };

/** {body, cap, gloss} SVG documents for a rig (look.style; unknown = jeep), badge tinted with paint.body. */
export function wheelSvg(style, paint = '#556', uid = '') {
  const w = WHEELS[STYLE[style] || 'jeep'], d = defs(paint), vb = 'viewBox="-50 -50 100 100" aria-hidden="true"';
  const top = w === WHEELS.truggy ? '<clipPath id="swTop"><rect x="-50" y="-50" width="100" height="78"/></clipPath>' : '';
  const u = t => uid ? t.replace(/(id="|url\(#)sw/g, `$1sw${uid}`) : t;   // several wheels in one document (previews)
  return {
    body: u(`<svg class="sw-body" ${vb}>${d.replace('</defs>', top + '</defs>')}${w.body}</svg>`),
    cap: u(`<svg class="sw-cap" ${vb}>${d}${w.cap}</svg>`),
    gloss: u(`<svg class="sw-gloss" ${vb}><defs><radialGradient id="swDome" cx=".36" cy=".3" r=".6"><stop offset="0" stop-color="#fff" stop-opacity=".42"/>`
      + '<stop offset=".45" stop-color="#fff" stop-opacity=".1"/><stop offset="1" stop-color="#fff" stop-opacity="0"/></radialGradient>'
      + '<linearGradient id="swGl" x1="0" y1="1" x2="1" y2="0"><stop offset="0" stop-color="#fff" stop-opacity="0"/><stop offset=".5" stop-color="#fff" stop-opacity=".28"/>'
      + '<stop offset="1" stop-color="#fff" stop-opacity="0"/></linearGradient></defs>'
      + '<circle r="10.2" fill="url(#swDome)"/><ellipse cx="-3.6" cy="-5.4" rx="2.6" ry="1.3" fill="#fff" opacity=".28" transform="rotate(-30 -3.6 -5.4)"/>'
      + (w.gloss ? rimGloss : '') + '</svg>'),
  };
}
