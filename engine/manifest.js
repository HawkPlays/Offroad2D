// Static content manifest (no DOM). content.json is primary; this mirrors it so every level + truck still
// loads on static hosts (Netlify / GitHub Pages) when content.json is missing, stale-cached or fails to parse.
// No directory listing is ever needed. tests/physics-tests.js checks it matches content.json.
export const FALLBACK_MANIFEST = {
  levels: [
    { id: 'level01', file: 'levels/level01.json', thumb: 'levels/thumbs/level01.jpg', name: 'Timber Gulch', difficulty: 'Easy', blurb: 'Where legends start' },
    { id: 'level02', file: 'levels/level02.json', thumb: 'levels/thumbs/level02.jpg', name: 'Razorback Gorge', difficulty: 'Hard', blurb: 'Bridges over nothing' },
    { id: 'level03', file: 'levels/level03.json', thumb: 'levels/thumbs/level03.jpg', name: 'Shattered Ridge', difficulty: 'Hard', blurb: 'The cliff says no' },
    { id: 'level05', file: 'levels/level05.json', thumb: 'levels/thumbs/level05.jpg', name: 'Split-Rut Canyon', difficulty: 'Hard', blurb: 'Pick a rut. Pray.' },
  ],
  vehicles: [
    { id: 'crawler01', file: 'vehicles/crawler01.json', name: 'Crawler 01', chips: ['1.4 t', '42:1 low'], trait: 'Short-wheelbase', stats: { power: 2, speed: 2, grip: 5, weight: 2 } },
    { id: 'crawler02', file: 'vehicles/crawler02.json', name: 'Mega Bogger', chips: ['2.3 t', 'mud tyres'], trait: 'Heavy mud plugger', stats: { power: 5, speed: 1, grip: 4, weight: 5 } },
    { id: 'crawler03', file: 'vehicles/crawler03.json', name: 'Rock Buggy', chips: ['0.9 t', 'long travel'], trait: 'Light long-travel', stats: { power: 3, speed: 4, grip: 4, weight: 1 } },
    { id: 'crawler04', file: 'vehicles/crawler04.json', name: 'V8 Truggy', chips: ['1.35 t', 'V8 torque'], trait: 'V8 rock bouncer', stats: { power: 4, speed: 3, grip: 3, weight: 2 } },
    { id: 'crawler05', file: 'vehicles/crawler05.json', name: 'Fango GT', chips: ['1.5 t', 'V10 H-range'], trait: 'Lifted supercar', stats: { power: 4, speed: 5, grip: 2, weight: 3 } },
  ],
  vehicleSlots: 5,
};

/** content.json (may be null / partial) + fallback -> manifest: content.json order + fields win, fallback fills missing entries / fields. */
export function mergeManifest(primary) {
  const p = primary && typeof primary === 'object' ? primary : {}, f = FALLBACK_MANIFEST;
  const list = (a, b) => {
    const out = (Array.isArray(a) ? a : []).filter(x => x && x.id).map(x => ({ ...(b.find(y => y.id === x.id) || {}), ...x }));
    for (const y of b) if (!out.some(x => x.id === y.id)) out.push({ ...y });
    return out;
  };
  return { ...f, ...p, levels: list(p.levels, f.levels), vehicles: list(p.vehicles, f.vehicles) };
}
