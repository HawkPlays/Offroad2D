// Node-only ESM loader hooks so `import 'https://esm.sh/...'` works headless (browser needs nothing).
// Fetched modules are cached in tests/.cache/ so later runs work offline.
import { readFile, writeFile, mkdir } from 'node:fs/promises';
import { createHash } from 'node:crypto';
import { fileURLToPath } from 'node:url';

const CACHE = fileURLToPath(new URL('./.cache/', import.meta.url));
const isHttp = s => /^https?:\/\//.test(s);

export async function resolve(specifier, context, next) {
  if (isHttp(specifier)) return { url: specifier, shortCircuit: true };
  if (context.parentURL && isHttp(context.parentURL) && /^[./]/.test(specifier)) {
    return { url: new URL(specifier, context.parentURL).href, shortCircuit: true };
  }
  return next(specifier, context);
}

export async function load(url, context, next) {
  if (!isHttp(url)) return next(url, context);
  const file = CACHE + createHash('sha1').update(url).digest('hex') + '.mjs';
  let source;
  try {
    source = await readFile(file, 'utf8');
  } catch {
    const res = await fetch(url);
    if (!res.ok) throw new Error(`https-loader: ${res.status} for ${url}`);
    source = await res.text();
    await mkdir(CACHE, { recursive: true });
    await writeFile(file, source);
  }
  return { format: 'module', source, shortCircuit: true };
}
