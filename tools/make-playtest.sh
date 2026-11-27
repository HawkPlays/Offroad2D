#!/bin/sh
# Run in a checkout of main's files: strips DEBUG + 2D camera + dev folders for the playtest branch.
set -e
rm -rf tests tools refs CLAUDE.md
python3 - <<'PY'
import re
h='engine/hud.js'; s=open(h).read()
s=s.replace("{ id: 'debug', label: 'DEBUG', key: 'KeyG' }","{ id: 'debug', label: 'DEBUG', key: null }")
s=s.replace("  state.cam = handlers.cam === '2D' ? '2D' : '2.5D';","  state.cam = '2.5D';")
s=s.replace("  const setCam = m => { state.cam = m;","  const setCam = m => { m = '2.5D'; state.cam = m;")
s=s.replace("  setCam(state.cam);\n","  setCam(state.cam); camT.el.style.display = 'none';\n",1)
s=re.sub(r"(  els\.debug = tile\(.*\)\.el;)", r"\1 els.debug.style.display = 'none';", s, 1)
open(h,'w').write(s)
i='index.html'; s=open(i).read()
s=s.replace("cam: (q.get('cam') || store.get('o2d.cam') || '2.5D').toUpperCase()","cam: '2.5D'").replace("debug: s.debug","debug: false")
open(i,'w').write(s)
PY
