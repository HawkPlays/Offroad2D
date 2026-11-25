#!/usr/bin/env python3
"""Dev server: like `python3 -m http.server` but tells the browser to always revalidate,
so phones never run a stale cached engine module next to a fresh index.html.
Usage (from the game folder): python3 tools/serve.py [port]   (default 8080)"""
import functools, http.server, os, sys

class NoCache(http.server.SimpleHTTPRequestHandler):
    extensions_map = {**http.server.SimpleHTTPRequestHandler.extensions_map,
                      '.js': 'text/javascript', '.mjs': 'text/javascript', '.json': 'application/json'}
    def end_headers(self):
        self.send_header('Cache-Control', 'no-cache')
        super().end_headers()

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
handler = functools.partial(NoCache, directory=root)
print(f'Offroad2D on http://localhost:{port}/  (Ctrl+C to stop)')
http.server.ThreadingHTTPServer(('', port), handler).serve_forever()
