"""Serve the repository for local development, with caching turned off.

`python -m http.server` lets the browser reuse cached copies of the page's
JavaScript modules, so after an edit (or a `git pull`) the browser can run a
mix of old and new files. This server tells it to check every file each time.

    python scripts/serve.py            # http://localhost:8000/
    python scripts/serve.py 8080       # another port
"""

import functools
import http.server
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    extensions_map = {
        **http.server.SimpleHTTPRequestHandler.extensions_map,
        ".js": "text/javascript",
        ".mjs": "text/javascript",
        ".wasm": "application/wasm",
    }

    def end_headers(self):
        self.send_header("Cache-Control", "no-store, must-revalidate")
        super().end_headers()


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    handler = functools.partial(NoCacheHandler, directory=str(ROOT))
    with http.server.ThreadingHTTPServer(("", port), handler) as server:
        print(f"Serving {ROOT} at http://localhost:{port}/ (no caching). Ctrl+C to stop.")
        server.serve_forever()


if __name__ == "__main__":
    main()
