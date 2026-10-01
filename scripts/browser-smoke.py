#!/usr/bin/env python3
"""Serve the repository, open a test page in headless Chrome (GPU disabled,
so drawing is CPU-only) and print the JSON the page reports. Exit 1 if the
page reported errors or timed out.

    python3 scripts/browser-smoke.py [robots] [seconds] [page] [playback]

page: web/tests/smoke.html (worker only, the default). The simulator page and
the flowchart page are checked by scripts/ui-check.mjs and scripts/docs-check.mjs.
"""
import http.server, json, os, shutil, subprocess, sys, tempfile, threading

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
robots = sys.argv[1] if len(sys.argv) > 1 else "10000"
seconds = sys.argv[2] if len(sys.argv) > 2 else "5"
page = sys.argv[3] if len(sys.argv) > 3 else "web/tests/smoke.html"
playback = sys.argv[4] if len(sys.argv) > 4 else "20"
result = {}
done = threading.Event()


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw):
        super().__init__(*a, directory=ROOT, **kw)

    def log_message(self, *a):
        pass

    def do_POST(self):
        body = self.rfile.read(int(self.headers["Content-Length"]))
        result.update(json.loads(body))
        self.send_response(204)
        self.end_headers()
        done.set()


server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
threading.Thread(target=server.serve_forever, daemon=True).start()
sep = "&" if "?" in page else "?"
url = f"http://127.0.0.1:{server.server_port}/{page}{sep}robots={robots}&seconds={seconds}&playback={playback}"
chrome = shutil.which("google-chrome") or shutil.which("chromium") or shutil.which("chromium-browser")
profile = tempfile.mkdtemp(prefix="lcm-chrome-")
proc = subprocess.Popen([chrome, "--headless=new", "--disable-gpu", "--window-size=1300,850", "--no-first-run", f"--user-data-dir={profile}",
                         "--remote-debugging-port=0", url], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
ok = done.wait(timeout=float(seconds) + 60)
proc.terminate()
server.shutdown()
shutil.rmtree(profile, ignore_errors=True)
if not ok:
    print("timed out waiting for the page to report")
    sys.exit(1)
shot = result.pop("screenshot", None)
if shot:
    import base64
    path = os.path.join(tempfile.gettempdir(), f"lcm-page-{robots}.png")
    with open(path, "wb") as f:
        f.write(base64.b64decode(shot.split(",", 1)[1]))
    result["screenshot_file"] = path
print(json.dumps(result, indent=2))
sys.exit(1 if result.get("errors") else 0)
