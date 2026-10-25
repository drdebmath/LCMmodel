// Serve the repository for local development, with caching turned off.
//
// A plain static server lets the browser reuse cached copies of the page's
// JavaScript modules, so after an edit (or a `git pull`) the browser can run a
// mix of old and new files. This server tells it to check every file each time.
//
//   node scripts/serve.mjs            # http://localhost:8000/
//   node scripts/serve.mjs 8080       # another port
import { createServer } from "node:http";
import { readFile, stat } from "node:fs/promises";
import { dirname, extname, join, normalize, resolve, sep } from "node:path";
import { fileURLToPath } from "node:url";

const ROOT = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const TYPES = {
  ".html": "text/html; charset=utf-8", ".js": "text/javascript", ".mjs": "text/javascript",
  ".css": "text/css", ".json": "application/json", ".wasm": "application/wasm",
  ".svg": "image/svg+xml", ".png": "image/png", ".md": "text/markdown; charset=utf-8",
};
const port = Number(process.argv[2] ?? 8000);

createServer(async (req, res) => {
  try {
    let path = normalize(join(ROOT, decodeURIComponent(new URL(req.url, "http://x").pathname)));
    if (path !== ROOT && !path.startsWith(ROOT + sep)) throw new Error("outside root");
    if ((await stat(path)).isDirectory()) path = join(path, "index.html");
    const body = await readFile(path);
    res.writeHead(200, {
      "Content-Type": TYPES[extname(path)] ?? "application/octet-stream",
      "Cache-Control": "no-store, must-revalidate",
    });
    res.end(body);
  } catch {
    res.writeHead(404).end("not found");
  }
}).listen(port, () => console.log(`Serving ${ROOT} at http://localhost:${port}/ (no caching). Ctrl+C to stop.`));
