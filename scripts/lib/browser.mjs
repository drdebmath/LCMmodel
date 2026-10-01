// Headless Chrome, driven over the DevTools protocol, serving the repository
// over HTTP (or, with LCM_BASE set, testing a published copy of it).
// Shared by scripts/ui-check.mjs and scripts/docs-check.mjs.
// Needs Node >= 22 (built-in WebSocket) and google-chrome or chromium on PATH;
// the GPU is disabled, so drawing is CPU-only.
import { spawn, execSync } from "node:child_process";
import { createServer } from "node:http";
import { mkdtempSync, mkdirSync, readFileSync, rmSync, writeFileSync, existsSync, statSync } from "node:fs";
import { tmpdir } from "node:os";
import { extname, join, normalize } from "node:path";

const ROOT = new URL("../..", import.meta.url).pathname;
const TYPES = { ".html": "text/html", ".js": "text/javascript", ".mjs": "text/javascript", ".css": "text/css",
  ".wasm": "application/wasm", ".json": "application/json", ".png": "image/png", ".svg": "image/svg+xml" };

const server = createServer((req, res) => {
  let path = normalize(join(ROOT, decodeURIComponent(new URL(req.url, "http://x").pathname)));
  // A folder serves its index.html, as GitHub Pages does.
  if (path.startsWith(ROOT) && existsSync(path) && statSync(path).isDirectory()) path = join(path, "index.html");
  if (!path.startsWith(ROOT) || !existsSync(path)) { res.writeHead(404).end(); return; }
  res.writeHead(200, { "content-type": TYPES[extname(path)] ?? "application/octet-stream" }).end(readFileSync(path));
});
export async function openBrowser(OUT) {
mkdirSync(OUT, { recursive: true });
// LCM_BASE=https://…/LCMmodel checks a published copy instead of this checkout.
await new Promise((r) => server.listen(0, "127.0.0.1", r));
const base = (process.env.LCM_BASE ?? `http://127.0.0.1:${server.address().port}`).replace(/\/$/, "");

const chromeBin = ["google-chrome", "chromium", "chromium-browser"].find((b) => {
  try { execSync(`command -v ${b}`, { stdio: "ignore" }); return true; } catch { return false; }
});
const profile = mkdtempSync(join(tmpdir(), "lcm-chrome-"));
const chrome = spawn(chromeBin, ["--headless=new", "--disable-gpu", "--no-first-run", "--hide-scrollbars",
  `--user-data-dir=${profile}`, "--remote-debugging-port=0", "about:blank"], { stdio: ["ignore", "ignore", "pipe"] });
const port = await new Promise((resolve, reject) => {
  let buf = "";
  chrome.stderr.on("data", (d) => { buf += d; const m = buf.match(/DevTools listening on ws:\/\/[^:]+:(\d+)/); if (m) resolve(m[1]); });
  setTimeout(() => reject(new Error("Chrome did not start")), 20000);
});
const targets = await (await fetch(`http://127.0.0.1:${port}/json/list`)).json();
const ws = new WebSocket(targets.find((t) => t.type === "page").webSocketDebuggerUrl);
await new Promise((r) => ws.addEventListener("open", r, { once: true }));

let nextId = 0;
const waiting = new Map();
const consoleErrors = [];
ws.addEventListener("message", ({ data }) => {
  const msg = JSON.parse(data);
  if (msg.id && waiting.has(msg.id)) { waiting.get(msg.id)(msg); waiting.delete(msg.id); }
  if (msg.method === "Runtime.exceptionThrown") consoleErrors.push(msg.params.exceptionDetails.exception?.description ?? msg.params.exceptionDetails.text);
  if (msg.method === "Runtime.consoleAPICalled" && msg.params.type === "error") consoleErrors.push(msg.params.args.map((a) => a.value ?? a.description).join(" "));
});
const send = (method, params = {}) => new Promise((resolve, reject) => {
  const id = ++nextId;
  waiting.set(id, (m) => (m.error ? reject(new Error(`${method}: ${m.error.message}`)) : resolve(m.result)));
  ws.send(JSON.stringify({ id, method, params }));
});
const js = async (expression) => {
  const r = await send("Runtime.evaluate", { expression, awaitPromise: true, returnByValue: true });
  if (r.exceptionDetails) throw new Error(r.exceptionDetails.exception?.description ?? r.exceptionDetails.text);
  return r.result.value;
};
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const shot = async (name) => {
  const { data } = await send("Page.captureScreenshot", { format: "png" });
  writeFileSync(join(OUT, `${name}.png`), Buffer.from(data, "base64"));
};
const key = async (k, code, vk, shift = false) => {
  const mods = shift ? 8 : 0;
  await send("Input.dispatchKeyEvent", { type: "keyDown", key: k, code, windowsVirtualKeyCode: vk, modifiers: mods });
  await send("Input.dispatchKeyEvent", { type: "keyUp", key: k, code, windowsVirtualKeyCode: vk, modifiers: mods });
};
const clickAt = async (x, y) => {
  for (const type of ["mouseMoved", "mousePressed", "mouseReleased"]) {
    await send("Input.dispatchMouseEvent", { type, x, y, button: "left", buttons: type === "mousePressed" ? 1 : 0, clickCount: 1 });
  }
};
const click = (sel) => js(`document.querySelector(${JSON.stringify(sel)}).click(), true`);
const until = async (expr, ms = 5000) => {
  const t0 = Date.now();
  while (Date.now() - t0 < ms) { if (await js(expr)) return true; await sleep(50); }
  return false;
};

const results = [];
const check = (name, ok, detail = "") => { results.push({ name, ok: Boolean(ok), detail }); };
async function finish() {
ws.close();
const exited = new Promise((r) => chrome.once("exit", r));
chrome.kill();
await Promise.race([exited, sleep(3000)]);
server.close();
try { rmSync(profile, { recursive: true, force: true, maxRetries: 5, retryDelay: 200 }); } catch {}
  let failed = 0;
  for (const r of results) { if (!r.ok) failed++; console.log(`${r.ok ? "ok  " : "FAIL"} ${r.name}${r.detail ? `  (${r.detail})` : ""}`); }
  console.log(`\n${results.length - failed} / ${results.length} checks passed · screenshots in ${OUT}`);
  process.exit(failed ? 1 : 0);
}
return { base, send, js, sleep, shot, key, clickAt, click, until, check, finish, consoleErrors };
}
