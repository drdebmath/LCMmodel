// SqGathering (Algorithm 8, sequential scheduler): runs every config in
// fixtures/sqgathering through the native CLI (`lcm seq`) and the browser
// build (web/pkg, in Node), both as one batch call and round by round, and
// requires the same trace and report down to the last bit. Budget-limited
// runs must say "incomplete" on both sides.
//
//   cargo build --release -p lcm-cli && ./scripts/build-wasm.sh
//   node scripts/check-seq-parity.mjs
import { spawnSync } from "node:child_process";
import { mkdtempSync, readFileSync, readdirSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { fileURLToPath } from "node:url";
import { initSync, sq_gathering_run, WasmSqGathering } from "../web/pkg/lcm_wasm.js";

const root = fileURLToPath(new URL("..", import.meta.url));
initSync({ module: readFileSync(join(root, "web/pkg/lcm_wasm_bg.wasm")) });
const targetDir = process.env.CARGO_TARGET_DIR ?? join(root, "target");
const native = join(targetDir, "release", process.platform === "win32" ? "lcm.exe" : "lcm");
const scratch = mkdtempSync(join(tmpdir(), "lcm-seq-parity-"));
const dir = join(root, "fixtures", "sqgathering");

/** Deep equality with numbers compared bit for bit (Object.is). */
function same(a, b, path = "$") {
  if (typeof a !== typeof b) return `${path}: ${typeof a} vs ${typeof b}`;
  if (typeof a === "number") return Object.is(a, b) ? null : `${path}: ${a} vs ${b}`;
  if (a === null || typeof a !== "object") return a === b ? null : `${path}: ${a} vs ${b}`;
  const keys = new Set([...Object.keys(a), ...Object.keys(b)]);
  for (const k of keys) {
    const d = same(a[k], b[k], `${path}.${k}`);
    if (d) return d;
  }
  return null;
}

let failures = 0;
for (const file of readdirSync(dir).filter((f) => f.endsWith(".json")).sort()) {
  const path = join(dir, file);
  const text = readFileSync(path, "utf8");

  const tracePath = join(scratch, file + ".jsonl");
  const out = spawnSync(native, ["seq", path, "--format", "json", "--trace", tracePath], { encoding: "utf8" });
  if (out.status !== 0 && out.status !== 3) throw new Error(`${file}: lcm exited ${out.status}\n${out.stderr}`);
  const nativeReport = JSON.parse(out.stdout.trim().split("\n").pop());
  const nativeTrace = readFileSync(tracePath, "utf8").trim().split("\n").filter(Boolean).map((l) => JSON.parse(l));

  const batch = JSON.parse(sq_gathering_run(text, true));

  const stepper = new WasmSqGathering(text);
  const stepped = [];
  for (let r = stepper.step(); r !== "null"; r = stepper.step()) stepped.push(JSON.parse(r));
  const steppedReport = JSON.parse(stepper.report());
  stepper.free();

  const exitMatches = (out.status === 3) === (nativeReport.status === "incomplete");
  const diff = same(nativeReport, batch.report, "report") ?? same(nativeTrace, batch.trace, "trace")
    ?? same(batch.trace, stepped, "stepped") ?? same(batch.report, steppedReport, "stepped.report")
    ?? (exitMatches ? null : `exit ${out.status} but status ${nativeReport.status}`);
  if (diff) failures++;
  const label = nativeReport.status === "incomplete" ? "INCOMPLETE (budget-limited)" : nativeReport.status;
  console.log(`${diff ? "FAIL" : "ok  "} ${file.padEnd(38)} rounds=${String(nativeReport.rounds).padEnd(5)} ${label}${diff ? `\n     ${diff}` : ""}`);
}
console.log(failures ? `\n${failures} fixture(s) differ` : "\nSqGathering: wasm == native (bit for bit), batch == stepped");
process.exit(failures ? 1 : 0);
