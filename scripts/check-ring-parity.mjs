// GatheRRing (crates/lcm-ring) in the ring page's package: runs a set of ring
// starts through the native CLI (`lcm ring --trace`) and web/ring-pkg (in
// Node, one activation at a time, as web/ring.js does), and requires the same
// moves and the same end. Also checks that the simulator's package (web/pkg)
// carries no ring code: only web/ring.html loads it.
//
//   cargo build --release -p lcm-cli && ./scripts/build-wasm.sh
//   node scripts/check-ring-parity.mjs
import { spawnSync } from "node:child_process";
import { mkdtempSync, readFileSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { fileURLToPath } from "node:url";
import { initSync, WasmRing } from "../web/ring-pkg/lcm_ring_wasm.js";

const root = fileURLToPath(new URL("..", import.meta.url));
initSync({ module: readFileSync(join(root, "web/ring-pkg/lcm_ring_wasm_bg.wasm")) });
const targetDir = process.env.CARGO_TARGET_DIR ?? join(root, "target");
const native = join(targetDir, "release", process.platform === "win32" ? "lcm.exe" : "lcm");
const scratch = mkdtempSync(join(tmpdir(), "lcm-ring-parity-"));

const CONFIGS = [
  { nodes: 9, robots: [0, 2, 5] },
  { nodes: 12, robots: [0, 0, 3, 4, 4, 8, 9] },
  { nodes: 15, robots: [0, 1, 3, 4, 6, 8, 9, 11, 12, 14] },
  { nodes: 6, robots: [0, 1, 3, 4] },
  { nodes: 10, robots: [0, 1, 2, 4, 5, 6, 7, 8] },
  // The paper's Fig. 4: gathers in epoch 4 = n - 2 (a known exception to Theorem V.1).
  { nodes: 6, robots: [0, 5, 3, 2, 0] },
  // n = 5, k = 4 stacked: never gathers (a known exception); stops at the epoch limit.
  { nodes: 5, robots: [0, 3, 1, 0], max_epochs: 12 },
  { nodes: 9, robots: [3, 4, 5] },
  { nodes: 13, robots: [0, 2, 2, 5, 7, 7, 11], order: [3, 1, 6, 0, 5, 2, 4], ties: "random", seed: 11 },
  { nodes: 16, robots: [1, 4, 6, 9, 12, 15], order: [5, 4, 3, 2, 1, 0], ties: "random", seed: 3 },
  { nodes: 20, robots: [0, 1, 2, 3], ties: "random", seed: 99 },
];

const LINE = /^turn\s+(\d+) epoch\s+(\d+) robot\s+(\d+):\s+(\d+) -> (\d+)\s+(T\d) (.*)$/;
let failures = 0;
CONFIGS.forEach((config, k) => {
  const path = join(scratch, `ring-${k}.json`);
  writeFileSync(path, JSON.stringify(config));
  const out = spawnSync(native, ["ring", path, "--trace"], { encoding: "utf8" });
  if (out.status !== 0) throw new Error(`config ${k}: lcm exited ${out.status}\n${out.stderr}`);
  const lines = out.stdout.trim().split("\n");
  const summary = JSON.parse(lines.pop());
  const nativeTurns = lines.map((l) => {
    const m = LINE.exec(l.trim());
    if (!m) throw new Error(`config ${k}: unexpected trace line ${l}`);
    return [m[1], m[2], m[3], m[4], m[5], m[6], m[7].trim()].join(" ");
  });

  const ring = new WasmRing(JSON.stringify(config));
  const turns = [];
  for (let t = ring.step(); t !== "null"; t = ring.step()) {
    const o = JSON.parse(t);
    turns.push([o.turn, o.epoch, o.robot, o.from, o.to, o.task, o.rule].join(" "));
  }
  const state = JSON.parse(ring.state());
  ring.free();

  const end = (s) => JSON.stringify([s.status, s.epoch, s.epochs, s.turns, s.positions, s.unsolvable ?? null]);
  const diff = turns.length !== nativeTurns.length ? `${turns.length} vs ${nativeTurns.length} turns`
    : turns.findIndex((t, i) => t !== nativeTurns[i]) >= 0 ? `turn ${turns.findIndex((t, i) => t !== nativeTurns[i]) + 1} differs`
    : end(state) !== end(summary) ? `end ${end(state)} vs ${end(summary)}`
    : null;
  if (diff) failures++;
  console.log(`${diff ? "FAIL" : "ok  "} n=${config.nodes} robots=[${config.robots}]: ${summary.status} in epoch ${summary.epoch}, ${summary.turns} turns${diff ? ` (${diff})` : ""}`);
});

const fig4End = (() => { const r = new WasmRing(JSON.stringify(CONFIGS[5])); while (r.step() !== "null"); const s = JSON.parse(r.state()); r.free(); return s; })();
if (fig4End.bound !== 3 || fig4End.status !== "gathered" || fig4End.epoch !== 4 || fig4End.epochs !== 3) {
  failures++;
  console.log(`FAIL Fig. 4 should gather in epoch 4 (3 completed), bound 3: ${JSON.stringify(fig4End)}`);
}

const simulator = readFileSync(join(root, "web/pkg/lcm_wasm.js"), "utf8");
if (simulator.includes("WasmRing")) {
  failures++;
  console.log("FAIL web/pkg/lcm_wasm.js contains WasmRing: the simulator package should not carry the ring code");
}

console.log(failures ? `${failures} failure(s)` : `${CONFIGS.length} ring runs match, the simulator package has no ring code`);
process.exit(failures ? 1 : 0);
