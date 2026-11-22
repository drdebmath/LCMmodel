// Every algorithm under every scheduler, rigid and non-rigid, in both builds:
// each run must end on its own, stop at its limit, or be refused by the
// algorithm with a reason. It must never crash, and the browser build must
// agree with the native one on every count, bit for bit.
//
//   cargo build --release -p lcm-cli && ./scripts/build-wasm.sh
//   node scripts/scheduler-matrix.mjs
import { spawnSync } from "node:child_process";
import { mkdtempSync, readFileSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { fileURLToPath } from "node:url";
import { algorithms, initSync, WasmSimulation } from "../web/pkg/lcm_wasm.js";

const root = fileURLToPath(new URL("..", import.meta.url));
initSync({ module: readFileSync(join(root, "web/pkg/lcm_wasm_bg.wasm")) });
const native = join(process.env.CARGO_TARGET_DIR ?? join(root, "target"), "release", process.platform === "win32" ? "lcm.exe" : "lcm");
const scratch = mkdtempSync(join(tmpdir(), "lcm-matrix-"));

const STOP = { 2: "Ended", 3: "MaxEvents", 4: "MaxTime", 5: "Stalled", 6: "MaxTurns", 8: "MaxRounds" };
const SCHEDULERS = ["async", "sequential", "ssync", "fsync", "kasync"];
const MOVEMENT = { rigid: { rigid_movement: true }, "non-rigid": { rigid_movement: false, delta: 4, stop_policy: "random" } };
const TWO_ROBOTS = /^(CompassGathering|RandomizedGathering)/;

let failures = 0, runs = 0, refused = 0;
const tally = {};
for (const { key } of JSON.parse(algorithms())) {
  for (const scheduler of SCHEDULERS) {
    for (const [moveName, movement] of Object.entries(MOVEMENT)) {
      const config = {
        algorithm: key, scheduler, num_of_robots: TWO_ROBOTS.test(key) ? 2 : 12, random_seed: 7,
        max_events: 300_000, ...movement, ...(scheduler === "kasync" ? { k_async: 2 } : {}),
      };
      const label = `${key.padEnd(27)} ${scheduler.padEnd(10)} ${moveName.padEnd(9)}`;
      // The browser build.
      let wasm;
      try {
        const sim = new WasmSimulation(JSON.stringify(config));
        let code;
        while ((code = sim.advance(1 << 20, Infinity)) === 0 || code === 7);
        wasm = { events: sim.event_count(), time: sim.time(), terminated: sim.terminated_count(), stopped: STOP[code] ?? `code ${code}` };
        sim.free();
      } catch (e) {
        wasm = { refused: String(e?.message ?? e) };
      }
      // The native build, same configuration.
      const path = join(scratch, `${key}-${scheduler}-${moveName}.json`);
      writeFileSync(path, JSON.stringify(config));
      const out = spawnSync(native, ["run", path], { encoding: "utf8" });
      let nat;
      if (out.status === 0) {
        const r = JSON.parse(out.stdout.trim().split("\n").pop());
        nat = { events: r.events, time: r.time, terminated: r.terminated, stopped: r.stopped };
      } else {
        nat = { refused: (out.stderr || out.stdout).trim() };
      }
      runs++;
      let problem = null;
      if (out.signal || /panicked|unreachable|RuntimeError/.test(`${out.stderr} ${wasm.refused ?? ""}`)) problem = "crashed";
      else if (wasm.refused || nat.refused) {
        if (!(wasm.refused && nat.refused)) problem = `refused in one build only: wasm=${wasm.refused ?? "ran"} native=${nat.refused ?? "ran"}`;
        else refused++;
      } else if (["events", "time", "terminated", "stopped"].some((k) => wasm[k] !== nat[k])) {
        problem = `builds differ: wasm=${JSON.stringify(wasm)} native=${JSON.stringify(nat)}`;
      }
      const outcome = wasm.refused ? "refused" : wasm.stopped;
      tally[outcome] = (tally[outcome] ?? 0) + 1;
      if (problem) failures++;
      const detail = wasm.refused ? `refused: ${wasm.refused.slice(0, 70)}` : `${wasm.stopped}, ${wasm.terminated}/${config.num_of_robots} terminated, ${wasm.events} events`;
      console.log(`${problem ? "FAIL" : "ok  "} ${label} ${detail}${problem ? `\n     ${problem}` : ""}`);
    }
  }
}
console.log(`\n${runs} runs: ${Object.entries(tally).map(([k, v]) => `${v} ${k}`).join(", ")}`);
console.log(failures ? `${failures} problem(s)` : "no crashes; every run ended, stopped at its limit or was refused with a reason; wasm == native");
process.exit(failures ? 1 : 0);
