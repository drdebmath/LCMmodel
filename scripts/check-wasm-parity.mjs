// Runs every fixture through the browser build (web/pkg, in Node) and the
// native build (target/release/lcm) and requires identical results: the same
// event count, end state and final time down to the last bit, plus agreement
// with the Python run that produced the fixture.
//
//   cargo build --release -p lcm-cli && ./scripts/build-wasm.sh
//   node scripts/check-wasm-parity.mjs
import { execFileSync } from "node:child_process";
import { mkdtempSync, readFileSync, readdirSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { initSync, WasmSimulation } from "../web/pkg/lcm_wasm.js";

const root = new URL("..", import.meta.url).pathname;
initSync({ module: readFileSync(join(root, "web/pkg/lcm_wasm_bg.wasm")) });
const native = join(root, "target/release/lcm");
const scratch = mkdtempSync(join(tmpdir(), "lcm-parity-"));

let failures = 0;
for (const file of readdirSync(join(root, "fixtures")).filter((f) => f.endsWith(".json")).sort()) {
  const fx = JSON.parse(readFileSync(join(root, "fixtures", file), "utf8"));
  const config = { ...fx.config, max_events: fx.final.event_count };

  const sim = new WasmSimulation(JSON.stringify(config));
  while (sim.advance(1 << 20, Infinity) === 0);
  const wasm = { events: sim.event_count(), time: sim.time(), terminated: sim.terminated_count(), ended: sim.ended() };
  sim.free();

  const path = join(scratch, file);
  writeFileSync(path, JSON.stringify(config));
  const out = JSON.parse(execFileSync(native, ["run", path], { encoding: "utf8" }));

  const same = wasm.events === out.events && wasm.time === out.time && wasm.terminated === out.terminated;
  const python = wasm.events === fx.final.event_count && wasm.ended === fx.final.ended
    && Math.abs(wasm.time - fx.final.time) <= 1e-9 * Math.max(1, Math.abs(fx.final.time));
  if (!same || !python) failures++;
  console.log(`${same && python ? "ok  " : "FAIL"} ${file.padEnd(26)} events=${wasm.events} time wasm=${wasm.time} native=${out.time} python=${fx.final.time}`);
}
// Sequential runs have no Python original, so they are checked against each
// other only: the browser build and the native build must agree on everything.
const STOPPED = { 2: "Ended", 3: "MaxEvents", 4: "MaxTime", 5: "Stalled", 6: "MaxTurns" };
const seq = (over) => ({ scheduler: "sequential", max_events: 2_000_000, ...over });
const SEQUENTIAL = {
  seq_gathering_round_robin: seq({ num_of_robots: 40, random_seed: 3 }),
  seq_sec_random_order_no_gap: seq({ algorithm: "SEC", num_of_robots: 40, activation_order: "random", turn_gap: "none", random_seed: 4 }),
  seq_nonrigid_random_stop: seq({ num_of_robots: 30, rigid_movement: false, delta: 8, stop_policy: "random", robot_speeds: 5, random_seed: 5 }),
  seq_nonrigid_fraction_stop: seq({ algorithm: "SEC", num_of_robots: 30, rigid_movement: false, delta: 3, stop_policy: "fraction", stop_fraction: 0.3, random_seed: 6 }),
  seq_explicit_schedule: seq({ num_of_robots: 3, initial_positions: [[-300, 0], [300, 0], [0, 400]], rigid_movement: false, delta: 1, stop_policy: "delta",
    schedule: [2, 2, { robot: 0, stop: 0.5 }, 1], schedule_end: "repeat", turn_gap: "none", robot_speeds: 1000 }),
  seq_turn_limit: seq({ num_of_robots: 20, max_turns: 15, random_seed: 7 }),
  seq_starts_stacked: seq({ num_of_robots: 24, multiplicities: 2, multiplicity_size: 3, random_seed: 8 }),
  async_nonrigid_delta: { algorithm: "SEC", num_of_robots: 30, rigid_movement: false, delta: 5, max_events: 2_000_000, random_seed: 9 },
};
for (const [name, config] of Object.entries(SEQUENTIAL)) {
  const sim = new WasmSimulation(JSON.stringify(config));
  let code;
  while ((code = sim.advance(1 << 20, Infinity)) === 0);
  const wasm = { events: sim.event_count(), time: sim.time(), terminated: sim.terminated_count(), turns: sim.turn_count(),
    epochs: sim.epochs_completed(), stopped: STOPPED[code] };
  sim.free();
  const path = join(scratch, `${name}.json`);
  writeFileSync(path, JSON.stringify(config));
  const out = JSON.parse(execFileSync(native, ["run", path], { encoding: "utf8" }));
  const same = ["events", "time", "terminated", "turns", "epochs", "stopped"].every((k) => wasm[k] === out[k]);
  if (!same) failures++;
  console.log(`${same ? "ok  " : "FAIL"} ${name.padEnd(30)} events=${wasm.events} turns=${wasm.turns} epochs=${wasm.epochs} ${wasm.stopped}${same ? "" : `  native=${JSON.stringify(out)}`}`);
}

console.log(failures ? `\n${failures} case(s) differ` : "\nwasm == native (bit for bit); the fixtures also match python");
process.exit(failures ? 1 : 0);
