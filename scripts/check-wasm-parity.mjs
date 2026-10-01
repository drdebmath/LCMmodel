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
console.log(failures ? `\n${failures} fixture(s) differ` : "\nwasm == native (bit for bit) and both match python");
process.exit(failures ? 1 : 0);
