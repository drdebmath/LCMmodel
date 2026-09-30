#!/usr/bin/env python3
"""Time the same simulation in the Python engine, the native Rust core and
the browser build (WebAssembly, run in Node).

All three use the portable random generator, so they replay the identical
run; the script checks the event counts and end times agree before it
compares speed. Only the event loop is timed, not start-up.

Needs: numpy for the Python engine, node, and the builds:
    cargo build --release -p lcm-cli && ./scripts/build-wasm.sh
    python3 scripts/compare-speed.py [robots ...]      (default: 20 50 100)
"""
import contextlib, io, json, os, subprocess, sys, tempfile, time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "reference"))

import numpy as np
from portable_rng import PortableRng

np.random.default_rng = lambda seed=None: PortableRng(seed)
with contextlib.redirect_stdout(io.StringIO()):
    import robot, scheduler, run


class _Quiet:
    def info(self, *a): pass
    def warning(self, *a): pass
    def error(self, *a): pass


robot.Robot._logger = _Quiet()
scheduler.Scheduler._logger = _Quiet()
run.log_info = run.log_error = lambda *a: None

WASM = """
import { readFileSync } from "node:fs";
import { initSync, WasmSimulation } from "%s/web/pkg/lcm_wasm.js";
initSync({ module: readFileSync("%s/web/pkg/lcm_wasm_bg.wasm") });
const sim = new WasmSimulation(readFileSync(process.argv[2], "utf8"));
const t0 = performance.now();
while (sim.advance(1 << 20, Infinity) === 0);
console.log(JSON.stringify({ events: sim.event_count(), time: sim.time(), wall_seconds: (performance.now() - t0) / 1000 }));
""" % (ROOT, ROOT)


def python_run(config):
    params = dict(config, initial_positions=[])
    run.setup_simulation(json.dumps(params))
    s = run.scheduler_instance
    events = 0
    t0 = time.perf_counter()
    while not s.terminate:
        s.handle_event()
        events += 1
    return {"events": events, "time": float(s.current_time), "wall_seconds": time.perf_counter() - t0}


def main():
    sizes = [int(a) for a in sys.argv[1:]] or [20, 50, 100]
    native = os.path.join(ROOT, "target", "release", "lcm")
    tmp = tempfile.mkdtemp(prefix="lcm-speed-")
    wasm_js = os.path.join(tmp, "wasm.mjs")
    with open(wasm_js, "w") as f:
        f.write(WASM)

    print(f"{'robots':>6} {'events':>9} | {'Python':>9} {'Rust native':>12} {'Rust browser':>13} | {'native ×':>9} {'browser ×':>10} | same run?")
    for n in sizes:
        config = {"algorithm": "Gathering", "num_of_robots": n, "random_seed": 1,
                  "width_bound": 600, "height_bound": 600, "sampling_rate": 0.1}
        path = os.path.join(tmp, f"{n}.json")
        with open(path, "w") as f:
            json.dump(config, f)
        py = python_run(config)
        rs = json.loads(subprocess.check_output([native, "run", path], text=True))
        wa = json.loads(subprocess.check_output(["node", wasm_js, path], text=True))
        same = py["events"] == rs["events"] == wa["events"] and \
            abs(py["time"] - rs["time"]) <= 1e-9 * max(1.0, py["time"]) and rs["time"] == wa["time"]
        print(f"{n:>6} {py['events']:>9} | {py['wall_seconds']:>8.2f}s {rs['wall_seconds']:>11.4f}s {wa['wall_seconds']:>12.4f}s"
              f" | {py['wall_seconds'] / rs['wall_seconds']:>8.0f}× {py['wall_seconds'] / wa['wall_seconds']:>9.0f}× | {'yes' if same else 'NO'}")


if __name__ == "__main__":
    main()
