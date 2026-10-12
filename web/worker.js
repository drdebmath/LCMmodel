/* Simulation worker: owns the Rust core (web/pkg, built by scripts/build-wasm.sh)
 * so the page never blocks on it.
 *
 * While playing, the worker simulates in ~6 ms slices and yields between
 * slices so messages get through. The page asks for a frame whenever it is
 * ready to draw; simulation speed never waits on drawing.
 *
 * Playback speed is simulated time per wall-clock second. The worker stops
 * each slice at the simulated time the clock allows (the core's `until_time`)
 * and idles until more is due; `Infinity` runs flat out. If the machine can't
 * keep up, the clock is re-anchored instead of letting a backlog build.
 *
 * Frame buffers are recycled: the page sends each frame's arrays back with
 * its next request and the worker refills them, so steady playback allocates
 * nothing per frame.
 *
 * Page -> worker
 *   {type: "hello"}                  reply {type: "ready", algorithms}
 *   {type: "start", config, run}     new simulation (paused); reply with its first frame.
 *                                    `run` is echoed on every frame so the page can
 *                                    drop frames that belong to an earlier run
 *   {type: "play"} / {type: "pause"}
 *   {type: "speed", value}           simulated time units per second, or Infinity
 *   {type: "frame", buffers?}        reply with the current frame (refilling `buffers`)
 *   {type: "step", unit, amount}     paused stepping: unit "event" (amount events)
 *                                    or "time" (amount time units); reply with a frame
 *   {type: "inspect", id, seq}       reply {type: "robot", id, seq, info}
 *   {type: "csv"}                    reply {type: "csv", text}
 *   {type: "stop"}                   drop the simulation
 * Worker -> page
 *   {type: "frame", ...stats, lastEvent?, xy, flags, light, fault, task, target, circle}
 *       arrays are transferred; `stop` is "running" while the simulation can
 *       continue, otherwise why it stopped
 *   {type: "error", message}
 */
// The page passes the package's build id (web/pkg/build.json) as ?v=…, and the
// package is loaded with the same id, so cached files of another build are
// never used.
const version = new URL(self.location.href).searchParams.get("v");
const suffix = version ? `?v=${encodeURIComponent(version)}` : "";
let wasm = null;

const STOP = ["budget", "until-time", "ended", "max-events", "max-time", "stalled", "max-turns"];
const SLICE_MS = 6;
const IDLE_MS = 4;
const ready = (async () => {
  const module = await import(`./pkg/lcm_wasm.js${suffix}`);
  await module.default(`./pkg/lcm_wasm_bg.wasm${suffix}`);
  wasm = module;
})();
let sim = null;
let playing = false;
let stop = "running";
let speed = Infinity;           // simulated time units per wall second
let anchor = { wall: 0, sim: 0 };
let batch = 64;      // events per advance() call, tuned so one call takes ~1 ms
let busyMs = 0;      // compute time since the last frame was sent
let sinceMs = 0;     // wall time the last frame was sent
let spare = null;    // buffers the page returned, refilled for the next frame
let run = 0;         // id of the current simulation, from the page

// Each play starts a new generation; a slice from an older one (still queued
// after a quick pause/play) exits instead of running a second loop.
let generation = 0;
// Zero-delay yield: setTimeout(0) is clamped to >= 4 ms once nested.
const channel = new MessageChannel();
channel.port1.onmessage = ({ data }) => slice(data);
const yieldThenSlice = () => channel.port2.postMessage(generation);

// Swarm size over the robots' moves (Looks and arrivals, not the clock ticks), sampled here as the
// simulation advances (not once per drawn frame), so a run that ends between two frames still has
// a curve. Time would squeeze the convergence against the right edge: the first long walks take
// most of the time. At most ~300 points: when full, every other point is dropped.
const TRACE_MAX = 300;
let trace = [];       // robot events, size, robot events, size, ...
let traceGap = 0;
let traceNext = 0;
let traceCostMs = 0;  // wall time spent sampling, kept under ~10% of the run so a big swarm pays little
let traceStartMs = 0;
function tracePoint(force = false) {
  const t = sim.event_count() - sim.tick_count();
  if (!force && t < traceNext) return;
  const began = performance.now();
  if (!force && traceCostMs > 0.1 * (began - traceStartMs)) return;
  const size = sim.swarm_size();
  traceCostMs += performance.now() - began;
  const n = trace.length;
  if (n && trace[n - 2] === t) trace[n - 1] = size; else trace.push(t, size);
  traceNext = t + traceGap;
  if (trace.length > 2 * TRACE_MAX) {
    const kept = [];
    for (let k = 0; k < trace.length - 2; k += 4) kept.push(trace[k], trace[k + 1]);
    kept.push(trace[trace.length - 2], trace[trace.length - 1]);
    trace = kept;
    traceGap = Math.max(traceGap * 2, t / TRACE_MAX);
  }
}

function reanchor() { anchor = { wall: performance.now(), sim: sim ? sim.time() : 0 }; }

function slice(gen) {
  if (!sim || !playing || gen !== generation) return;
  const start = performance.now();
  const due = speed === Infinity ? Infinity : anchor.sim + ((start - anchor.wall) / 1000) * speed;
  let caughtUp = false;
  while (performance.now() - start < SLICE_MS) {
    const t0 = performance.now();
    // A call never covers more than ~1% of what has run so far, so the swarm-size curve gets
    // enough points however fast the run goes (the cost is a few hundred extra calls per run).
    const code = sim.advance(Math.min(batch, Math.max(64, Math.floor(sim.event_count() / 100))), due);
    tracePoint();
    const took = performance.now() - t0;
    if (code === 1) { caughtUp = true; break; }          // reached the time the clock allows
    if (code !== 0) { stop = STOP[code]; playing = false; break; }
    if (took < 0.5) batch = Math.min(batch * 2, 1 << 20);
    else if (took > 2) batch = Math.max(batch >> 1, 1);
  }
  busyMs += performance.now() - start;
  // Falling more than a quarter second behind: drop the backlog rather than burst to catch up.
  if (!caughtUp && speed !== Infinity && due - sim.time() > speed * 0.25) reanchor();
  if (!playing) { tracePoint(true); postFrame(); }  // tell the page it ended without waiting to be asked
  else if (caughtUp) setTimeout(slice, IDLE_MS, gen);
  else yieldThenSlice();
}

function freshBuffers(n) {
  return {
    xy: new Float32Array(2 * n), flags: new Uint8Array(n), light: new Uint8Array(n),
    fault: new Uint8Array(n), task: new Uint8Array(n), target: new Float32Array(2 * n),
    circle: new Float32Array(3 * n),
  };
}

function postFrame(extra = {}) {
  const n = sim.robot_count();
  const b = spare && spare.flags.length === n ? spare : freshBuffers(n);
  spare = null;
  sim.fill_frame(b.xy, b.flags, b.light, b.fault, b.task, b.target, b.circle);
  const now = performance.now();
  self.postMessage({
    type: "frame",
    run,
    time: sim.time(),
    originX: sim.frame_origin_x(),
    originY: sim.frame_origin_y(),
    events: sim.event_count(),
    ticks: sim.tick_count?.() ?? 0,
    robots: n,
    terminated: sim.terminated_count(),
    epochs: sim.sequential?.() ? sim.epochs_completed() : null,
    trace: Float64Array.from(trace),
    ended: sim.ended(),
    stop,
    playing,
    busy: sinceMs ? Math.min(1, busyMs / (now - sinceMs)) : 0,
    ...extra,
    ...b,
  }, Object.values(b).map((a) => a.buffer));
  busyMs = 0;
  sinceMs = now;
}

const KIND = { "-1": null, 0: "Crash", 1: "Look", 2: "Wait", 3: "Visualize" };
const OUTCOME = { 0: "ignored", 2: "moved", 3: "frozen / waiting", 4: "terminated", 5: "crashed", 99: "visualize tick", [-1]: "ended" };

function step(unit, amount) {
  playing = false;
  let lastEvent = null;
  if (unit === "event") {
    for (let i = 0; i < amount && !sim.ended(); i++) {
      const [time, robot, kind, outcome] = sim.step_one();
      lastEvent = { time, robot, kind: KIND[kind], outcome: OUTCOME[outcome] ?? String(outcome) };
    }
  } else {
    const code = sim.advance(0xffffffff, sim.time() + amount);
    if (code >= 2) stop = STOP[code];
  }
  if (sim.ended()) stop = "ended";
  tracePoint(true);
  postFrame({ lastEvent });
}

self.onmessage = async ({ data }) => {
  try {
    await ready;
    switch (data.type) {
      case "hello":
        self.postMessage({ type: "ready", version, algorithms: JSON.parse(wasm.algorithms()) });
        break;
      case "start":
        sim?.free();
        sim = new wasm.WasmSimulation(JSON.stringify(data.config));
        run = data.run ?? run + 1;
        playing = false;
        stop = "running";
        batch = 64;
        trace = []; traceGap = 0; traceNext = 0; traceCostMs = 0; traceStartMs = performance.now();
        tracePoint(true);
        busyMs = 0;
        sinceMs = 0;
        postFrame();
        break;
      case "play":
        if (sim && stop === "running" && !playing) { playing = true; generation++; reanchor(); yieldThenSlice(); }
        break;
      case "speed":
        speed = data.value;
        reanchor();
        break;
      case "pause":
        playing = false;
        break;
      case "frame":
        if (data.buffers) spare = data.buffers;
        if (sim) postFrame();
        break;
      case "step":
        if (sim && stop === "running") step(data.unit, data.amount ?? 1);
        break;
      case "inspect":
        if (sim) self.postMessage({ type: "robot", id: data.id, seq: data.seq, info: JSON.parse(sim.robot_info(data.id)) });
        break;
      case "csv":
        if (sim) self.postMessage({ type: "csv", text: sim.export_csv(), time: sim.time() });
        break;
      case "stop":
        playing = false;
        sim?.free();
        sim = null;
        break;
    }
  } catch (error) {
    playing = false;
    self.postMessage({ type: "error", message: String(error?.message ?? error) });
  }
};
