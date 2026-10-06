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
 *                                    A sequential algorithm (group "sequential") runs on
 *                                    WasmSeqSimulation with options `config.sequential`;
 *                                    its time and events count rounds
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

const STOP = ["budget", "until-time", "ended", "max-events", "max-time", "stalled"];
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

function reanchor() { anchor = { wall: performance.now(), sim: sim ? sim.time() : 0 }; }

function slice(gen) {
  if (!sim || !playing || gen !== generation) return;
  const start = performance.now();
  const due = speed === Infinity ? Infinity : anchor.sim + ((start - anchor.wall) / 1000) * speed;
  let caughtUp = false;
  while (performance.now() - start < SLICE_MS) {
    const t0 = performance.now();
    const code = sim.advance(batch, due);
    const took = performance.now() - t0;
    if (code === 1) { caughtUp = true; break; }          // reached the time the clock allows
    if (code !== 0) { stop = STOP[code]; playing = false; break; }
    if (took < 0.5) batch = Math.min(batch * 2, 1 << 20);
    else if (took > 2) batch = Math.max(batch >> 1, 1);
  }
  busyMs += performance.now() - start;
  // Falling more than a quarter second behind: drop the backlog rather than burst to catch up.
  if (!caughtUp && speed !== Infinity && due - sim.time() > speed * 0.25) reanchor();
  if (!playing) postFrame();       // tell the page it ended without waiting to be asked
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
    events: sim.event_count(),
    robots: n,
    terminated: sim.terminated_count(),
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
  postFrame({ lastEvent });
}

self.onmessage = async ({ data }) => {
  try {
    await ready;
    switch (data.type) {
      case "hello":
        self.postMessage({ type: "ready", version, algorithms: JSON.parse(wasm.algorithms()) });
        break;
      case "start": {
        sim?.free();
        sim = null;
        // Sequential algorithms (the "Sequential ›" menu) run one robot per
        // round on their own scheduler; `sequential` carries their options.
        const { sequential, ...config } = data.config;
        const group = JSON.parse(wasm.algorithms()).find((a) => a.key === config.algorithm)?.group;
        sim = group === "sequential"
          ? new wasm.WasmSeqSimulation(JSON.stringify(config), JSON.stringify(sequential ?? {}))
          : new wasm.WasmSimulation(JSON.stringify(config));
        run = data.run ?? run + 1;
        playing = false;
        stop = "running";
        batch = 64;
        busyMs = 0;
        sinceMs = 0;
        postFrame();
        break;
      }
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
