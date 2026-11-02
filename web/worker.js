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
 * Turn pacing (sequential scheduler): with a speed of {turn: seconds} every walk takes that long
 * on screen however short it is in simulated time, so the late, tiny moves can be watched one at a
 * time. Turns in which nobody moves are skipped at full speed. A walk has no events inside it, so
 * the worker asks the core to draw the robots at a time part-way along it (set_view_time); the run's
 * own clock only ever moves by handling events, so the result is the same at any pace.
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
 *   {type: "speed", value}           simulated time units per second, or Infinity, or
 *                                    {turn: seconds}: each walk takes that long (sequential only)
 *   {type: "frame", buffers?}        reply with the current frame (refilling `buffers`)
 *   {type: "step", unit, amount}     paused stepping: unit "event" (amount events)
 *                                    or "time" (amount time units); reply with a frame
 *   {type: "inspect", id, seq}       reply {type: "robot", id, seq, info}; info.why: what the robot
 *                                    would see and the rule it would apply if it looked now (≤ 2,000 robots)
 *   {type: "csv"}                    reply {type: "csv", text}
 *   {type: "turn", robot, stop?}     adversary (sequential, schedule_end "await"): add a turn for
 *                                    `robot`, stopped after `stop` of the way, and play it
 *   {type: "round", robots}          adversary (SSYNC, schedule_end "await"): add a round and play it
 *   {type: "preview", robot, seq}    reply {type: "preview", robot, seq, target: [x, y] | null}:
 *                                    where the robot would go if it looked now
 *   {type: "stop"}                   drop the simulation
 * Worker -> page
 *   {type: "frame", ...stats, lastEvent?, xy, flags, light, fault, task, target, circle}
 *       arrays are transferred; `stop` is "running" while the simulation can
 *       continue, otherwise why it stopped; `awaiting` while an adversary's
 *       schedule has run out (the run waits, it has not stopped); `notes` (first
 *       frame of a run): where the settings leave the algorithm's model
 *   {type: "error", message}
 */
// The page passes the package's build id (web/pkg/build.json) as ?v=…, and the
// package is loaded with the same id, so cached files of another build are
// never used.
const version = new URL(self.location.href).searchParams.get("v");
const suffix = version ? `?v=${encodeURIComponent(version)}` : "";
let wasm = null;

const STOP = ["budget", "until-time", "ended", "max-events", "max-time", "stalled", "max-turns", "moved", "max-rounds", "awaiting"];
const AWAITING = 9;
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
let mutualVisibility = false; // the run is MutualVisibility: count who sees everyone once it is over
let seeing = null;            // that count, worked out once per run
let speed = Infinity;           // simulated time units per wall second
let anchor = { wall: 0, sim: 0 };
let perTurn = null;             // seconds each walk takes, in turn-paced playback; null otherwise
let pace = null;                // the turn being played: { from, arrival, progress, settle, robot, ... }.
                                // progress runs from -settle/perTurn to 0 while the robot's decision is shown
                                // (the walk has not started), then from 0 to 1 over the walk
let lastPace = 0;               // wall time of the previous paced slice
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

function reanchor() { anchor = { wall: performance.now(), sim: sim ? sim.time() : 0 }; lastPace = performance.now(); }

function setSpeed(value) {
  if (value && typeof value === "object") {
    perTurn = value.turn > 0 ? value.turn : null;
    if (perTurn !== null) speed = Infinity;
  } else {
    if (perTurn !== null) { pace = null; sim?.set_view_time(NaN); }  // leave a walk where the run's clock is
    perTurn = null;
    speed = value;
  }
  reanchor();
}

const FAST_EVENTS = 4096;  // events handled per call while skipping to the next walk, or to a time within one

/** Turn-paced playback: skip to the next walk at full speed, then play it over `perTurn` seconds. */
function pacedSlice(gen) {
  const start = performance.now();
  // Time spent playing since the last slice: a pause, or a hidden tab, adds nothing, and a walk
  // keeps its progress through a pause.
  let dt = Math.min(0.1, Math.max(0, (start - lastPace) / 1000));
  lastPace = start;
  let waiting = false;
  while (performance.now() - start < SLICE_MS) {
    // Between walks: run flat out (until = Infinity). During one: up to the time its progress has reached.
    let until = Infinity;
    if (pace) {
      pace.progress = Math.min(1, pace.progress + dt / perTurn);
      dt = 0;  // this slice's time counts once
      // The last step is exactly the arrival time, not "from + 1 × (arrival - from)", which can
      // round to a hair less and never handle the arrival. Before the walk starts the robots
      // stand where they were when it looked.
      until = pace.progress >= 1 ? pace.arrival
        : pace.progress <= 0 ? pace.from
        : pace.from + pace.progress * (pace.arrival - pace.from);
    }
    const code = sim.advance_to_move(FAST_EVENTS, until);
    tracePoint();
    if (code === 7) {
      // A robot started a walk: the first of the run, the next turn's (which can begin at the very
      // instant the last one ends, with no gap), or the one that follows a skipped turn.
      const robot = sim.moved_robot();
      const arrival = sim.arrival_time(robot);
      if (arrival > sim.time()) {
        // What the page tells the viewer: who walks, where the algorithm sends it, how far it really
        // goes (the adversary may stop it short), and the time to show the decision before the walk.
        const info = JSON.parse(sim.robot_info(robot));
        const goal = info.goal ?? info.target ?? [info.x, info.y];
        const target = info.target ?? goal;
        const settle = Math.min(0.6, Math.max(0.15, 0.3 * perTurn));
        pace = {
          from: sim.time(), arrival, progress: -settle / perTurn, settle, robot, algorithm: info.algorithm,
          length: Math.hypot(goal[0] - info.x, goal[1] - info.y),
          cut: Math.hypot(goal[0] - target[0], goal[1] - target[1]) > 1e-9,
        };
      } else pace = null;
    } else if (code === 1) {  // reached `until`, and no walk began there
      if (pace.progress >= 1) pace = null;
      else { sim.set_view_time(until); waiting = true; break; }
    } else if (code === AWAITING) { playing = false; break; }
    else if (code !== 0) { stop = STOP[code]; playing = false; break; }
  }
  busyMs += performance.now() - start;
  if (!playing) { pace = null; sim.set_view_time(NaN); tracePoint(true); postFrame(); }
  else if (waiting) setTimeout(slice, IDLE_MS, gen);
  else yieldThenSlice();
}

function slice(gen) {
  if (!sim || !playing || gen !== generation) return;
  if (perTurn !== null) return pacedSlice(gen);
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
    if (code === AWAITING) { playing = false; break; }   // the adversary's turn to choose
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
    time: sim.frame_time(),  // the run's time, or the time part-way along a walk being paced
    originX: sim.frame_origin_x(),
    originY: sim.frame_origin_y(),
    events: sim.event_count(),
    ticks: sim.tick_count?.() ?? 0,
    robots: n,
    terminated: sim.terminated_count(),
    epochs: sim.sequential() || sim.in_rounds() ? sim.epochs_completed() : null,
    turns: sim.sequential() ? sim.turn_count() : sim.in_rounds() ? sim.round_count() : null,
    unit: sim.in_rounds() ? "round" : "turn",
    members: sim.in_rounds() ? Array.from(sim.round_members()) : null,
    awaiting: sim.awaiting_schedule(),
    compass: n <= 2000 && sim.compass ? Array.from(sim.compass()) : null,
    deferred: sim.deferred_looks ? sim.deferred_looks() : 0,
    // Mutual visibility, once the run is over: how many robots see all the others (a Look each, so only then).
    seeing: mutualVisibility && stop !== "running" ? (seeing ??= sim.seeing_everyone()) : null,
    // While a turn is played one at a time: who is walking, in which beat, how far, and the settle time.
    walk: perTurn !== null && pace
      ? { robot: pace.robot, phase: pace.progress < 0 ? "decide" : "move", length: pace.length, cut: pace.cut,
          settle: pace.settle, algorithm: pace.algorithm }
      : null,
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
  pace = null;
  let lastEvent = null;
  if (unit === "event") {
    for (let i = 0; i < amount && !sim.ended(); i++) {
      const [time, robot, kind, outcome] = sim.step_one();
      lastEvent = { time, robot, kind: KIND[kind], outcome: OUTCOME[outcome] ?? String(outcome) };
    }
  } else {
    const code = sim.advance(0xffffffff, sim.time() + amount);
    if (code >= 2 && code !== AWAITING) stop = STOP[code];
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
        mutualVisibility = data.config.algorithm === "MutualVisibility";
        seeing = null;
        run = data.run ?? run + 1;
        playing = false;
        stop = "running";
        pace = null;
        batch = 64;
        trace = []; traceGap = 0; traceNext = 0; traceCostMs = 0; traceStartMs = performance.now();
        tracePoint(true);
        busyMs = 0;
        sinceMs = 0;
        postFrame({ notes: JSON.parse(wasm.notes(JSON.stringify(data.config))) });
        break;
      case "play":
        if (sim && stop === "running" && !playing) { playing = true; generation++; reanchor(); yieldThenSlice(); }
        break;
      case "speed":
        setSpeed(data.value);
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
        if (sim) {
          const info = JSON.parse(sim.robot_info(data.id));
          // A Look costs a pass over the swarm (more for some algorithms): only for small swarms.
          if (info && sim.robot_count() <= 2000) info.why = sim.explain(data.id) ?? null;
          self.postMessage({ type: "robot", id: data.id, seq: data.seq, info });
        }
        break;
      case "turn":
      case "round":
        if (!sim || stop !== "running") break;
        if (data.type === "turn") sim.push_turn(data.robot, data.stop ?? NaN);
        else sim.push_round(Uint32Array.from(data.robots));
        playing = true; generation++; reanchor(); yieldThenSlice();
        break;
      case "preview": {
        if (!sim) break;
        const p = sim.preview_target(data.robot);
        self.postMessage({ type: "preview", robot: data.robot, seq: data.seq, target: p.length ? [p[0], p[1]] : null });
        break;
      }
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
