# LCM core schema

This is the contract for `lcm-core`: what a simulation is configured with,
what state it holds, what order things happen in, what an algorithm may see
and return, and what leaves the core for rendering. The Rust types in
`crates/lcm-core` are the executable form of this document; when they
disagree, one of them is a bug.

Scope: the core has no DOM, no file I/O, no JSON in its hot path, no threads
and no GPU. It must run 10,000 robots inside a browser Web Worker on the CPU.

## 1. Reference semantics

The original Python simulator (commit `e68bb8f`, since removed from the
repository) is the behavioural reference; its recorded runs are in `fixtures/`. The Rust core reproduces its math
exactly, including float details that change results:

| Python construct | Rust equivalent (`lcm_core::pyfloat`) |
| --- | --- |
| `sum(floats)` (CPython ≥ 3.12, Neumaier-compensated) | `py_sum` |
| `x += v` loops | plain `+=` in the same order |
| `a % m` on floats (sign follows divisor) | `py_mod` |
| `round(x, p)` (ties to even) | `py_round` |
| `10 ** -p`, `math.pow(10, -p)` | `pow10_neg(p)` (correctly rounded literal) |
| `math.dist`, `math.hypot` | `libm::hypot` |
| `math.sin/cos/atan2/sqrt/log1p` | `libm::*` (identical on native and wasm) |
| `min(seq)` / `max(seq)` / `min(key=…)` | first extremum wins |
| `sorted(key=…)` | stable sort on the same key tuple |

The only intended differences are listed in §9.

## 2. Configuration (`SimConfig`)

JSON form (serde, camelCase is **not** used; names match `main.js params()`):

```json
{
  "algorithm": "Gathering",
  "num_of_robots": 10000,
  "initial_positions": null,
  "robot_speeds": 1.0,
  "visibility_radius": null,
  "num_of_faults": 0,
  "fault_type": "crash",
  "rigid_movement": true,
  "width_bound": 600,
  "height_bound": 600,
  "lambda_rate": 5.0,
  "sampling_rate": 0.1,
  "threshold_precision": 5,
  "random_seed": 12345,
  "multiplicity_detection": false,
  "max_events": null,
  "max_time": null
}
```

| Field | Type | Rule |
| --- | --- | --- |
| `algorithm` | string | A registry key. Ported so far: `Gathering`, `SEC`, `CircleFormation`, `GoToCenter`, `SqGathering`, `PatternFormation`, `SqPatternFormation` (the others follow once the core is verified) |
| `num_of_robots` | u32 | 1 ..= 100 000 |
| `initial_positions` | `[[x,y],…]` or null | If null or its length ≠ `num_of_robots`, positions are drawn (§4.3) |
| `robot_speeds` | f64 | > 0, finite |
| `visibility_radius` | f64 or null | null = unlimited; else > 0 |
| `num_of_faults` | u32 | clamped to `num_of_robots` |
| `fault_type` | string | `crash`, `byzantine`, `omission`, `delay`, `mixed` |
| `rigid_movement` | bool | |
| `width_bound`, `height_bound` | f64 | 0 or null = unbounded |
| `lambda_rate` | f64 | > 0: activation rate of the exponential clock |
| `sampling_rate` | f64 | > 0: period of `Visualize` events |
| `threshold_precision` | u8 | 1 ..= 15; ε = 10^-p |
| `random_seed` | u64 | |
| `multiplicity_detection` | bool | display only; no algorithm reads it (§9) |
| `max_events`, `max_time` | u64 / f64 or null | safety limits, same semantics as the original simulator (§6) |
| `open_world` | bool | `true`: no world box, algorithms get no bounds. Default `false` (the original) |
| `start` | object or null | a generated start (§4.3); null keeps the original start |
| `scheduler` | string | `async` (the original, default); `sequential`: one robot at a time does a whole Look-Compute-Move while the rest stay still; `ssync`: rounds in which a non-empty subset of the robots looks at the same instant, computes and moves, the round ending once all of them have stopped; `fsync`: rounds in which every live robot is active; `kasync`: as `async`, but while one robot is in a cycle (its Look until the end of its move) no other robot starts more than `k_async` Looks (Kirkpatrick et al. 2024); a Look that would is put off until that cycle ends |
| `ssync_probability` | f64 | `ssync`: chance in (0, 1] that a live robot is active in a round (default 0.5). A round with nobody drawn activates one robot at random, so rounds are never empty |
| `ssync_fairness` | u32 | `ssync`: a robot left out of this many rounds in a row (≥ 1, default 4) is activated in the next one |
| `rounds` | list or null | `ssync` only: the robots of each round, in order (`[[0, 1], [2]]`), instead of random draws. Every round names at least one existing robot |
| `max_rounds` | u64 or null | `ssync`/`fsync` only: stop after this many rounds (`advance` returns `MaxRounds`). null: no limit |
| `k_async` | u32 | `kasync`: the bound k, at least 1 (default 1) |
| `multiplicity` | string | what an algorithm reading `Look::occupied` (§7) can tell about points several robots share: `none` (only the set of occupied points), `weak` (default: one robot or more than one), `strong` (the number of robots), `local` (whether its own point holds more than one robot, nothing about other points; Kamei et al. 2011). The ported algorithms read the raw view and ignore it |
| `frames` | string | the coordinate system each robot computes in (§7.1): `global` (default, the original), `rotated`, `disoriented`, `compass` |
| `compass_error` | f64 | `frames` `compass`: the largest deviation of a robot's north from the global north, in degrees, 0 ..= 180 (default 0) |
| `compass_static` | bool | `frames` `compass`: true (default) draws one deviation per robot for the whole run; false draws a new one at every Look |
| `obstructed_visibility` | bool | a robot does not see robots hidden behind another robot (default false) |
| `robot_radius` | f64 | ≥ 0: robots are discs of this radius when they block a view; 0 (default): points, which block only when in line to within ε |
| `activation_order` | string | sequential only: `round_robin` (0, 1, …, n-1, default) or `random` (a fresh shuffle each epoch, from the seed). Every epoch gives each robot that is neither crashed nor terminated exactly one turn |
| `delta` | f64 or null | non-rigid movement (`rigid_movement` false): a robot always covers at least `delta` along its way, or reaches its destination if that is nearer. null (default): a non-rigid robot still reaches its destination, as in the Python original |
| `stop_policy` | string | where a robot is stopped once its destination is farther than `delta`: `delta` (exactly `delta`), `random` (default: uniform between `delta` and the destination, from the run's random stream), `fraction` (`stop_fraction` of the way, never less than `delta`) |
| `stop_fraction` | f64 | in (0, 1], default 0.5; used by `stop_policy` `fraction` |
| `schedule` | list or null | sequential only: the turns in order, instead of `activation_order`. Each turn is a robot index, or `{"robot": i, "stop": f}` where `f` in (0, 1] is the fraction of the way the adversary lets that robot travel (needs non-rigid movement and `delta`). Turns of crashed or terminated robots are skipped. An epoch ends once every robot live at its start has had a turn |
| `schedule_end` | string | what follows the end of `schedule` (sequential) or `rounds` (ssync): `round_robin` (default) hands the run back to the scheduler's own fair choice, which is turns 0, 1, 2, … for sequential and random rounds for ssync (the name is the sequential meaning); `repeat` plays the list again (it must name every robot); `await` waits for the next turn or round to be added while the run goes on (`push_turn`, `push_round`; `advance` returns `AwaitingSchedule`), and needs a `schedule` or `rounds` list, which may start empty. `await` is refused with any other scheduler, where there is no list to wait on |
| `max_turns` | u64 or null | sequential only: stop after this many turns (one per Look); `advance` then returns `MaxTurns`. The last turn's move still finishes. null: no limit |
| `multiplicities` | u32 | start with this many multiplicity points (robots standing on one spot), made from the start positions whatever their source: that many robots, chosen at random from `random_seed` (a stream of its own), become anchors and other robots are moved onto them. 0 (default): none |
| `multiplicity_size` | u32 | robots on each starting multiplicity, at least 2; 0 (default) picks a size: a third of the robots for one multiplicity, up to 4 each for several. `multiplicities` × `multiplicity_size` must not exceed `num_of_robots` |
| `turn_gap` | string | sequential only: `random` (exponential with rate `lambda_rate`, default) or `none` (the next Look at the instant the last turn ends) |

Invalid values are a `ConfigError`, never a silent default.

## 3. State (structure of arrays)

Robot `i` is index `i` in every array. IDs are dense `u32`.

| Array | Type | Meaning |
| --- | --- | --- |
| `pos` | `Point` | position at the last `Wait` (Python `coordinates`) |
| `start_pos` | `Point` | position when the current move began |
| `target` | `Option<Point>` | last computed destination (`calculated_position`) |
| `stop` | `Option<Point>` | non-rigid movement with a `delta`: where the adversary stops the robot on its way to `target`; `None` when it walks all the way. Only the move itself uses it, so `target` keeps showing what the algorithm computed |
| `start_time` | `Option<f64>` | move start; `None` when not moving |
| `speed` | f64 | after the delay fault's ×0.4 |
| `state` | `RobotState` | `Wait`, `Look`, `Move`, `Crash` |
| `frozen`, `terminated` | bool | |
| `fault` | `FaultKind` | `None`, `Crash`, `Byzantine`, `Omission`, `Delay` |
| `light` | `Option<Light>` | `Blue` (look), `Red` (move), `Green` (wait) |
| `last_light_time` | f64 | light cooldown (0.5 time units) |
| `algo` | u8 | index into the simulation's algorithm table |
| `task` | `Option<TaskColor>` | TwoTask colour (`Red` = SEC, `Blue` = Gathering) |
| `overlay` | `Option<Circle>` | the circle the robot last computed (Python `robot.sec`) |
| `travelled` | f64 | total distance moved |

The position of robot `i` at time `t` is derived, never stored:
`position_at(i, t)` interpolates `start_pos → stop` (else `target`) at `speed` while
`state == Move`, exactly like Python `get_position`.

## 4. Randomness

### 4.1 Generator

One `SplitMix64` stream seeded with `random_seed`, consumed in exactly the
order Python consumes `self.generator`. The recorded fixtures were produced with the same generator, so they are
reproducible bit for bit.

| Method | Definition |
| --- | --- |
| `next_u64` | SplitMix64 step |
| `random()` | `(next_u64 >> 11) · 2⁻⁵³` in [0, 1) |
| `uniform(a, b)` | `a + (b − a) · random()` |
| `exponential(scale)` | `−scale · log1p(−random())` |
| `integers(0, n)` | rejection: `threshold = (2⁶⁴ − n) mod n`; draw until `v ≥ threshold`; return `v mod n` |
| `shuffle(xs)` | Fisher–Yates from the end: for `i = len−1 … 1`, swap `i` with `integers(0, i+1)` |
| `choice(n, k)` | partial Fisher–Yates on `0..n`: for `i < k`, swap `i` with `i + integers(0, n−i)`; return the first `k` |

### 4.2 Consumption order

1. Per robot in ID order, TwoTask only: `random() < 0.5` ⇒ SEC/red, else Gathering/blue.
2. Faults: `choice(n, k)`; the j-th chosen robot gets `fault_type`, or `ALL[j mod 4]` for `mixed`.
3. Initial activations: `exponential(1/λ)` for every robot in ID order.
4. During the run: `exponential(1/λ)` per rescheduled activation (also consumed
   when the robot is terminated and nothing is scheduled); Welzl
   `shuffle` + `integers`; Byzantine `uniform(0, 2π)`; Omission `random()`.

### 4.3 Initial positions

In this order:

1. `initial_positions`, if it has one entry per robot (custom coordinates; as in Python).
2. `start`, if set: generated by `start::generate` (crates/lcm-core/src/start.rs) from
   three choices. The **pattern** (`cloud`, `square`, `rectangle`, `circle`, `polygon`,
   `line`, `clusters`) is the overall shape; the **arrangement** (`filled`, `edge`, `ring`;
   circles allow all three, squares, rectangles and polygons `filled` or `edge`, a line
   only `edge`, cloud and clusters only `filled`) says inside the shape or on its edge;
   the **distribution** (`uniform`, `even`, `jittered`, `gaussian`; edges and rings have
   no `gaussian`, a cloud is always `gaussian`, clusters `gaussian` or `uniform`) says
   how robots are spaced. Parameters: `size`, `aspect`, `inner`, `sides`, `clusters`,
   `spread` (σ), `jitter`, `rotation` (degrees). Any other combination is rejected by
   `validate`. Random draws use their own generator seeded with
   `random_seed ^ 0xA0761D6478BD642F`, so the start is independent of the activation
   draws; `even` placements use no randomness. These starts are new: the Python has
   no equivalent.
3. Otherwise the original: a **separate** generator seeded with the same seed
   (the original simulator does this) draws all `x = uniform(−W/2, W/2)`, then all
   `y = uniform(−H/2, H/2)`, with W, H defaulting to 100.

The world box only places robots and is drawn; robots are never kept inside it.

## 5. Events and ordering

```text
Event { time: f64, robot: i32 (−1 = visualize), kind: EventKind }
EventKind ∈ { Crash, Look, Wait, Visualize }
```

The queue is a binary min-heap ordered by `(time, robot, kind)`; kinds rank
in Python string order `Crash < Look < Terminated < Visualize < Wait`. Time
uses `f64::total_cmp`.

Handling one event (Python `handle_event`), in order:

1. Empty queue ⇒ `Ended`.
2. `time < now` ⇒ skipped (`Ignored`), clock unchanged. Else `now = time`.
3. `Visualize` ⇒ reschedule at `time + sampling_rate`, return `Visualize`.
   No robot state changes; no termination check.
4. Crashed robot, non-crash event ⇒ reschedule, `Ignored`. Terminated robot ⇒ `Ignored`.
5. `Look` ⇒ §7 look/compute; then `Terminated`, `Frozen` (reschedule), or
   `Moved` (schedule `Wait` at `time + dist/speed`; if that does not advance
   the clock the move is finalised immediately and the robot rescheduled).
6. `Wait` ⇒ finish the move, reschedule.
7. `Crash` ⇒ mark crashed.
8. Global termination: every robot that is neither crashed nor Byzantine is
   terminated (or every robot has crashed) ⇒ `Ended`.

`StepOutcome` mirrors Python's exit codes: `Ignored`(0), `Moved`(2),
`Frozen`(3), `Terminated`(4), `Crashed`(5), `Visualize`(99), `Ended`(−1).

## 6. Driving a simulation

```rust
let mut sim = Simulation::new(&config, plan)?;   // plan from lcm-algorithms
sim.step();                                     // one event
sim.advance(Budget { max_events, until_time }); // many events, returns why it stopped
sim.write_frame(sim.time(), &mut frame);        // render data (§8)
```

`advance` is how the browser runs: a frame's budget of events, then render.
Rendering frequency never changes results. `max_events` / `max_time` apply
the original simulator's checks: time is tested before each event, the event counter after
it is incremented.

`advance` also stops with `Stalled` under the sequential scheduler when a whole
epoch passes in which no robot moved (a move of at least ε) or terminated.
Robots are deterministic and the world did not change, so the next epoch would
be the same and the run could never end. This is checked only without faults
(an omission fault skips moves at random). It is what stops a run with
`turn_gap` `none` whose robots never move: simulated time would not advance at
all. The run is not `ended`; `sim.stalled()` is true.

Likewise `MaxTurns` ends a sequential run that has taken `max_turns` turns
(`sim.turn_limit_reached()`), and `MaxRounds` an SSYNC or FSYNC run that has
played `max_rounds` rounds; they say nothing about the algorithm, only that the
budget ran out. `AwaitingSchedule` is not an end: a `schedule_end` `await` list
has run out, and the run carries on once `push_turn` or `push_round` adds more.

## 7. Algorithm interface

```rust
pub trait Algorithm: Send + Sync {
    fn key(&self) -> &'static str;
    fn compute(&self, look: &Look<'_>, rng: &mut Rng, scratch: &mut Scratch) -> Decision;
    // Optional, with defaults:
    fn randomized(&self) -> bool { false }        // true: no stall check (§6)
    fn stops_when_alone(&self) -> bool { true }   // false: the algorithm decides termination
    fn explain(&self, look: &Look<'_>, scratch: &mut Scratch) -> Option<String> { None }
}

pub struct Look<'a> {
    pub me: u32,             // my robot id
    pub me_index: usize,     // my index inside `view`
    pub position: Point,     // my position ((0, 0) in a private frame, §7.1)
    pub view: &'a View,      // visible robots, ID order, including me
    pub model: &'a Model,    // ε, precision, visibility, world bounds, multiplicity model
}
// look.occupied(): the occupied points as `model.multiplicity` lets the robot
// perceive them, and the one it stands on (crates/lcm-core/src/occupancy.rs).

pub struct View {           // SoA, rebuilt in a reused buffer per look
    pub id: Vec<u32>,
    pub pos: Vec<Point>,
    pub state: Vec<RobotState>,
    pub terminated: Vec<bool>,
    pub frozen: Vec<bool>,
}

pub struct Decision {
    pub target: Point,
    pub terminate: bool,
    pub overlay: OverlayUpdate, // Keep | Set(Circle) | Clear
}
```

The view is a snapshot at the exact look time: moving robots are
interpolated, never stale. Algorithms may not keep state between looks
(robots are oblivious); `scratch` is reusable memory, not memory of the past.
What the core does around `compute` (Byzantine override, "only I am visible"
termination, omission, the freeze threshold) is fixed by §5 and is not the
algorithm's business.

### 7.1 Private frames (`frames`)

With `frames` other than `global`, the core gives each Look a frame of the
robot's own and translates in both directions; the algorithm never sees a
global coordinate. Its `Look` then contains exactly:

| Field | `global` | `rotated`, `disoriented`, `compass` |
| --- | --- | --- |
| `position` | the robot's global position | `(0, 0)`: the robot is the origin of its frame |
| `view.pos` | global positions | every visible robot (itself included, at `(0, 0)`) in the robot's own axes and unit length |
| `model.eps`, `model.visibility` | as configured | scaled to the robot's unit length, so "within ε" and "within sight" mean the same as globally |
| `model.width_bound`, `height_bound` | the world box | `None`: the box is drawn in global coordinates, which the robot does not know |
| IDs, states, `terminated`, `frozen` | unchanged | unchanged |

The frames themselves:

- `rotated`: a random rotation in [0, 2π) and a random unit length, ½ to 2
  local units per global unit (log-uniform); the common handedness.
- `disoriented`: as `rotated`, and the y axis is mirrored with probability ½:
  no common axes, unit length or handedness. These are the disoriented robots
  of the sequential-schedulers paper (Flocchini et al., arXiv:2412.10733).
- `compass`: the global axes turned by the robot's deviation, uniform in
  ±`compass_error` degrees; unit length and handedness are shared (Izumi et
  al. 2012). The deviation is drawn once per robot at the start
  (`compass_static`) or at every Look.

A **new frame is drawn at every Look** (for `compass`, a new deviation when the
compass is dynamic), so nothing about one Look's frame carries to the next.
The destination and any circle (`OverlayUpdate::Set`) the algorithm returns are
mapped back to global coordinates by the core; movement, non-rigid stops and
the rest of §5 happen in global coordinates as before.

Frames draw from their own generator, seeded with
`random_seed ^ 0x9E3779B97F4A7C15`, never from the run's stream (§4), so turning
frames on leaves the scheduler's draws unchanged: a frame-invariant algorithm
(one that uses only distances and angles between robots, like every ported
one; `tests/local_frames.rs` checks each registered algorithm) runs the same
events with frames on as off, up to rounding. `explain` and `preview_target`
use a copy of that generator and restore a dynamic compass's deviation, so
asking changes nothing about the run.

Adding an algorithm = one file implementing `Algorithm` plus one registry
line in `lcm-algorithms`. A registry entry builds the algorithm's `Plan` from the
run's `SimConfig` (so it can use the seed or the robot count) and may refuse
settings it cannot work with by returning `ConfigError::Unsupported`, for
example an algorithm that is only defined under the sequential scheduler.

## 8. Frame (core → renderer)

Written into caller-owned buffers so the worker can transfer them without
copying or JSON:

| Buffer | Type | Length | Content |
| --- | --- | --- | --- |
| `xy` | f32 | 2n | position at the frame time |
| `flags` | u8 | n | bits 0–1 state (0 wait, 1 look, 2 move, 3 crash), bit 2 frozen, bit 3 terminated, bit 4 has target |
| `light` | u8 | n | 0 none, 1 blue, 2 red, 3 green |
| `fault` | u8 | n | 0 none, 1 crash, 2 byzantine, 3 omission, 4 delay |
| `task` | u8 | n | 0 none, 1 red, 2 blue |
| `target` | f32 | 2n | target, NaN when none |
| `circle` | f32 | 3n | overlay `cx, cy, r`, NaN when none |

At 10,000 robots one full frame is about 280 KB.

## 9. Deliberate differences from the Python

| Python | Rust | Why |
| --- | --- | --- |
| `numpy` PCG64 | SplitMix64 (§4) | numpy's streams cannot be reproduced practically; seeds therefore map to different (equally valid) schedules |
| Multiplicity is computed in O(n²) on every snapshot | Computed only when a frame asks for it | No algorithm reads it; results are identical |
| Every look formats a log string of the whole swarm | No logging in the hot path | Output only |

Found while auditing, to be handled when those algorithms are ported: the
Python Welzl recurses n deep, so SEC, GoToCenter and TwoTask raise
`RecursionError` at n ≥ ~1000, every robot freezes and the run "terminates"
having done nothing.

## 10. Parity protocol

The fixtures in `fixtures/` were recorded from the original Python simulator,
unmodified except for a portable random generator, and hold, per event, `(time, robot, kind, outcome)`
and the acting robot's resulting position, target, state and flags, plus the
final state. `cargo test -p lcm-algorithms --test parity` runs the Rust core on the settings of every
fixture and requires identical event kinds, robots and outcomes, and times
and coordinates equal within 1e-9 relative. A divergence is reported with the
first differing event.
