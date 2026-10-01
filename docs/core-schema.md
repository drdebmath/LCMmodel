# LCM core schema

This is the contract for `lcm-core`: what a simulation is configured with,
what state it holds, what order things happen in, what an algorithm may see
and return, and what leaves the core for rendering. The Rust types in
`crates/lcm-core` are the executable form of this document; when they
disagree, one of them is a bug.

Scope: the core has no DOM, no file I/O, no JSON in its hot path, no threads
and no GPU. It must run 10,000 robots inside a browser Web Worker on the CPU.

## 1. Reference semantics

The Python simulator (`robot.py`, `scheduler.py`, `run.py` at commit
`e68bb8f`) is the behavioural reference. The Rust core reproduces its math
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
| `algorithm` | string | A registry key. Ported so far: `Gathering` (the others follow once the core is verified) |
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
| `max_events`, `max_time` | u64 / f64 or null | safety limits, same semantics as `run.py` (§6) |
| `open_world` | bool | `true`: no world box, algorithms get no bounds. Default `false` (the original) |
| `start` | object or null | a generated start (§4.3); null keeps the original start |

Invalid values are a `ConfigError`, never a silent default.

## 3. State (structure of arrays)

Robot `i` is index `i` in every array. IDs are dense `u32`.

| Array | Type | Meaning |
| --- | --- | --- |
| `pos` | `Point` | position at the last `Wait` (Python `coordinates`) |
| `start_pos` | `Point` | position when the current move began |
| `target` | `Option<Point>` | last computed destination (`calculated_position`) |
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
`position_at(i, t)` interpolates `start_pos → target` at `speed` while
`state == Move`, exactly like Python `get_position`.

## 4. Randomness

### 4.1 Generator

One `SplitMix64` stream seeded with `random_seed`, consumed in exactly the
order Python consumes `self.generator`. The Python reference harness
(`reference/portable_rng.py`) implements the same generator so fixtures are
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
   (Python `run.py` does this) draws all `x = uniform(−W/2, W/2)`, then all
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
`run.py`'s checks: time is tested before each event, the event counter after
it is incremented.

## 7. Algorithm interface

```rust
pub trait Algorithm: Send + Sync {
    fn key(&self) -> &'static str;
    fn compute(&self, look: &Look<'_>, rng: &mut Rng, scratch: &mut Scratch) -> Decision;
}

pub struct Look<'a> {
    pub me: u32,             // my robot id
    pub me_index: usize,     // my index inside `view`
    pub position: Point,     // my position
    pub view: &'a View,      // visible robots, ID order, including me
    pub model: &'a Model,    // ε, precision, visibility, world bounds
}

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

Adding an algorithm = one file implementing `Algorithm` plus one registry
line in `lcm-algorithms`.

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

`reference/gen_fixtures.py` runs the unmodified Python modules with the
portable generator and records, per event, `(time, robot, kind, outcome)`
and the acting robot's resulting position, target, state and flags, plus the
final state. `cargo test -p lcm-algorithms --test parity` runs the Rust core on the settings of every
fixture and requires identical event kinds, robots and outcomes, and times
and coordinates equal within 1e-9 relative. A divergence is reported with the
first differing event.
