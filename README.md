# LCM Simulator

A simulator for **Look-Compute-Move (LCM) robots**: identical, oblivious point robots that repeatedly *look* at where the others are, *compute* a destination with their algorithm, and *move* towards it. You choose the algorithm, the scheduler (asynchronous or sequential), the visibility, the movement model and the faults, then watch the swarm or run it headless.

The simulation core is written in Rust. It runs natively (command line, tests) and, compiled to WebAssembly, inside a background worker of the browser page, with no server and no GPU. It is deterministic: the same configuration and seed always give the same run, event for event. Its results were checked against recorded runs of the original Python implementation (kept as `fixtures/`).

**Highlights:** 10,000 robots at 60 fps on the CPU alone; about 350 KB to download; 99–357× faster in the browser (135–784× natively) than the Python engine it replaced.

## Try it

No install, runs entirely in your browser.

- Simulator: <https://drdebmath.github.io/LCMmodel/>
- Interactive flowcharts with the code of every step: <https://drdebmath.github.io/LCMmodel/web/docs.html>

Locally: `node scripts/serve.mjs`, then open <http://localhost:8000/> (the server turns browser caching off, so edits show at once).

## Contents

- [What it simulates](#what-it-simulates)
- [Algorithms](#algorithms)
- [Schedulers and movement](#schedulers-and-movement)
- [Gathering under the sequential scheduler](#gathering-under-the-sequential-scheduler)
- [Watching small moves](#watching-small-moves)
- [Sharing a case: links and embedding](#sharing-a-case-links-and-embedding)
- [Command line](#command-line)
- [How the Rust implementation is built](#how-the-rust-implementation-is-built)
- [Development and verification](#development-and-verification)
- [Repository layout](#repository-layout)

## What it simulates

- **Robots** are points in the plane with a speed. Each is always in one of four states: waiting, looking, moving or crashed.
- **Look** gives a robot a snapshot of the robots it can see (everyone, or those within a *visibility radius*), with optional **multiplicity detection** (it can tell when several robots share a point).
- **Compute** runs the robot's algorithm on the snapshot. Robots are *oblivious*: nothing is remembered from one look to the next.
- **Move** walks towards the destination at the robot's speed. *Rigid* movement always arrives; *non-rigid* movement lets an adversary stop the robot after at least δ.
- **Faults**: crash, Byzantine, omission and delay faults can be assigned to some robots, and robots can carry coloured lights (the luminous-robot model).
- **Starts**: random in a box, exact coordinates, or generated shapes (square, rectangle, circle, clusters, cloud; inside, on the edge or in a ring; random or even spacing), with optional stacked points.

## Algorithms

| Name on the page | Key | What the robots do |
| --- | --- | --- |
| Center of Gravity | `Gathering` | Move to the average position of the robots they see. |
| Smallest Enclosing Circle | `SEC` | Move onto the smallest circle enclosing the robots they see. |
| Uniform Circle Formation | `CircleFormation` | Spread evenly on a circle. |
| Go To Center | `GoToCenter` | Head for the centre of the smallest enclosing circle without losing sight of a neighbour. |
| Gathering under the sequential scheduler | `SqGathering` | Gather with weak multiplicity detection (Algorithm 8 of [arXiv:2412.10733](https://arxiv.org/abs/2412.10733)); needs the sequential scheduler. |

Every step of each algorithm, with the code that runs it, is on the [flowchart page](https://drdebmath.github.io/LCMmodel/web/docs.html).

## Schedulers and movement

The core runs robots under one of two schedulers, chosen in Settings → Scheduler (or `scheduler` in the config, see [`docs/core-schema.md`](docs/core-schema.md)).

- **Async** (the default, as in the original): every robot wakes on its own random clock (rate λ), so several can be moving at once and a robot can see another halfway through a move.
- **Sequential**: one robot at a time does a whole Look-Compute-Move while all the others stay still, as in the sequential scheduler of [Universal pattern formation by oblivious robots under sequential schedulers](https://arxiv.org/abs/2412.10733) (Section 2.2). Choose the **turn order** (round-robin, or a fresh random order each epoch drawn from the seed) and the **pause between turns** (random with rate λ, or none). An epoch is over once every robot that is still active has had a turn; the page shows the epoch count. A run whose robots can never move again ends as *stalled* instead of looping forever, and a **turn limit** stops a run after that many turns.
- **Non-rigid movement:** turn *Rigid movement* off and set a minimum move **δ**. A robot that is stopped on the way still covers at least δ (or arrives, if its destination is within δ), and an adversary chooses where it stops: right after δ, at random, or after a fraction of the way. With δ left at 0 a non-rigid robot still arrives, as in the original. In the config an explicit `schedule` can also list the turns, and name the stopping point of each.
- **Stacked starts:** under Start → More options, *Stacked points* makes the run begin with that many spots where several robots stand on top of each other (multiplicities), for algorithms that must cope with robots sharing a point.

## Gathering under the sequential scheduler

This is Algorithm 8 (Section 6) of [Universal pattern formation by oblivious robots under sequential schedulers](https://arxiv.org/abs/2412.10733): **gathering with weak multiplicity detection, under the sequential scheduler**. A robot sees which points are occupied and, for each, whether it holds more than one robot (never how many). With κ the number of such multiplicity points, the robot taking its turn:

1. κ = 1: stays if it is on the multiplicity, otherwise moves to it;
2. κ > 1: if it is on a multiplicity, moves to the midpoint between its point and the closest occupied point; otherwise stays;
3. κ = 0: moves to the closest point occupied by another robot.

It only runs under the sequential scheduler (choosing it on the page switches the scheduler), and it refuses limited visibility and faults, which the paper's model does not have. It is not the same as Center of Gravity gathering. Under the key `SqGathering` it appears as chart 6 of the flowchart page. `web/sqgathering.html` shows it one turn at a time with hand traces, and `lcm seq <config.json> --trace -` prints a turn-by-turn trace on the command line. Details: [`docs/sqgathering.md`](docs/sqgathering.md).

## Watching small moves

robots that gather end up very close together, so the page has tools to follow them (press `?` on the page for all the keys).

- **Follow** (`Z`, or the target button): the view zooms in as the swarm shrinks, down to the limit of the numbers (positions are drawn as offsets from a moving origin, so they stay sharp zoomed in a billion times). If a run stalls because the robots are closer than the numbers can tell apart, the page says so.
- **A turn at a time:** under the Sequential scheduler, the speed menu has *0.25 s/turn*, *1 s/turn* and *3 s/turn*. Each turn is shown in two beats: the robot looks (a ring, lines to the robots it sees, a ✕ on its destination), then it walks there, with a caption saying what the rule is. The panel counts turns and epochs.
- **Grid** (`G`): labelled lines that keep a sensible spacing (1, 2, 5, 10, …) at any zoom; the panel shows the spacing and how far in the view is.
- **Overview map** (`O`, on wide screens): the starting area with a rectangle for what the view shows; click it to move there.
- **Measure** (`M`): click two points or two robots to read the distance (a ruler stuck to robots follows them as they move; `Esc` clears).

## Sharing a case: links and embedding

set a case up on the page, then in the export menu (the download button) choose *Copy link to this case*, or *Embed in a web page…* for the `<iframe>` code to paste into another page (you pick the buttons, the speed and the size). The page can also be set up from its link by hand, e.g. `?algorithm=SqGathering&robots=6&seed=7&speed=turn:1`. Any setting can be given by its key (see `web/settings.js`), plus `robots`, `seed`, `visibility` (a radius) and `points=x,y;x,y;…` (exact starting positions); `speed` is a playback speed (`1`, `5`, `20`, `100`, `max`, or `turn:0.25`, `turn:1`, `turn:3`), and `theme` is `light` or `dark`. Add `embed` to put it in another web page with an `<iframe>`: only play/pause, reset and the speed menu show (`controls=` picks others from `play,reset,step,speed`), it starts playing the first time it is scrolled into view (`autoplay=0` turns that off) and after that only the button plays it (`loop=1` starts it again after it ends); `grid=0` hides the grid, `links=1` draws who sees whom, `open=0` drops the link to the full page; the wheel and fingers scroll the page around it; the view keeps the swarm (and any circle its algorithm draws) in the middle of the box, zooming out if it spreads but never in; *Open in simulator ↗* opens the same case in the full page. The box is as big as the iframe: set its `width` and `height`. For example:

```html
<iframe src="https://drdebmath.github.io/LCMmodel/?embed&algorithm=GoToCenter&points=-80,10;0,0;95,-5&visibility=100&scheduler=sequential&speed=turn:1"
        width="100%" height="420" loading="lazy" style="border:1px solid #ccc; border-radius:8px"></iframe>
```

## Command line

`cargo build --release -p lcm-cli`, then `./target/release/lcm run config.json` runs a configuration (the same settings as the page, see [`docs/core-schema.md`](docs/core-schema.md)) and prints a one-line JSON summary. Add `--trace trace.jsonl` (or `trace.csv`) to also write what every robot event did: the robot, the event, its position and target, the circle its algorithm computed, and at each Look which robots it could see. `--ticks` adds the clock ticks. For example, Go-To-Center on a chain of three robots with limited sight, one at a time:

```json
{"algorithm": "GoToCenter", "num_of_robots": 3, "initial_positions": [[-80, 10], [0, 0], [95, -5]],
 "visibility_radius": 100, "scheduler": "sequential", "turn_gap": "none", "robot_speeds": 20}
```

## How the Rust implementation is built

Four crates in one Cargo workspace (`Cargo.toml`), layered so that nothing below knows about anything above:

```
lcm-core        the simulation: robots, events, scheduler, starts, frames     (no I/O, no threads)
   ▲
lcm-algorithms  the algorithms and the registry that names them
   ▲        ▲
lcm-cli    lcm-wasm     native command line  /  browser bindings (wasm-bindgen)
```

### `lcm-core`: the simulation

The core has no DOM, file I/O, threads or GPU, so the same code runs natively and in the worker. Its contract (settings, state, the order of random draws, the events, every deliberate rule) is [`docs/core-schema.md`](docs/core-schema.md).

| Module | Responsibility |
| --- | --- |
| `config.rs` | `SimConfig`, the one description of a run (algorithm, robots, seed, speeds, visibility, scheduler, faults, limits). `validate()` rejects bad settings with a `ConfigError`; `resolve_positions()` fixes the start. Serialised with serde (feature `serde`) as the JSON the page and the CLI send. |
| `start.rs` | Starting positions: `StartSpec` = pattern × arrangement × distribution, plus stacked points (multiplicities). Draws come from a generator of their own, so a start never disturbs the run's random stream. |
| `robots.rs` | `Robots`, robot state as a **structure of arrays** (positions, targets, states, faults, lights, …), so 10,000 robots stay cache friendly. `position_at(i, t)` interpolates a moving robot at any time. |
| `event.rs` | `Event` (time, robot, kind: Crash, Look, Visualize, Wait) and `EventQueue`, a min-heap with a fixed tie-break order. |
| `sim.rs` | `Simulation`, the event loop. Async: every robot has its own random clock and events are handled in time order. Sequential: one robot at a time gets a whole turn, in an order that repeats each *epoch*. Handles rigid and non-rigid moves, faults, lights, stalls and limits. `advance(Budget)` runs until a budget, a limit or the end and returns a `StopReason`; `step()` handles one event. |
| `algorithm.rs` | The interface to algorithms: the `Algorithm` trait, `Look` (what a robot sees), `View` (the snapshot), `Decision` (target, terminate, circle to draw), `Scratch` (reusable buffers) and `Plan` (which robot runs which algorithm). |
| `frame.rs` | `Frame`: what leaves the core for drawing: typed arrays of positions (as offsets from a moving origin, so a swarm shrinking a billion-fold stays sharp), flags, lights, faults, targets, circles and multiplicities. Buffers are reused every frame. |
| `rng.rs` | `Rng`, the only source of randomness (SplitMix64), so a seed fixes a run. |
| `geom.rs`, `pyfloat.rs` | Points, circles and distances; and float helpers that reproduce the rounding of the original implementation exactly (compensated sums, round-half-even), which is what makes runs match the recorded ones. |

**One event.** `Simulation::step` pops the earliest event. A *Look* builds the robot's `View` (everyone in range, in ID order), calls `Algorithm::compute` and records the target; if the robot has somewhere to go it starts moving and a *Wait* event is queued at its arrival time. A *Wait* ends the move. *Crash* and *Visualize* events model faults and the clock tick. Under the sequential scheduler the same steps run, but the next robot's Look is queued only after the previous robot has finished its move.

### `lcm-algorithms`: the algorithms

Each algorithm is one module that implements `lcm_core::Algorithm`: `gathering.rs`, `sec.rs`, `circle_formation.rs`, `go_to_center.rs` and `sq_gathering.rs`. `lib.rs` holds `REGISTRY`, a list of entries (`key`, display `name`, `build`); `build` makes the `Plan` from the run's `SimConfig` and may refuse settings the algorithm cannot work with (`ConfigError::Unsupported`). `simulation(&config)` looks the algorithm up and builds a ready `Simulation`. `sq_trace.rs` replays a sequential run turn by turn, for the CLI and the step-by-step page.

**Adding an algorithm:** write the module, add one line to `REGISTRY`, add a flowchart definition if you want one, and add tests (see [AGENTS.md](AGENTS.md)).

### `lcm-cli`: the native runner

The `lcm` binary: `lcm run <config.json>` (one run, optional trace file), `lcm seq` (gathering under the sequential scheduler, turn by turn) and `lcm bench` (events per second on this machine). It is also what the parity scripts compare the browser build against.

### `lcm-wasm`: the browser bindings

A thin adapter, with no simulation logic: it parses the settings JSON, drives `Simulation::advance` and copies frames into typed arrays that the worker can transfer to the page without copying. It also lists the algorithms (`algorithms()`) so the page's menus come from the registry. It is built by `scripts/build-wasm.sh` into `web/pkg/`, which is committed.

### The browser page (`web/`)

Plain ES modules, no bundler.

| File | Role |
| --- | --- |
| `index.html` (root), `web/index.html` | The simulator page and a redirect from the old `/web/` address. |
| `app.js` | The page: settings panel, toolbar, HUD, run states (loading, ready, running, paused, ended). |
| `settings.js`, `start.js` | The settings, defined once; the panel and the `SimConfig` are built from them. |
| `client.js`, `worker.js` | The page side and the worker side of the protocol. The worker owns the Rust core, simulates in ~6 ms slices and sends a frame whenever the page asks, so drawing never slows the simulation. |
| `renderer.js` | The CPU Canvas2D renderer: zoom, follow, grid, measure, sprites for larger robots. |
| `overview.js`, `inspector.js` | The overview map and the per-robot inspector. |
| `embed.js` | A run set up from the page's link; the `?embed` version for `<iframe>`s. |
| `docs.html`, `docs.js`, `docs.css` | The flowchart page, generated by `scripts/gen-flowcharts.mjs`. |
| `sqgathering.*` | The step-by-step page for gathering under the sequential scheduler. |
| `pkg/` | The built core: `lcm_wasm.js`, `lcm_wasm_bg.wasm`, `build.json` (a content hash the page uses to avoid mixing old and new files). |

## Development and verification

Requires Rust (stable), Node ≥ 22 and Chrome (for the page checks).

```sh
rustup target add wasm32-unknown-unknown
cargo install wasm-bindgen-cli --version 0.2.129 --locked
cargo test --workspace                        # Rust tests, including replays of the recorded runs in fixtures/
cargo build --release -p lcm-cli && ./scripts/build-wasm.sh
node scripts/check-wasm-parity.mjs            # browser build vs native, bit for bit
node scripts/check-seq-parity.mjs             # sequential-scheduler traces: browser build vs native
node scripts/gen-flowcharts.mjs --check       # flowcharts match the code (omit --check to regenerate)
node scripts/ui-check.mjs                     # the simulator page in headless Chrome
node scripts/docs-check.mjs                   # the flowchart page in headless Chrome
node scripts/responsive-check.mjs             # both pages at phone, tablet and desktop sizes
node scripts/sqgathering-check.mjs            # the step-by-step page and the simulator
node scripts/embed-check.mjs                  # set up from a link, and embedded in another page
node scripts/serve.mjs                        # serve at http://localhost:8000/ with no caching
```

The same checks run on GitHub's machines for every push and pull request (`.github/workflows/ci.yml`). Contributing: see [AGENTS.md](AGENTS.md).

## Repository layout

- `crates/`: the Rust workspace (`lcm-core`, `lcm-algorithms`, `lcm-cli`, `lcm-wasm`); unit tests inside modules, integration tests in each crate's `tests/`.
- `web/`: the browser pages; `index.html` at the root opens the simulator.
- `docs/`: [`core-schema.md`](docs/core-schema.md) (the core's contract), `flowcharts.md` (generated), [`sqgathering.md`](docs/sqgathering.md).
- `fixtures/`: recorded runs of the original implementation (and configs) that the tests replay.
- `scripts/`: build (`build-wasm.sh`), parity and page checks (`*.mjs`), the flowchart generator and the dev server.
- [`report.md`](report.md): the full report on the port (method, results, every test).
