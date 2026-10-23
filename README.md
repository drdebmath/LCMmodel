# Asynchronous Scheduler

A simulator for Look-Compute-Move robots in the Asynchronous System

## Rust + WebAssembly version (branch `rust-core`)

The simulator core has been ported to Rust and runs in the browser as WebAssembly, in a background worker, with **no Python or Pyodide**. The math is unchanged. The Rust core runs its own simulations from the settings (it does not need Python or any recorded data); to prove it matches, its tests run the same settings and seeds as recorded Python runs and compare the two event by event. So far the core and five algorithms (Gathering, Smallest Enclosing Circle, Uniform Circle Formation, Go-To-Center and SqGathering) are ported; the other algorithms follow.
New in the Rust core: **SqGathering** (Algorithm 8 of the sequential-schedulers paper), below.

**Try it** (no install, runs entirely in your browser, CPU only):

- New simulator (Rust core, up to 10,000 robots): <https://imstillsamarth.github.io/LCMmodel/>
- Interactive flowcharts with the code of every step: <https://imstillsamarth.github.io/LCMmodel/web/docs.html>
- The original Pyodide simulator, for comparison: <https://imstillsamarth.github.io/LCMmodel/old/>

**Highlights:** 99–357× faster than the Python engine in the browser on identical runs (135–784× natively); 10,000 robots at 60 fps without a GPU; about 350 KB to download instead of about 16 MB.

**Schedulers:** the core runs robots under one of two schedulers, chosen in Settings → Scheduler (or `scheduler` in the config, see [`docs/core-schema.md`](docs/core-schema.md)).

- **Async** (the default, as in the original): every robot wakes on its own random clock (rate λ), so several can be moving at once and a robot can see another halfway through a move.
- **Sequential**: one robot at a time does a whole Look-Compute-Move while all the others stay still, as in the sequential scheduler of [Universal pattern formation by oblivious robots under sequential schedulers](https://arxiv.org/abs/2412.10733) (Section 2.2). Choose the **turn order** (round-robin, or a fresh random order each epoch drawn from the seed) and the **pause between turns** (random with rate λ, or none). An epoch is over once every robot that is still active has had a turn; the page shows the epoch count. A run whose robots can never move again ends as *stalled* instead of looping forever, and a **turn limit** stops a run after that many turns.
- **Non-rigid movement:** turn *Rigid movement* off and set a minimum move **δ**. A robot that is stopped on the way still covers at least δ (or arrives, if its destination is within δ), and an adversary chooses where it stops: right after δ, at random, or after a fraction of the way. With δ left at 0 a non-rigid robot still arrives, as in the original. In the config an explicit `schedule` can also list the turns, and name the stopping point of each.
- **Stacked starts:** under Start → More options, *Stacked points* makes the run begin with that many spots where several robots stand on top of each other (multiplicities), for algorithms that must cope with robots sharing a point.

**SqGathering** (Algorithm 8, Section 6 of [arXiv:2412.10733](https://arxiv.org/abs/2412.10733)): gathering with weak multiplicity detection, under the sequential scheduler only (picking it on the page switches the scheduler to Sequential). Flowchart: chart 6 of the flowchart page. Step-by-step view with hand traces: `web/sqgathering.html`. Turn-by-turn trace on the command line: `lcm seq <config.json> --trace -`. Details: [`docs/sqgathering.md`](docs/sqgathering.md).

**Watching small moves:** robots that gather end up very close together, so the page has tools to follow them (press `?` on the page for all the keys).

- **Follow** (`Z`, or the target button): the view zooms in as the swarm shrinks, down to the limit of the numbers (positions are drawn as offsets from a moving origin, so they stay sharp zoomed in a billion times). If a run stalls because the robots are closer than the numbers can tell apart, the page says so.
- **A turn at a time:** under the Sequential scheduler, the speed menu has *0.25 s/turn*, *1 s/turn* and *3 s/turn*. Each turn is shown in two beats: the robot looks (a ring, lines to the robots it sees, a ✕ on its destination), then it walks there, with a caption saying what the rule is. The panel counts turns and epochs.
- **Grid** (`G`): labelled lines that keep a sensible spacing (1, 2, 5, 10, …) at any zoom; the panel shows the spacing and how far in the view is.
- **Overview map** (`O`, on wide screens): the starting area with a rectangle for what the view shows; click it to move there.
- **Measure** (`M`): click two points or two robots to read the distance (a ruler stuck to robots follows them as they move; `Esc` clears).

**A case in a link, and in your own page:** the page can be set up from its link, e.g. `?algorithm=SqGathering&robots=6&seed=7&speed=turn:1`. Any setting can be given by its key (see `web/settings.js`), plus `robots`, `seed`, `visibility` (a radius) and `points=x,y;x,y;…` (exact starting positions); `speed` is a playback speed (`1`, `5`, `20`, `100`, `max`, or `turn:0.25`, `turn:1`, `turn:3`), and `theme` is `light` or `dark`. Add `embed` to put it in another web page with an `<iframe>`: only play/pause, reset and the speed menu show (`controls=` picks others from `play,reset,step,speed`), it starts playing the first time it is scrolled into view (`autoplay=0` turns that off) and after that only the button plays it; the wheel and fingers scroll the page around it; the view keeps the swarm (and any circle its algorithm draws) in the middle of the box, zooming out if it spreads but never in; *Open in simulator ↗* opens the same case in the full page. The box is as big as the iframe: set its `width` and `height`. For example:

```html
<iframe src="https://drdebmath.github.io/LCMmodel/?embed&algorithm=GoToCenter&points=-80,10;0,0;95,-5&visibility=100&scheduler=sequential&speed=turn:1"
        width="100%" height="420" loading="lazy" style="border:1px solid #ccc; border-radius:8px"></iframe>
```

A sample page with two demos: `web/tests/embed-host.html`.

**What's where:** [`report.md`](report.md) is the full report (method, results, every test). [`docs/core-schema.md`](docs/core-schema.md) is the core's contract. The code is in `crates/` (Rust) and `web/` (browser page; `index.html` at the root opens it, the original page is in `old/`).

**From the command line:** `cargo build --release -p lcm-cli`, then `./target/release/lcm run config.json` runs a configuration (the same settings as the page, see [`docs/core-schema.md`](docs/core-schema.md)) and prints a one-line JSON summary. Add `--trace trace.jsonl` (or `trace.csv`) to also write what every robot event did: the robot, the event, its position and target, the circle its algorithm computed, and at each Look which robots it could see. `--ticks` adds the clock ticks. For example, Go-To-Center on a chain of three robots with limited sight, one at a time:

```json
{"algorithm": "GoToCenter", "num_of_robots": 3, "initial_positions": [[-80, 10], [0, 0], [95, -5]],
 "visibility_radius": 100, "scheduler": "sequential", "turn_gap": "none", "robot_speeds": 20}
```

**Verify it yourself** (Rust, Node ≥ 22, Chrome; Python with numpy for the Python comparisons):

```sh
rustup target add wasm32-unknown-unknown
cargo install wasm-bindgen-cli --version 0.2.129 --locked
cargo test --workspace                        # Rust tests, including the Rust core vs recorded Python runs
cargo build --release -p lcm-cli && ./scripts/build-wasm.sh
node scripts/check-wasm-parity.mjs            # browser build vs native, bit for bit
node scripts/check-seq-parity.mjs             # SqGathering traces: browser build vs native
python3 scripts/compare-speed.py 20 50 100    # the same run in Python and Rust, timed
node scripts/ui-check.mjs                     # the simulator page in headless Chrome
node scripts/docs-check.mjs                   # the flowchart page in headless Chrome
node scripts/responsive-check.mjs             # both pages at phone, tablet and desktop sizes
node scripts/sqgathering-check.mjs            # SqGathering on both pages in headless Chrome
node scripts/embed-check.mjs                  # set up from a link, and embedded in another page
python3 scripts/serve.py                      # then open http://localhost:8000/ (original: /old/); no browser caching
```

The same checks run on GitHub's machines for every push and pull request (`.github/workflows/ci.yml`).

The rest of this README describes the original Python version.

## Installing required packages

`pip install -r requirements.txt`

## Running the Simulator

### Windows

`python run.py`

### MacOS / Linux

`python3 run.py`

The configuration takes the following variables.

- number of robots
  - Number of robots to generate random positions for
- initial positions (can be random) : list of points in 2D
  - Initial robot positions that are randomly generated unless specified by the user
- speed of robots : number
- type of scheduler (default: async)
- probability distribution (default: exponential)
- visibility radius : number
  - Visibility radius of the robots
- orientation of robots: same or random (input as a list of triple(translation, rotation, reflection)) (NOT IMPLEMENTED)
- multiplicity detection
  - Detects if multiple robots are in the same point
- colors (NOT IMPLEMENTED)
- obstructed visibility (NOT IMPLEMENTED)
- rigidity
  - Rigid movement allows the robot to reach it's destination when in the MOVE state.
  - Non-rigid movement means the robot will fall short of it's destination when in the MOVE state.

## Default Configuration

- Unlimited visibility
- No Multiplicity detection
- Ignore Collisions
- Transparent robots
- Rigid movement
