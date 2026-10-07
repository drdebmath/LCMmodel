# Asynchronous Scheduler

A simulator for Look-Compute-Move robots in the Asynchronous System

## Rust + WebAssembly version (branch `rust-core`)

The simulator core has been ported to Rust and runs in the browser as WebAssembly, in a background worker, with **no Python or Pyodide**. The math is unchanged. The Rust core runs its own simulations from the settings (it does not need Python or any recorded data); to prove it matches, its tests run the same settings and seeds as recorded Python runs and compare the two event by event. So far the core and two algorithms (Gathering and Smallest Enclosing Circle) are ported; the other algorithms follow.

**Try it** (no install, runs entirely in your browser, CPU only):

- New simulator (Rust core, up to 10,000 robots): <https://imstillsamarth.github.io/LCMmodel/>
- Interactive flowcharts with the code of every step: <https://imstillsamarth.github.io/LCMmodel/web/docs.html>
- The original Pyodide simulator, for comparison: <https://imstillsamarth.github.io/LCMmodel/old/>

**Highlights:** 99–357× faster than the Python engine in the browser on identical runs (135–784× natively); 10,000 robots at 60 fps without a GPU; about 350 KB to download instead of about 16 MB.

**Schedulers:** the core runs robots under one of two schedulers, chosen in Settings → Scheduler (or `scheduler` in the config, see [`docs/core-schema.md`](docs/core-schema.md)).

- **Async** (the default, as in the original): every robot wakes on its own random clock (rate λ), so several can be moving at once and a robot can see another halfway through a move.
- **Sequential**: one robot at a time does a whole Look-Compute-Move while all the others stay still, as in the sequential scheduler of [Universal pattern formation by oblivious robots under sequential schedulers](https://arxiv.org/abs/2412.10733) (Section 2.2). Choose the **turn order** (round-robin, or a fresh random order each epoch drawn from the seed) and the **pause between turns** (random with rate λ, or none). An epoch is over once every robot that is still active has had a turn; the page shows the epoch count. A run whose robots can never move again ends as *stalled* instead of looping forever.
- **Non-rigid movement:** turn *Rigid movement* off and set a minimum move **δ**. A robot that is stopped on the way still covers at least δ (or arrives, if its destination is within δ), and an adversary chooses where it stops: right after δ, at random, or after a fraction of the way. With δ left at 0 a non-rigid robot still arrives, as in the original. In the config an explicit `schedule` can also list the turns, and name the stopping point of each.

**What's where:** [`report.md`](report.md) is the full report (method, results, every test). [`docs/core-schema.md`](docs/core-schema.md) is the core's contract. The code is in `crates/` (Rust) and `web/` (browser page; `index.html` at the root opens it, the original page is in `old/`).

**Verify it yourself** (Rust, Node ≥ 22, Chrome; Python with numpy for the Python comparisons):

```sh
rustup target add wasm32-unknown-unknown
cargo install wasm-bindgen-cli --version 0.2.129 --locked
cargo test --workspace                        # Rust tests, including the Rust core vs recorded Python runs
cargo build --release -p lcm-cli && ./scripts/build-wasm.sh
node scripts/check-wasm-parity.mjs            # browser build vs native, bit for bit
python3 scripts/compare-speed.py 20 50 100    # the same run in Python and Rust, timed
node scripts/ui-check.mjs                     # the simulator page in headless Chrome
node scripts/docs-check.mjs                   # the flowchart page in headless Chrome
node scripts/responsive-check.mjs             # both pages at phone, tablet and desktop sizes
python3 -m http.server 8000                   # then open http://localhost:8000/ (original: /old/)
```

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
