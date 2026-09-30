# Asynchronous Scheduler

A simulator for Look-Compute-Move robots in the Asynchronous System

## Rust + WebAssembly version (branch `rust-core`)

The simulator core has been ported to Rust and runs in the browser as WebAssembly, in a background worker, with **no Python or Pyodide**. The math is unchanged: the Rust core replays recorded runs of the Python simulator event by event. So far the core and the Gathering algorithm are ported; the other algorithms follow.

**Try it** (no install, runs entirely in your browser, CPU only):

- New simulator (Rust core, up to 10,000 robots): <https://imstillsamarth.github.io/LCMmodel/web/>
- Interactive flowcharts with the code of every step: <https://imstillsamarth.github.io/LCMmodel/web/docs.html>
- The original Pyodide simulator, for comparison: <https://imstillsamarth.github.io/LCMmodel/>

**Highlights:** 99–357× faster than the Python engine in the browser on identical runs (135–784× natively); 10,000 robots at 60 fps without a GPU; about 350 KB to download instead of about 16 MB.

**What's where:** [`report.md`](report.md) is the full report (method, results, every test). [`docs/core-schema.md`](docs/core-schema.md) is the core's contract. The code is in `crates/` (Rust) and `web/` (browser page).

**Verify it yourself** (Rust, Node ≥ 22, Chrome; Python with numpy for the Python comparisons):

```sh
rustup target add wasm32-unknown-unknown
cargo install wasm-bindgen-cli --version 0.2.129 --locked
cargo test --workspace                        # Rust core vs the Python, event by event
cargo build --release -p lcm-cli && ./scripts/build-wasm.sh
node scripts/check-wasm-parity.mjs            # browser build vs native, bit for bit
python3 scripts/compare-speed.py 20 50 100    # the same run in Python and Rust, timed
node scripts/ui-check.mjs                     # the simulator page in headless Chrome
node scripts/docs-check.mjs                   # the flowchart page in headless Chrome
python3 -m http.server 8000                   # then open http://localhost:8000/web/
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
