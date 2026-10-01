# LCM Simulator: Rust + WebAssembly Port — Progress Report

**Dates:** 29 September – 1 October 2026
**Repository:** LCMmodel (`github.com/drdebmath/LCMmodel`), branch `rust-core`. Live preview: <https://imstillsamarth.github.io/LCMmodel/> (original page: `/old/`)
**Goal:** Port the Look-Compute-Move robot simulator from Python/Pyodide to Rust compiled to WebAssembly, so that around **10,000 robots can be simulated entirely in the browser on the CPU (no GPU)**, **without changing the original math**, as the base for a one-stop simulator for many algorithms.

---

## 1. Summary

| Item                                                                                     | Status                                                           |
| ---------------------------------------------------------------------------------------- | ---------------------------------------------------------------- |
| Analysis of the current simulator and of the CCMModel Rust port                          | Done                                                             |
| Core schema (the contract for the port)                                                  | Done: `docs/core-schema.md`                                      |
| Rust simulation core                                                                     | Done and verified against the Python, event by event             |
| Algorithms                                                                               | Gathering ported (1 of 7); the others are deliberately postponed |
| Foundation (Rust core running in the browser, in a background worker)                    | Done and verified bit for bit against native                     |
| Simulator page, first version (full controls, CPU canvas for 10,000 robots)              | Done (29 Sep); replaced by the redesign below                    |
| Simulator UI redesign (full-screen canvas, floating panels, inspector, stepping, export) | Done (30 Sep); 65 automated browser checks pass (1 Oct)          |
| Start settings: open world, pattern / arrangement / distribution, your own positions     | Done (1 Oct): §5.10                                              |
| Flowcharts: Markdown for GitHub + interactive page with the source of every step         | Done, redesigned (30 Sep); 19 automated browser checks pass      |
| Hosting: GitHub Pages, new simulator at the main address, original at `/old/`            | Done (1 Oct): §5.10                                              |
| Core speed work, analysis dashboard, remaining algorithms, removing Pyodide              | Not started (planned)                                            |

**Key results**

- The Rust core reproduces the Python simulator's runs **event by event** in 9 test scenarios (up to 200,000 events each); numbers agree to 15–16 significant digits.
- The browser build and the native build give **bit-identical** results.
- On identical runs, Rust is **135×–784× faster natively** and **99×–357× faster in the browser** than the Python engine (20–100 robots), and the gap grows with the number of robots.
- The new simulator page shows **10,000 robots at 60 fps using only the CPU**, drawing a frame in **3.7–6 ms** (2.4× faster than its first version).
- The page loads nothing from the internet: no CSS framework, fonts or icon fonts; the first swarm appears ≈130 ms after the page opens.
- The browser download shrinks from **≈16 MB (Pyodide)** to **≈400 KB** (274 KB WebAssembly + 18 KB bindings + 107 KB page code and styles).
- Four behaviours of the original were found and documented (§4), including a crash of SEC/GoToCenter at 1,000+ robots.

---

## 2. Analysis of the existing simulator

### 2.1 Structure

| File                                         |    Lines | Role                                                                    |
| -------------------------------------------- | -------: | ----------------------------------------------------------------------- |
| `robot.py`                                   |      945 | Robot state machine, geometry, 6 algorithms, faults, lights             |
| `scheduler.py`                               |      459 | Discrete-event scheduler (heap), fault assignment, termination, TwoTask |
| `run.py`                                     |      174 | JS↔Python bridge (JSON in/out), limits of 10,000 events / time 1,000    |
| `main.js`, `index.html`                      | 393, 215 | UI, Canvas2D drawing, Pyodide loader                                    |
| `pyodide/`                                   |    16 MB | Vendored Pyodide + numpy                                                |
| `static/`, `requirements.txt`, `config.json` |        – | Leftovers of an older Flask/SocketIO version (unused)                   |

Algorithms: Gathering (centre of gravity), SEC (smallest enclosing circle, Welzl), Go-To-Center (Ando et al. 1999), Uniform Circle Formation, Spreading (Lloyd, 32×32 grid), Pattern Formation (star), TwoTask (random mix of SEC and Gathering). Faults: crash, Byzantine, omission, delay, mixed.

### 2.2 Why the current version is slow

1. **One event per animation frame:** `simStep()` handles a single event per `requestAnimationFrame`, capping the browser at ~60 events/s. Combined with the 10,000-event limit in `run.py`, runs with more than a few robots never finish in the browser.
2. **O(n²) multiplicity check on every LOOK**, although no algorithm reads its result.
3. **A log string describing the whole swarm is built on every LOOK.**
4. **JSON serialisation on every step** across the JS↔Pyodide boundary.
5. Pyodide itself: ≈16 MB download and several seconds of start-up.

### 2.3 Measured Python cost (CPython 3.12.3, Intel i5-12450HX)

Time for **one** robot's LOOK + compute (snapshot included):

| Algorithm        | 100 robots | 1,000 robots | 10,000 robots |
| ---------------- | ---------: | -----------: | ------------: |
| Gathering        |    0.77 ms |        57 ms |         6.4 s |
| SEC              |     2.3 ms |        58 ms |         6.4 s |
| GoToCenter       |     2.3 ms |        58 ms |         6.4 s |
| CircleFormation  |    0.71 ms |        57 ms |         6.4 s |
| PatternFormation |     2.4 ms |       234 ms |        26.8 s |
| TwoTask          |     1.5 ms |        59 ms |         6.5 s |

At 10,000 robots, **99.8 % of the 6.4 s is the unused multiplicity check**; the algorithm itself takes ≈14 ms.

Full runs with 50 robots (Python engine, numpy generator):

| Algorithm        |  Events |   Time | Terminated |
| ---------------- | ------: | -----: | ---------- |
| Gathering        |  60,978 |  8.8 s | yes        |
| SEC              |  48,330 | 20.3 s | yes        |
| GoToCenter       |  37,648 | 23.8 s | yes        |
| CircleFormation  |  35,573 |  5.9 s | yes        |
| PatternFormation |  53,194 | 26.2 s | yes        |
| TwoTask          | 99,406+ | > 45 s | no         |

Each robot activates roughly 300–900 times before a run ends (measured: 335–905 at 50 robots). At 60 events/s, the 50-robot Gathering run would need ≈17 minutes in the browser, and it is stopped at 10,000 events.

---

## 3. Analysis of the CCMModel port (reference)

The same group ported **CCMModel** (mobile agents on port-labelled graphs) from Python/Pyodide to Rust:

|              | Before (`imstillsamarth/CCMModel`)     | After (`drdebmath/CCMModel`)                            |
| ------------ | -------------------------------------- | ------------------------------------------------------- |
| Size         | ≈4,000 lines, Python + networkx        | ≈30,000 lines, 13 Rust crates                           |
| Browser      | Pyodide + Cytoscape.js, JSON histories | Rust/WASM in a Web Worker, binary trace, Canvas2D       |
| Research use | Browser only                           | Native CLI with parallel seed sweeps, CSV               |
| Speed        | –                                      | 17–27× (Drop-and-Freeze), 1,100–5,200× (Help-by-Scouts) |

It was carried out from a written migration guide (`agent.md`) with rules such as *one implementation per algorithm*, *determinism*, and *preserve behaviour before changing semantics*, and used recorded Python runs as test fixtures.

**Adopted here:** schema-first approach, dependency-free core, recorded Python fixtures, a native CLI, typed arrays instead of JSON, Web Worker execution.

**Improved on:**

- a plug-in interface for algorithms (CCMModel needs edits in ~5 places per algorithm);
- the Python reference fixtures are kept permanently (CCMModel's comparison harness was deleted, so its benchmark can no longer be reproduced);
- live playback, not only run-then-replay.

**Differences that matter for LCM:**

- CCM is all integers. LCM uses floating-point geometry, so float behaviour (rounding, `sum()`, `%`) must be matched explicitly.
- LCM's target is 10× larger (10,000 robots rather than < 1,000).

---

## 4. Behaviours of the original found during the work

These were **documented, not changed**; each will be a deliberate decision later.

| #   | Finding                                                                                                                                                                                                         | Evidence                                                               |
| --- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------- |
| 1   | **SEC and GoToCenter crash at ≥ 1,000 robots.** The Welzl routine recurses n deep, hits Python's recursion limit, and the error is caught, so every robot freezes and the run "terminates" having done nothing. | 990 robots: works; 1,000 and 2,000: `maximum recursion depth exceeded` |
| 2   | **TwoTask never terminates.**                                                                                                                                                                                   | Stopped at 200,000 events (sim time ≈3,500) without ending             |
| 3   | **Non-rigid movement behaves exactly like rigid.** The WAIT event is scheduled at the full-distance arrival time, so robots always reach their target.                                                          | Forcing rigid behaviour for non-rigid robots changed no test result    |
| 4   | **Gathering's centre includes crashed robots, but its termination test ignores them.**                                                                                                                          | `robot.py` `_midpoint` / `_midpoint_terminal`                          |
| –   | The old canvas sets its size using the device pixel ratio but draws without scaling, so drawings are mis-scaled on high-DPI screens.                                                                            | `main.js` `resizeCanvas`                                               |

---

## 5. What was built

### 5.1 Layout

```text
Cargo.toml                  workspace
docs/core-schema.md         the contract: config, state, randomness, event order, algorithm interface, frame format
crates/lcm-core/            the simulation engine (no browser, file or GPU dependencies)
crates/lcm-algorithms/      algorithms + registry (Gathering only for now) + parity tests
crates/lcm-cli/             native runner and benchmark (`lcm run`, `lcm bench`)
crates/lcm-wasm/            browser bridge (WebAssembly bindings)
index.html                  the simulator page (served at the site's main address); its files are in web/
old/index.html              the original Pyodide page, served at /old/
web/                        simulator: styles.css, app.js, settings.js, start.js, client.js, worker.js,
                            renderer.js, inspector.js, icons.js, pkg/ (built core + build.json)
                            · flowcharts: docs.html (generated), docs.css, docs.js · index.html: redirect to /
reference/                  Python harness: portable random generator + fixture generator
fixtures/                   9 recorded Python runs + 5,000 CPython round() results
scripts/                    build-wasm.sh, check-wasm-parity.mjs, compare-speed.py, gen-flowcharts.py,
                            ui-check.mjs, docs-check.mjs, browser-smoke.py, lib/browser.mjs (headless-Chrome driver)
```

About 6,500 lines of new code, tests and scripts. The original Python files (`robot.py`, `scheduler.py`, `run.py`) are **unchanged**. The original page moved from `index.html` to `old/index.html` with one added line (`<base href="../">`, so it still loads its files from the root), and one line of `main.js` changed so the bundled Pyodide fallback is found from `/old/` too (§5.10).

Toolchain: Rust 1.98.1 (+ `wasm32-unknown-unknown`), wasm-bindgen 0.2.129, Node 24.14.1, Python 3.12.3, Google Chrome (headless tests).

### 5.2 Schema (`docs/core-schema.md`)

It defines:

1. **Reference semantics:** which Python float behaviours are reproduced exactly (Neumaier-compensated `sum()` of Python ≥ 3.12, sign rule of `%`, `round()` ties-to-even on the exact decimal value, `10 ** -p`, first-wins `min`/`max`, stable sorts).
2. **Configuration:** the same field names as today's `main.js` `params()`, validated with explicit errors.
3. **State:** stored as arrays per property (structure of arrays).
4. **Randomness:** the exact draw order of the Python scheduler.
5. **Events:** ordering identical to Python's heap of `(time, id, state)` tuples.
6. **Algorithm interface.**
7. **Frame format** for the browser.
8. **Deliberate differences from the Python.**

### 5.3 Rust core (`crates/lcm-core`)

- **Python-exact float helpers** (`pyfloat.rs`): `py_sum`, `py_mod`, `py_round`, `pow10_neg`.
- **Random generator** (`rng.rs`): SplitMix64 with `random`, `uniform`, `exponential`, `integers`, `shuffle` and `choice`. The same generator is implemented in `reference/portable_rng.py`.
- **Event loop** (`sim.rs`): a line-by-line port of `Scheduler.handle_event` and `Robot.look/move/wait`. It covers the Byzantine override, the "only I am visible" termination, omission, the freeze threshold, sub-tick moves, the light cooldown and global termination.
- **Robot state as arrays** (`robots.rs`), and positions interpolated at the exact look time.
- **Event queue** (`event.rs`), ordered like Python's heap.
- **Frames** (`frame.rs`): positions, flags, lights, faults, tasks, targets and circles as typed arrays, plus an O(n log n) multiplicity count for display (same groups as Python's O(n²) scan).
- **Algorithm interface** (`algorithm.rs`): `compute(look, rng, scratch) -> Decision { target, terminate, overlay }`. Adding an algorithm means one file plus one registry line.
- `advance(budget)`: runs many events per call, stopping at an event budget, a simulated time, a limit, or the end.

### 5.4 Algorithms (`crates/lcm-algorithms`)

- **Gathering** is ported and serves as the driver for testing the core.
- SEC, GoToCenter, CircleFormation, PatternFormation, Spreading and TwoTask were written in a first pass, then **set aside** so that the core could be verified alone first. They are not part of this branch.

### 5.5 Foundation: the core in the browser

- `crates/lcm-wasm`: a thin bridge. It creates a simulation from JSON config, advances it, and returns frames as typed arrays.
- `web/worker.js`: owns the simulation in a Web Worker. It simulates continuously in ≈6 ms slices, and hands frames to the page as transferable arrays (no JSON, no copying).
- `scripts/build-wasm.sh`: builds `web/pkg/` with WebAssembly SIMD enabled, and strips build paths from the binary.

### 5.6 Simulator page, first version (`web/`, 29 September)

This first version was replaced by the redesign in §5.8; it is kept here as a record.


- **Controls:** all of today's controls (algorithm, speed, visibility with Inf, faulty count and fault type, rigid movement, lights, world W/H, λ, samples, precision, seed).
- **Robots:** 1–10,000, on a log-scale slider with an exact number box.
- **Playback speed:** 1×, 5×, 20×, 100× or Max simulated time per second. The worker paces itself with the core's `until_time`, and drops any backlog if the machine can't keep up.
- **Run control:** Start, Pause/Resume, Stop.
- **Renderer** (`renderer.js`): CPU Canvas2D. Robots are grouped by colour (≈10 fill calls per frame instead of one per robot), with the same colours and legend as today. It draws target lines, visibility circles, light rings and algorithm circles. Above 2,000 robots, outlines, target lines and circles are hidden to keep drawing fast.
- **View:** zoom with buttons or the mouse wheel around the cursor, drag to pan, reset view, correct high-DPI scaling.
- **Live stats:** simulated time, events, events/s, terminated, draw time and fps, worker load.
- **Theme and layout:** dark mode (shared setting with the old page) and a mobile panel toggle.

### 5.7 Flowcharts (`scripts/gen-flowcharts.py`)

As in CCMModel, the flowcharts are generated from one set of chart definitions into two outputs, with the code of each function copied straight from the source files, so a chart can never show code that differs from what runs:

- `docs/flowcharts.md`: Mermaid charts that GitHub draws, with swimlanes and colours by step type, a table explaining every step, and the source of every function a chart names, with file and line links.
- `web/docs.html`: the interactive page, redesigned on 30 September (§5.9). It is linked from the simulator's toolbar ("How it works").

The four charts:

1. **The event loop:** `advance` / `step`, i.e. `Scheduler.handle_event`. Lanes: run loop, take the next event, handle it, after each robot event.
2. **One Look–Compute–Move cycle:** faults, freezing, termination, the move and the WAIT. Lanes: **LOOK / COMPUTE / MOVE**.
3. **Gathering.** Lanes: target, termination.
4. **In the browser.** Lanes: **Page / Worker / Rust core**.

Each remaining algorithm gets its own chart when it is ported.

Safety checks:

- `--check` fails if either output no longer matches the code.
- A chart that names a function which does not exist stops generation with an error.

### 5.8 Simulator UI redesign (`web/`, 30 September)

Rebuilt from scratch after choosing, with the project owner: a full-screen canvas with floating panels; a clean research-tool style that follows the system's light or dark setting; plain JavaScript with no build step; and three new features (robot inspector, step-by-step controls, export + keyboard shortcuts).

**Design**

- Own design system (`styles.css`): colour tokens for light and dark, panels, buttons, switches. No CSS framework, web fonts or icon fonts; the icons are inline SVG (`icons.js`).
- Floating panels over a full-screen canvas:
  - **toolbar** (top left): algorithm, Play/Pause, step one event, step one time unit, Reset, playback speed, Settings, "How it works";
  - **settings** (left): grouped and collapsible (Swarm, Movement, Visibility, Faults, World, Scheduler), each setting with a hint;
  - **status** (top right): time, a progress bar of terminated robots, events, events per second, drawing performance, the last stepped event;
  - **legend** (bottom): only the colours on screen, with counts;
  - **view tools** (bottom right): zoom, fit the world in view, real browser full screen, lights, export, theme, shortcuts.
- The world is fitted into the space the panels leave free.

**Behaviour**

- A new run appears as soon as the page opens. Changing a setting before pressing Play rebuilds the swarm at once; after the run has started, the panel says "Reset to apply" instead of throwing the run away.
- **Robot inspector:** hovering a robot shows its id and state; clicking it opens a card with algorithm, phase, position, target, distance to target, speed, distance travelled, fault and light, refreshed while the run plays. The selected robot is highlighted with its target, visibility circle and computed circle at any swarm size.
- **Stepping:** one event at a time (the status panel shows what happened, e.g. "t 1.092 · robot 493 · Look → moved") or one unit of simulated time.
- **Export:** PNG of the view with a caption (algorithm, robots, time, seed); CSV of every robot in full precision, produced by the Rust core.
- **14 keyboard shortcuts** (Space, →, Shift+→, R, F, Shift+F, +/−, L, S, H, E, Shift+E, T, Esc, ?) and a clean view that hides the panels.

**Efficiency**

- Drawing rewritten after measuring the options (§7.4): `fillRect` per robot for small dots, pre-drawn sprites per colour for larger robots and light rings, at whole-pixel positions.
- At most one redraw per screen frame, however many frames, zooms or hovers arrive.
- Frame arrays are recycled between the page and the worker (`fill_frame`), so steady playback allocates nothing per frame.
- Status text is updated a few times a second rather than every frame.
- Nothing is fetched or drawn while paused or after the run ends.

**Bridge additions** (`crates/lcm-wasm`): `step_one` (one event and what it did), `robot_info` (everything about one robot), `export_csv`, and `fill_frame` (write a frame into caller-owned arrays).

### 5.9 Flowchart page redesign (`web/docs.html`, 30 September)

- **Same design system as the simulator,** with a shared theme setting; nothing loaded from the internet.
- **Step cards:** a coloured bar and icon by type (start, step, question, finish, "ends this pass"), the function name as a code-font subtitle, and a `</>` badge on steps that open code.
- **Swimlanes** showing the structure, rounded orthogonal arrows, green "yes" branches, dashed "no" branches, and label chips.
- **Every step is clickable (48 in total)** and has its own plain-language explanation. The 20 steps that stand for a function also show its source with syntax highlighting, real line numbers, Copy and a GitHub link. "from / next" buttons jump to connected steps.
- **Guided tour** through each chart (← / →). Hovering a step fades everything except its connections.
- **Readable by default:** charts open fitted to the width (the event loop at 92 %, previously 58 %); the mouse wheel scrolls, Ctrl + wheel zooms; the detail panel slides in only when a step is selected.
- **Links to a single step** (e.g. `docs.html#look-compute-move/view`); number keys switch charts.
- **"Open the simulator":** a prominent card at the top of the sidebar, and a button in the top bar.

### 5.10 Start settings, cache fix and new addresses (1 October)

**Where robots start.** Before, robots always started at random inside the 600 × 600 world box, which made the start look like a closed square (and a square start gathers in an X pattern). The Start group of the settings now has:

- **Open world** (on by default): no world box; robots start wherever the pattern puts them. Off: the original world box and its width/height. In both cases robots are never kept inside the box, exactly as in the original.
- Three settings, each answering one question:

| Setting      | Question                   | Choices                                                                                                                                                       |
| ------------ | -------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Pattern      | What shape?                | Cloud (no shape), Square, Rectangle, Circle, Polygon, Line, Groups (clusters), My own positions; with Open world off also "Original: random in the world box" |
| Arrangement  | Inside it, or on its edge? | Filled, Edge only, Ring (circles); shown only when the pattern has more than one                                                                              |
| Distribution | How are robots spaced?     | Random, Evenly spaced, Evenly spaced a bit messy, Bunched in the middle (Gaussian); only those that fit                                                       |

- Only the main slider for the shape is shown (named for it: Side length, Width, Diameter, Length); rotation, hole size, aspect, messiness and similar are under "More options". A sentence under the settings says what the choice gives, e.g. *"500 robots evenly spaced on the edge of a circle 400 wide."* Help texts appear on hovering a label.
- **My own positions:** an editor window for typed or pasted coordinates: one `x, y` per line (comma, space, tab or `;`), `#` comments, a header row (so the simulator's own CSV export pastes back), or JSON `[[x, y], …]`. Errors are reported by line and never start a broken run; the robot count follows the number of positions.

The generator is in `crates/lcm-core/src/start.rs` (36 combinations; documented in `docs/core-schema.md` §4.3). It has its own random generator seeded from the run's seed, so a start is reproducible and independent of the activation draws, and "evenly spaced" uses no randomness at all. These starts are new: the original has no equivalent, and the original start (Open world off, Original pattern) still matches the Python exactly. Even spacing uses a grid (squares, rectangles), a sunflower pattern (circles, rings), equal arc lengths (edges, lines) and a Fibonacci lattice over the polygon's centre triangles (filled polygons).

The page went through two simplifications after review: the first version (two drop-downs, a row of presets, every parameter visible) was too crowded; the second (fewer controls) still used unclear words. The three-question layout above replaced both.

**Fix: an old cached core with a new page.** After an update, a browser could keep the previous WebAssembly file and run it with the new page code, failing with *"unknown field `open_world`"*. The build now writes an id (a hash of the core files) to `web/pkg/build.json`; the page loads the worker and the core with that id in their address, so a new build is always fetched in full. The page also explains the error if it ever happens.

**Fix:** the hover tooltip of a robot stayed on screen after its run was replaced.

**Addresses.** The new simulator is now the main address, <https://imstillsamarth.github.io/LCMmodel/>; the original is at `/old/`; old links to `/web/` redirect to the main address.

**Wording.** The README said the Rust core "replays recorded runs of the Python simulator". It does not: it runs its own simulations from the settings. The recorded Python runs are only used by the tests to compare against. The README and the test's name (`rust_core_matches_python_fixtures`) now say so.

---

## 6. Verification

### 6.1 Method

The Python simulator is the reference ("answer key"), because the goal is identical behaviour:

1. `reference/gen_fixtures.py` runs the **unmodified** Python modules. The only substitution is numpy's generator, replaced by the portable one. Each fixture records every event (time, robot, kind, outcome, the robot's position or target, flags) for the first 1,500 events, plus the final state of every robot.
2. The Rust test (`crates/lcm-algorithms/tests/parity.rs`) runs the Rust core with each fixture's settings and seed and requires:
   - identical events for the first 1,500;
   - identical end state for every robot after up to 200,000 events: position, target, state, frozen, terminated, light and distance travelled;
   - identical event count and end time.

   Tolerance is 1e-9 relative.
3. `scripts/check-wasm-parity.mjs` runs every fixture in the browser build and the native build and requires the results to be identical down to the last bit.
4. `scripts/browser-smoke.py` runs the worker and the full page in headless Chrome with `--disable-gpu`.

Other checks are independent of the Python:

- the random generator against published SplitMix64 reference values;
- `py_round` against 5,000 CPython `round()` results;
- the event order against Python's tuple-ordering rules.

### 6.2 Scenarios covered (9 fixtures, all driven by Gathering)

Normal gathering (8 robots), limited visibility (20 robots, radius 150), non-rigid movement, explicit start positions, and crash, Byzantine, omission, delay and mixed faults.

### 6.3 Results

| Check                                     | Result        |
| ----------------------------------------- | ------------- |
| Rust unit tests (7)                       | pass          |
| `round()` vs 5,000 CPython results        | all identical |
| Replay of the 9 Python fixtures           | all pass      |
| Browser build vs native build, 9 fixtures | bit-identical |
| Lint (`cargo clippy`, all targets)        | 0 warnings    |
| Builds for WebAssembly                    | yes           |

How exact the match with Python is:

| Fixture              |  Events | Events bit-identical (first 1,500) | Worst relative difference | Worst final position difference |
| -------------------- | ------: | ---------------------------------: | ------------------------: | ------------------------------: |
| explicit_positions   |     159 |                             87.4 % |                   2.1e-16 |                               0 |
| fault_byzantine      | 200,000 |                             85.7 % |                   1.4e-15 |                         6.5e-14 |
| fault_crash          |   9,029 |                             86.5 % |                   2.0e-16 |                         1.5e-16 |
| fault_delay          |  26,534 |                             86.3 % |                   2.0e-16 |                         2.3e-15 |
| fault_mixed          | 200,000 |                             86.4 % |                   2.0e-16 |                         4.0e-15 |
| fault_omission       |  15,522 |                             83.2 % |                   2.0e-16 |                         1.8e-16 |
| gathering_20_vis150  |   6,803 |                             66.9 % |                   2.1e-15 |                         8.1e-16 |
| gathering_8          |   5,877 |                             68.3 % |                   9.3e-16 |                         1.6e-16 |
| gathering_8_nonrigid |   6,666 |                             81.3 % |                   1.4e-15 |                         1.1e-15 |

Every event (order, robot, kind, outcome) is the same. The remaining differences are in the last one or two digits. They come from different maths libraries (glibc for Python, the Rust `libm` crate for Rust) rounding `log1p`, `hypot`, `sin` and `cos` differently. They do not grow over 200,000 events.

### 6.4 Checking that the tests can fail (mutation tests)

| Deliberate change to the core                   | Outcome                                                                                                                                               |
| ----------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------- |
| Omission probability 0.5 → 0.4                  | Caught immediately (`fault_omission`, event #2; `fault_mixed`, final state)                                                                           |
| Light cooldown 0.5 → 0.4                        | **Not caught at first:** the test did not compare lights. The test was extended to compare the full robot state, and now catches it in all 9 fixtures |
| Non-rigid robots snap to target like rigid ones | Not caught, correctly: the change is equivalent in the original (finding #3)                                                                          |

### 6.5 Known gaps in verification

- Only Gathering is verified; the other six algorithms are not yet ported.
- Fixtures are small (4–20 robots, 9 configurations); large swarms are not compared directly with Python (too slow in Python).
- The browser-vs-native check compares totals (event count, end time, terminated count), not every robot's position.
- Drawing is checked visually from screenshots, not automatically.

**Proposed additions:** a sweep of ≈200 random small configurations through both Python and Rust; a 200-robot fixture; a per-robot bit comparison for the browser build; behaviour checks independent of Python (e.g. Gathering ends with all robots within ε of one point).

---

## 7. Performance

### 7.1 Identical runs: Python vs Rust (`scripts/compare-speed.py`)

Same configuration and same random draws in all three, so the same events and the same result ("same run" is checked). Only the event loop is timed.

| Robots |  Events |  Python | Rust native | Rust in browser (WASM) | Native speed-up | Browser speed-up |
| -----: | ------: | ------: | ----------: | ---------------------: | --------------: | ---------------: |
|     20 |  28,779 |  1.20 s |    0.0089 s |               0.0121 s |            135× |              99× |
|     50 |  62,730 |  8.08 s |    0.0171 s |               0.0470 s |            473× |             172× |
|    100 | 163,983 | 61.94 s |    0.0790 s |               0.1735 s |            784× |             357× |

The Python column is desktop CPython. The existing web page runs Python through Pyodide, handles one event per frame and stops at 10,000 events, so the practical gain in the browser is larger than shown.

### 7.2 Larger swarms (native, `lcm bench`)

| Case                             | Result                                                                                                        |
| -------------------------------- | ------------------------------------------------------------------------------------------------------------- |
| 1,000 robots, full Gathering run | 1,889,642 events in 7.3 s (the Python would need roughly 14 hours, estimated from its measured cost per LOOK) |
| 10,000 robots                    | ≈17,500–18,650 events/s                                                                                       |
| 10,000 robots, visibility 50     | ≈12,240 events/s (slower: every robot's distance is still checked; to be fixed with a spatial grid)           |

A full run to the end at 10,000 robots is **estimated at 15–20 minutes natively**. This is extrapolated from the measured rate, not timed. The cost comes from the model itself: each LOOK reads all 10,000 positions, and each robot looks ≈900 times, which is ≈9×10¹⁰ position reads per run.

### 7.3 In the browser

| Measurement                                                               | Result                                              |
| ------------------------------------------------------------------------- | --------------------------------------------------- |
| Raw WebAssembly, 10,000 robots (Node)                                     | 5,546 events/s (≈3.2× slower than native)           |
| Profile of the WebAssembly at 10,000 robots                               | `hypot` 52.5 %, `position_at` 24.3 %, `look` 20.1 % |
| WebAssembly SIMD enabled                                                  | No measurable change (kept on)                      |
| Worker duty-cycle fix (simulate continuously instead of 8 ms per request) | 1,497 → 5,008 events/s; worker busy 30 % → 98 %     |

Cause of the WebAssembly gap: most robots are moving most of the time, and every LOOK recomputes the full path length of every moving robot with `hypot`, a slow software routine in WebAssembly. Computing each length once at the start of the move gives bit-identical results. This is the first item of the core speed stage.

### 7.4 Simulator page in headless Chrome, GPU disabled (CPU only)

First version (29 September):

|                Robots | Events/s | FPS | Draw time incl. painting | Longest main-thread pause while simulating |
| --------------------: | -------: | --: | -----------------------: | -----------------------------------------: |
|     50 (20× playback) |    paced |  60 |                  0.39 ms |                                      20 ms |
|  2,000 (20× playback) |   30,450 |  62 |                 10.07 ms |                                      37 ms |
| 10,000 (20× playback) |    5,871 |  62 |                  7.43 ms |                                      44 ms |

- The frame budget at 60 fps is 16.7 ms.
- The first page load pauses ≈1.5 s, caused by the Tailwind CDN compiling styles (same as the old page). It does not happen during simulation.
- Other page checks:
  - **Pacing:** 5 s at 20× reached simulated time 99.8.
  - **Pause/Resume:** time frozen while paused; after 0.8 s of resume at 20×, +15.8 time units.
  - **Run to the end:** 30 robots at Max finished in 0.10 s (48,566 events, "every robot terminated").
  - **Page vs native, same configuration at t ≈ 99.8:** 2,247 vs 2,248 events.
  - **Screenshots:** confirmed the colours of all fault types, visibility circles, light rings, target lines and cluster formation under limited visibility.

Redesign (30 September), measured by `scripts/ui-check.mjs` at 1440 × 900:

| At 10,000 robots                             | First version                           | Redesign                                 |
| -------------------------------------------- | --------------------------------------- | ---------------------------------------- |
| Drawing one frame, including painting        | 14.3 ms (7.4 ms in a smaller window)    | 3.7–6.0 ms                               |
| Frames per second                            | 60                                      | 60                                       |
| Longest main-thread pause while running      | 44 ms                                   | 17 ms                                    |
| Simulation speed at Max                      | ≈5,000 events/s                         | ≈5,200–5,400 events/s (worker 97 % busy) |
| First swarm on screen after opening the page | ≈1.5 s (styles compiled in the browser) | ≈130 ms                                  |

How 10,000 robots are drawn was chosen by measuring the options in headless Chrome with the GPU off (ms per frame, including painting):

| Method                                                                 | Normal screen | High-DPI (2×) |
| ---------------------------------------------------------------------- | ------------- | ------------- |
| One path of rectangles (first version)                                 | 14.9          | 22.1          |
| **`fillRect` per robot (chosen for small dots)**                       | **5.9**       | **8.6**       |
| One path of circles                                                    | 32.0          | 41.8          |
| Writing pixels directly (ImageData)                                    | 2.8           | 9.7           |
| 2,000 outlined circles, one path                                       | 9.4           | 48.0          |
| **2,000 outlined circles, stamped sprites (chosen for larger robots)** | **9.6**       | **14.7**      |

The simulation speed was also checked on its own: the core alone runs at 5,632–5,803 events/s at 10,000 robots throughout a run, and the full page at 5,383 events/s with drawing and 5,098 without, so the page costs the simulation almost nothing.

### 7.5 Size

|                                                          | Download |
| -------------------------------------------------------- | -------- |
| Current page (Pyodide + numpy + Python files)            | ≈16 MB   |
| New page (WebAssembly + bindings + page code and styles) | ≈350 KB  |

---

## 8. Structural changes compared with the original

| Original                                                                   | New                                                                                         |
| -------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------- |
| One `Robot` class holding state, geometry, all algorithms and logging      | Separate engine, algorithms (one file each), browser bridge, page                           |
| Algorithm lookup table inside `Robot`                                      | Plug-in interface + registry                                                                |
| Global state in `run.py` (one simulation at a time)                        | `Simulation` objects (any number)                                                           |
| Random generator shared as a class attribute                               | Owned by each simulation                                                                    |
| One Python object per robot; a new snapshot dictionary per LOOK            | Arrays per property; one reused snapshot buffer                                             |
| O(n²) multiplicity check and full-swarm log string on every LOOK           | Removed from the hot path (same results)                                                    |
| Pyodide on the main thread, JSON per step, one event per frame             | WebAssembly in a Web Worker, typed arrays, thousands of events per frame                    |
| One draw call per robot                                                    | Batched by colour, level of detail for large swarms                                         |
| Tailwind, fonts and icons loaded from CDNs; styles compiled in the browser | Own stylesheet and inline SVG icons; nothing loaded from the internet                       |
| A long scrolling control panel; stats at the bottom                        | Floating toolbar, grouped settings, status, legend and view tools over a full-screen canvas |
| New arrays and a full stats rebuild every frame                            | Recycled frame arrays; status text a few times a second; nothing drawn when idle            |

**Math changes:** none. Planned exception: the recursive Welzl (finding #1) will be replaced by an iterative version with the same result and the same random draws when SEC is ported.

---

## 9. How to reproduce

```sh
cd LCMmodel && git checkout rust-core
export PATH=$HOME/.cargo/bin:$PATH

# correctness
cargo test --workspace                          # Rust core vs Python, event by event
cargo build --release -p lcm-cli && ./scripts/build-wasm.sh
node scripts/check-wasm-parity.mjs              # browser build vs native, bit for bit
python3 reference/gen_fixtures.py               # regenerate fixtures from the unmodified Python (needs numpy)

# speed
python3 scripts/compare-speed.py 20 50 100      # identical runs: Python vs Rust native vs browser (needs numpy)
./target/release/lcm bench --robots 10000 --seconds 10

# browser (headless Chrome, GPU disabled)
python3 scripts/browser-smoke.py 10000 5                                # worker only
node scripts/ui-check.mjs                       # simulator page: 65 checks + screenshots
node scripts/docs-check.mjs                     # flowchart page and addresses: 19 checks + screenshots

# flowcharts
python3 scripts/gen-flowcharts.py               # regenerate docs/flowcharts.md and web/docs.html
python3 scripts/gen-flowcharts.py --check       # fail if they are out of date with the code

# side by side, by hand
python3 -m http.server 8000
#   new page: http://localhost:8000/        old page: http://localhost:8000/old/
#   flowcharts: http://localhost:8000/web/docs.html
```

---

## 10. Remaining work

| #   | Stage                | Content                                                                                                                                                                                                                                                                                       |
| --- | -------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 3   | Core speed           | Cache each move's path length (the 52 % `hypot` cost); running totals for centroid-based algorithms; spatial grid for limited visibility; counters instead of scans. Each change is checked against the exact mode. Expected: a full 10,000-robot Gathering run in seconds instead of minutes |
| 4   | Analysis dashboard   | Sweeps over robot counts and seeds in parallel Web Workers, charts, table, CSV export                                                                                                                                                                                                         |
| 5   | Remaining algorithms | SEC (with the iterative Welzl), GoToCenter, CircleFormation, PatternFormation, Spreading, TwoTask, each with Python fixtures                                                                                                                                                                  |
| 6   | Switch-over          | Hosting done (new page at the main address, original at `/old/`). Left: remove Pyodide once every algorithm is ported, CI                                                                                                                                                                     |

**Limits to keep in mind:**

- SEC and GoToCenter with unlimited visibility remain O(n) per LOOK at any speed, so full runs at 10,000 robots will take minutes. With limited visibility they become fast.
- The number of activations per robot is set by the experiment's parameters (speed, λ, precision), not by the implementation.

**Open decisions:**

1. What to do about findings #1–#4 (fix or keep the original behaviour).
2. Whether to add the proposed extra verification before stage 3 (recommended).
3. When to remove the original page and Pyodide (after all algorithms are ported).
4. Browsers still to test by hand: Firefox, Safari, mobile layout.
5. The flowchart page's GitHub buttons point to the `rust-core` branch of `imstillsamarth/LCMmodel`; switch `REPO_URL` in `scripts/gen-flowcharts.py` if the code moves.

---

## Appendix A: Complete test log

Every test run on 29–30 September 2026, with its result. "Repeatable" tests are in the repository and can be rerun with the commands in §9.

### A.1 Automated Rust tests (`cargo test --workspace`)

|   # | Test                                        | What it checks                                                                                    | Result               |
| --: | ------------------------------------------- | ------------------------------------------------------------------------------------------------- | -------------------- |
|   1 | `rng::matches_reference_splitmix64`         | Random generator output equals the published SplitMix64 reference values                          | Pass                 |
|   2 | `rng::index_and_choice_stay_in_range`       | Random integers stay in range; `choice` returns distinct values                                   | Pass                 |
|   3 | `pyfloat::sum_is_compensated_like_cpython`  | `py_sum` rounds like Python 3.12's `sum()` (e.g. `sum([0.1]*10) == 1.0`)                          | Pass                 |
|   4 | `pyfloat::modulo_follows_divisor_sign`      | `py_mod` follows Python's sign rule for `%`                                                       | Pass                 |
|   5 | `pyfloat::round_matches_cpython`            | Hand-picked `round()` cases, including ties and 2.675                                             | Pass                 |
|   6 | `event::pops_in_python_tuple_order`         | Events leave the queue in Python's `(time, id, state)` order                                      | Pass                 |
|   7 | `geom::interpolation_is_clamped`            | Interpolation and distance                                                                        | Pass                 |
|   8 | `py_round::matches_cpython_round`           | `py_round` against 5,000 CPython 3.12 `round()` results, bit for bit                              | Pass (5,000 / 5,000) |
|   9 | `parity::rust_core_matches_python_fixtures` | The 9 recorded Python runs: first 1,500 events one by one, then the full end state of every robot | Pass (9 / 9)         |
|  10 | `parity::every_registered_algorithm_builds` | Every registered algorithm builds and runs 500 events                                             | Pass                 |

Also: `cargo clippy --workspace --all-targets` gave 0 warnings, and `cargo check --target wasm32-unknown-unknown` succeeded.

### A.2 Automated cross-build and browser tests

|   # | Test                                                                    | What it checks                                                                                     | Result                                                                      |
| --: | ----------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------- |
|  11 | `node scripts/check-wasm-parity.mjs`                                    | Browser build vs native build vs Python, 9 fixtures                                                | Pass: event count and end time bit-identical, and both match Python         |
|  12 | `browser-smoke.py 10000 5` (worker, headless Chrome, GPU off)           | The worker runs 10,000 robots; frames have correct sizes and finite values; the page never freezes | Pass: no errors, 5,008 events/s, worker busy 98 %, longest page pause 10 ms |
|  13 | `browser-smoke.py 50 60` (worker)                                       | A run that finishes is reported as ended                                                           | Pass: 62,730 events in 0.049 s, 50 / 50 terminated, `stop = ended`          |
|  14 | `page-check.html`, 50 robots, 20×                                       | Full page: stats, drawing, pacing                                                                  | Pass: 60 fps, draw 0.39 ms, simulated time 99.8 after 5 s                   |
|  15 | `page-check.html`, 2,000 robots, 20×                                    | Heaviest drawing case (outlines and target lines on)                                               | Pass: 62 fps, draw 10.07 ms, 30,450 events/s                                |
|  16 | `page-check.html`, 10,000 robots, 20×                                   | Target size                                                                                        | Pass: 62 fps, draw 7.43 ms, 5,871 events/s, longest pause 44 ms             |
|  17 | `page-check.html`, 60 robots, 8 mixed faults, visibility 120, lights on | Colours of every fault type, visibility circles, light rings                                       | Pass (checked on the screenshot)                                            |
|  18 | `page-check.html?pause=1`, 200 robots                                   | Pause freezes time, Resume continues                                                               | Pass: time 19.9 → 19.9 while paused → 35.7 after 0.8 s at 20×               |
|  19 | `page-check.html`, 30 robots, Max speed                                 | Run to the end on the full page                                                                    | Pass: 48,566 events in 0.10 s, message "every robot terminated"             |
|  20 | `scripts/compare-speed.py 20 50 100`                                    | Identical runs in Python, native Rust and browser Rust; same events and end time                   | Pass (same run: yes ×3); timings in §7.1                                    |

Tests 14–19 used the first version of the page. That page and its test (`web/tests/page-check.html`) were replaced on 30 September by the redesign, whose checks are in A.5.

### A.3 One-off investigations and measurements

|   # | Investigation                                                         | Result                                                                     |
| --: | --------------------------------------------------------------------- | -------------------------------------------------------------------------- |
|  21 | Python cost per LOOK at 100 / 1,000 / 10,000 robots, all 6 algorithms | Table §2.3; multiplicity check is 99.8 % of the time at 10,000             |
|  22 | Python full runs, 50 robots, all 6 algorithms                         | Table §2.3; TwoTask did not terminate                                      |
|  23 | Activations per robot (Python)                                        | 335–905 LOOKs per robot at 50 robots                                       |
|  24 | SEC at 500 / 990 / 1,000 / 2,000 robots (Python)                      | Works up to 990; `RecursionError` at 1,000 and 2,000 (finding #1)          |
|  25 | Exactness of the Rust core vs Python per fixture                      | Table §6.3: all events equal, worst difference 6.5e-14                     |
|  26 | Mutation tests (3 deliberate changes to the core)                     | Table §6.4                                                                 |
|  27 | `lcm bench`: native speed at 1,000 and 10,000 robots                  | Table §7.2                                                                 |
|  28 | Raw WebAssembly speed in Node (1,000 / 10,000 robots)                 | 76,364 and 5,546 events/s; frame capture 0.06 / 0.44 ms                    |
|  29 | CPU profile of the WebAssembly at 10,000 robots                       | `hypot` 52.5 %, `position_at` 24.3 %, `look` 20.1 %                        |
|  30 | WebAssembly SIMD on vs off                                            | No measurable difference; results still bit-identical                      |
|  31 | Page vs native at the same configuration (50 robots, t ≈ 99.8)        | 2,247 vs 2,248 events: same run                                            |
|  32 | Drawing 10,000 robots five ways, normal and high-DPI                  | Table §7.4; `fillRect` and sprites chosen                                  |
|  33 | Core alone at three points of a 10,000-robot run                      | 5,632 / 5,632 / 5,803 events/s: the core does not slow down later in a run |
|  34 | Full page at Max, drawing on vs off                                   | 5,383 vs 5,098 events/s: drawing costs the simulation almost nothing       |

### A.4 Problems found by testing, and how they were fixed

| Problem                                                                                                                                                                                                                                          | Found by                                    | Fix                                                                                                 | Verified by                                          |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------- | --------------------------------------------------------------------------------------------------- | ---------------------------------------------------- |
| The parity test did not compare lights, frozen flag, target or distance travelled                                                                                                                                                                | Mutation test (light cooldown not caught)   | End-state comparison extended to the full robot state                                               | Same mutation now fails all 9 fixtures               |
| The browser ran at 1,497 events/s: the worker was idle ≈70 % of the time                                                                                                                                                                         | Browser smoke test vs raw WebAssembly speed | Worker simulates continuously; the page only asks for frames                                        | 5,008 events/s, worker busy 98 %                     |
| Pause followed by a quick Resume could start two simulation loops                                                                                                                                                                                | Code review of the pacing logic             | Generation counter: an older loop exits                                                             | Pause test (#18)                                     |
| `compare-speed.py` passed the wrong argument to Node                                                                                                                                                                                             | First run of the script                     | Argument index fixed                                                                                | Test #20                                             |
| Main-thread pause measurement included page load                                                                                                                                                                                                 | Page test results (#14–16)                  | Page-load and running pauses measured separately                                                    | 1.5 s at first load only; ≤ 44 ms while simulating   |
| Measured draw time excluded Chrome's painting                                                                                                                                                                                                    | Suspiciously low 0.6 ms at 10,000 robots    | Timing forces painting to finish (`getImageData`)                                                   | Real cost 7.43 ms (#16)                              |
| Legend showed "No robots" at the start                                                                                                                                                                                                           | UI check                                    | Legend updated from the draw that counted the colours                                               | Legend check passes                                  |
| H did not hide the panels (a more specific CSS rule won)                                                                                                                                                                                         | UI check                                    | Hiding rule made to win                                                                             | H check passes                                       |
| The world was centred under the settings panel                                                                                                                                                                                                   | Screenshot review                           | The world is fitted into the space the panels leave free                                            | Screenshots                                          |
| Two-key rows in the shortcuts dialog were split across columns                                                                                                                                                                                   | Screenshot review                           | Key combinations grouped                                                                            | Screenshot                                           |
| Drawing 10,000 robots took 14.3 ms per frame                                                                                                                                                                                                     | UI check (forced painting)                  | fillRect dots and stamped sprites (§7.4)                                                            | 3.7–6.0 ms                                           |
| "10,000 robots: speed" check read 2,872 events/s                                                                                                                                                                                                 | UI check                                    | Test fixed: it read the smoothed HUD value; it now counts events directly (the page runs at ≈5,200) | Check passes                                         |
| After a run ended the page kept fetching and redrawing frames                                                                                                                                                                                    | User noticed the draw time still changing   | Frame requests stop when the run ends                                                               | 0 frames in 1.5 s after the end (90 without the fix) |
| The fit button looked like full screen and seemed to do nothing                                                                                                                                                                                  | User report                                 | Real full-screen button added; fit has its own icon                                                 | Full-screen checks pass                              |
| Flowchart: legend icons black; empty gap above early steps; badges on text; hint over a card; file paths wrapping                                                                                                                                | Screenshot review                           | Styles fixed; the view never scrolls past the chart's edges                                         | Screenshots                                          |
| Flowchart: the event loop opened at 58 % zoom                                                                                                                                                                                                    | Flowchart check                             | Charts open fitted to width; the detail panel only opens on selection                               | Opens at 92 %                                        |
| Flowchart: 11 steps had no explanation of their own                                                                                                                                                                                              | Flowchart check                             | Explanation written for every step                                                                  | 48 / 48 steps                                        |
| A frame of a finished run arriving after Reset could mark the new run ended                                                                                                                                                                      | Code review before publishing               | Each run is numbered; frames of earlier runs are ignored                                            | New check passes                                     |
| Play pressed within 0.12 s of a settings change played the old settings                                                                                                                                                                          | Code review before publishing               | Play and stepping apply a pending change first                                                      | New check passes                                     |
| Flowchart 1: the stop question left out the time limit that paces playback; "crashed → skip" was wrong for a crashed robot's own crash event; crash-faulty robots were said to crash at their first activation (they are crashed from the start) | Re-checking every chart against the code    | Steps and explanations corrected                                                                    | Charts regenerated; 17 / 17 checks                   |

### A.5 Browser checks for the redesigned pages (30 September)

`node scripts/ui-check.mjs` drives the simulator page in headless Chrome (GPU off) with real clicks and key presses: **41 / 41 pass**.

| Area             | Checks                                                                                                                                                                                               |
| ---------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Loading          | The first swarm appears (≈130 ms); the page starts ready; the legend lists only colours present, with counts                                                                                         |
| Toolbar and view | "How it works" is a labelled toolbar button; the bottom tools no longer hold it; the full-screen button enters full screen and shows it can exit; Shift + F leaves it; Fit brings a zoomed view back |
| Stepping         | Step one event shows what happened; stepping pauses; step one time unit advances time by up to 1; → steps one event                                                                                  |
| Play / pause     | Space plays; Space pauses and time stays put                                                                                                                                                         |
| Inspector        | Clicking a robot opens its details; the robot is highlighted; Esc closes the card                                                                                                                    |
| Settings         | A change before Play rebuilds the swarm; a change during a run asks to reset; R resets with the new settings; Play straight after a change runs the new settings; fault colours appear in the legend |
| End of a run     | The run finishes and says so; Play is disabled; no frames are fetched or drawn afterwards; a stale frame from an earlier run cannot end a new one                                                    |
| Export           | CSV has a header and one full-precision row per robot; PNG export produces an image                                                                                                                  |
| Theme and panels | Follows the system dark setting; T cycles light → dark → system; H hides the panels; ? opens the shortcuts; Esc closes them                                                                          |
| 10,000 robots    | 60 fps; drawing under 16 ms per frame on the CPU (3.7–6.0 ms); the main thread never blocked over 50 ms (17 ms); at Max the worker stays busy (≈5,200 events/s, 97 %)                                |
| Health           | No errors in the console                                                                                                                                                                             |

`node scripts/docs-check.mjs` drives the flowchart page: **17 / 17 pass**.

| Area           | Checks                                                                                                                                                                                                 |
| -------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Loading        | The page loads with four charts; simulator buttons in the sidebar (top left) and the top bar (top right)                                                                                               |
| Steps          | Every one of the 48 steps opens its own explanation; the 20 code steps show highlighted code with line numbers                                                                                         |
| Tour           | Starts at step 1; → forward; ← back; the address links to the current step                                                                                                                             |
| Navigation     | A question links to each of its answers; following a link selects that step; hovering highlights a step's connections; zoom in and fit; number keys switch charts; a deep link opens a chart at a step |
| Theme and exit | Follows the system dark theme; the sidebar button opens the simulator                                                                                                                                  |
| Health         | No errors in the console                                                                                                                                                                               |

### A.6 Tests added on 1 October

| Test                                                                     | What it checks                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Result                                                                                                                                                                         |
| ------------------------------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `start::every_supported_combination_places_n_finite_robots_in_its_shape` | All 36 allowed combinations, for 1, 2, 7 and 500 robots: the right count, finite, inside / on the shape                                                                                                                                                                                                                                                                                                                                                                       | Pass                                                                                                                                                                           |
| `start::same_seed_same_start_and_even_ignores_the_seed`                  | The same seed gives the same start; "evenly spaced" does not depend on the seed, the others do                                                                                                                                                                                                                                                                                                                                                                                | Pass                                                                                                                                                                           |
| `start::even_placements_are_exact`                                       | Exact positions for a line, a circle, a 3 × 3 grid, and 8 robots on a square's edge (corners and midpoints)                                                                                                                                                                                                                                                                                                                                                                   | Pass                                                                                                                                                                           |
| `start::a_filled_polygon_is_covered_evenly`                              | 6,000 robots in a hexagon: each centre triangle holds 1/6 (±8 %), the inner half holds 1/4 (±2 %)                                                                                                                                                                                                                                                                                                                                                                             | Pass                                                                                                                                                                           |
| `start::free_cloud_has_the_requested_spread`                             | A 40,000-robot cloud is centred and has the requested σ                                                                                                                                                                                                                                                                                                                                                                                                                       | Pass                                                                                                                                                                           |
| `start::rotation_turns_the_shape`                                        | Rotation by 90°                                                                                                                                                                                                                                                                                                                                                                                                                                                               | Pass                                                                                                                                                                           |
| `start::mismatched_choices_are_rejected_with_a_clear_reason`             | e.g. a square ring, a Gaussian edge                                                                                                                                                                                                                                                                                                                                                                                                                                           | Pass                                                                                                                                                                           |
| `config::` 3 tests                                                       | Open world gives no bounds; own positions win over a generated start, which wins over the original box; a bad start is rejected                                                                                                                                                                                                                                                                                                                                               | Pass                                                                                                                                                                           |
| `ui-check.mjs`, 24 new checks                                            | Open world by default and the view fits the cloud; World mode gives the original box; only the controls that apply are shown; shape-specific labels; the summary sentence; circle edge, square edge, rectangle edge and filled polygon positions; only fitting distributions offered; own positions: count, locked robot count, exact positions, editor window, Esc, typo reported by line, JSON, CSV export pasted back; the tooltip clears; the core loaded is this build's | 65 / 65                                                                                                                                                                        |
| `docs-check.mjs`, 2 new checks                                           | `/web/` redirects to the main address (keeping the `#…` part); `/old/` loads the original Pyodide simulator                                                                                                                                                                                                                                                                                                                                                                   | 19 / 19 (one early run failed 2 checks and another stopped without a result; the next 7 runs passed; most likely the `/old/` check, which downloads Pyodide from the internet) |
| One-off: `/old/` with the Pyodide CDN blocked                            | The original page falls back to its bundled `pyodide/` from `/old/` too                                                                                                                                                                                                                                                                                                                                                                                                       | Failed at first (Pyodide resolved the folder against `/old/`); fixed by making the path absolute in `main.js`; then pass                                                       |

Unchanged and still passing: Python parity (9 / 9 fixtures), browser build = native bit for bit, `cargo clippy` 0 warnings, flowcharts up to date.
