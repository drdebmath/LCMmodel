# Extras: plan (branch `extras`, from `extended-models`)

## 0. Where things stand (before this work)

### Simulator (Rust core, `crates/`)
- **Engine:** event-driven Look-Compute-Move, one event queue, deterministic from the seed.
  Replays the original Python simulator event by event (37 recorded runs); the browser build is
  bit-for-bit the native one. 10,000 robots at 60 fps in the browser, CPU only.
- **Schedulers:** async (the original), sequential (Samarth, #8), SSYNC and FSYNC (extended-models),
  explicit schedules, a live "await" schedule for an adversary.
- **Robot model:** rigid / non-rigid with δ and three stop policies; visibility radius; obstructed
  visibility with a robot radius; multiplicity detection none / weak / strong; private frames
  global / rotated / disoriented; faults crash, Byzantine, omission, delay.
- **Algorithms (7):** Gathering (centre of gravity), SEC, Uniform Circle Formation, Go-To-Center,
  Spreading, TwoTask, SqGathering. Only PatternFormation is still Python-only.
- **Tools:** `lcm run` (summary + per-event trace), `lcm seq` (SqGathering turn table), `lcm bench`.
- **Known issue:** on extended-models, Gathering is ~4-5 % slower than main on a fixed workload
  (results identical). Cause not found yet.
- **Not modelled:** discrete spaces (graphs, rings); lights as memory/communication (LUMI);
  randomised algorithms beyond faults; bounded asynchrony; compass error.

### Page (`index.html`, `web/`)
- Full-screen canvas; toolbar (algorithm, examples, editor, adversary, play/step/reset, speed with
  per-turn pacing); settings panel built from one list (`web/settings.js`); status card (time,
  terminated, swarm-size plot, scale, zoom, grid, turns/rounds and epochs, event rate); inspector
  for one robot; legend; view tools (zoom, follow, fit, grid, measure, links, export, theme, help);
  overview map; turn caption; adversary and editor panels; model notes; flowcharts page; the
  SqGathering step-by-step page.
- **Weak spots worth researching:** no rewind or timeline; the inspector shows raw fields, not
  "what this robot sees"; one long settings list; no charts across runs (sweeps); new panels
  untested by CI and unchecked on phones; per-turn pacing only for the sequential scheduler; no
  way to draw a ring or graph.

## 1. Features, in build order

| # | Feature | Source | Core change | Test that proves it |
|---|---|---|---|---|
| 1 | **Compass error φ** (static: one deviation per robot; dynamic: a new one each Look) | Izumi et al. 2012 [4] | `frames: "compass"` + `compass_error`, `compass_static`; frames drawn in [-φ, φ], unit length kept, handedness shared | the two-robot algorithm of [4] gathers for φ below the bound and the test shows a failing schedule above it (as far as the paper's constructions allow) |
| 2 | **k-ASYNC scheduler** | Kirkpatrick et al. 2024 [9] | async, but while robot j is in a cycle, no other robot starts more than k Looks; a Look that would break this waits until j's cycle ends | property test over random runs: the bound is never broken; k = ∞ equals async |
| 3 | **Local multiplicity detection** | Kamei et al. 2011 [20] | `multiplicity: "local"`: a robot knows only whether its own point is shared | unit tests on `Look::occupied`; SqGathering under local detection (expected to fail: it needs κ) |
| 4 | **Gathering with weak multiplicity and crash faults** | Pattanayak et al. 2019 [22] | new algorithm, from the paper's rules | gathers the non-faulty robots with crashed robots present, over many seeds |
| 5 | **Gathering on a ring** | Navarra & Piselli 2025 [13] | new discrete engine (`lcm-ring`): n nodes, robots on nodes, round robin; its own config, trace, Wasm entry | the paper's cases, hand traces, every start that the paper says is gatherable gathers |
| 6 | **Randomized gathering** | Pattanayak, Augustine, Mandal 2021 [21] | new algorithm using the run's generator | gathers from symmetric starts that deterministic gathering cannot break, over many seeds |
| 7 | **Mutual visibility** | from the obstructed-visibility literature ([30] and others) | new algorithm under obstructed visibility | ends with every robot seeing every other (no three collinear in the way) |

Every algorithm is implemented **from the paper's own rules**. Where a paper could not be read,
or a rule had to be completed, the code and docs say so.

## 2. Page changes

- **Settings:** Robot model → Frames gains "Compass with error φ" with φ and static/dynamic;
  Scheduler gains "k-ASYNC" with k; Multiplicity gains "Local".
- **Drawing:** a small compass needle on each robot (its own north) when compass error is on;
  under k-ASYNC, robots in an active cycle outlined; ring view: nodes on a circle, robots stacked
  on nodes with counts, the active robot and its move.
- **Ring mode:** picking "Ring gathering" switches the canvas to the ring view and the settings
  to ring settings (nodes, robots, start), with examples from the paper and the adversary and
  editor working on nodes (click a node to add or remove a robot).
- **Examples:** compass just below and above the bound; k = 1 vs large k; crash-tolerant
  gathering; randomized gathering from a symmetric start; mutual visibility on a line.
- **Notes:** each algorithm's note says where a run leaves the paper's model.

## 3. How each piece is checked

`cargo test --workspace`; native = Wasm parity for the new runs; Python parity unchanged (defaults
untouched); the browser checks on GitHub Actions for every push to `extras` on the fork. Nothing
goes to `main`; no pull request without asking.
