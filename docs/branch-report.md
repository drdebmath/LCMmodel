# Report: the extended-models, extras and UI branches, and how to merge them

Work on the fork `TirthankerSinghIIT/LCMmodel`, built on the sequential scheduler and SqGathering
(PRs #7 and #8 upstream). Nothing here has been merged into `drdebmath/LCMmodel` yet.

> **One branch: `combined`.** Everything below (`extended-models`, `extras`, `ui-clean`, `overnight`,
> `integrate-main`) is on the branch `combined`, together with upstream `main` up to 3efab0b (the
> Python implementation removed, the Node flowchart generator and dev server). It merges into
> `main` without conflicts. Checked on it: 229 Rust tests, 0 clippy warnings, browser = native bit
> for bit (50 cases), every algorithm under every scheduler rigid and not (120 runs, no crash; see
> `scripts/scheduler-matrix.mjs`), and the page checks. The older branches are kept only as history.
> On top of that, `combined` adds an info card for every setting, notes rewritten in full sentences,
> nested sub-options with a one-line summary of the timing, and a fuller ring page (see
> [whats-new](whats-new.md), *On the page*).

## In one paragraph

The simulator gained three schedulers (SSYNC, FSYNC, k-ASYNC), four robot-model options
(multiplicity detection none / local / weak / strong, private frames, unreliable compasses,
obstructed visibility), seven algorithms (Spreading and TwoTask ported, two-robot compass
gathering, two randomized gatherings, gathering despite crashes, mutual visibility) and a
separate ring-gathering model with its own page. The page gained a grouped algorithm menu, an
algorithm brief in plain words, settings in tabs, play-the-adversary, a scenario editor, an
inspector that explains each robot's next move, gliding robots, trails, pulses, compass roses and
phone/tablet layouts. Every default reproduces the earlier runs exactly: the Python parity
fixtures still replay event by event, and the browser build matches the native build bit for bit.

## What was added

### Schedulers and robot models (Rust core, `crates/lcm-core`)

| Feature | Setting (`SimConfig`) | Notes |
|---|---|---|
| SSYNC, FSYNC | `scheduler: "ssync" / "fsync"`, `ssync_probability`, `ssync_fairness`, `rounds`, `max_rounds` | All robots of a round look at one snapshot; explicit rounds can be given |
| k-ASYNC | `scheduler: "kasync"`, `k_async` | A Look that would break the bound waits until a cycle ends |
| Live schedules | `schedule_end: "await"` | Powers play-the-adversary: the run waits for the next chosen turn or round |
| Multiplicity detection | `multiplicity: none / local / weak / strong` | `Look::occupied()` gives each algorithm what its robot can perceive |
| Private frames | `frames: global / rotated / disoriented / compass` | Each Look in the robot's own coordinates; results mapped back |
| Unreliable compass | `compass_error`, `compass_static` | Error drawn once, or at every Look |
| Obstructed visibility | `obstructed_visibility`, `robot_radius` | Robots in line hide each other |
| Explanations | `Algorithm::explain`, `Simulation::explain` | What a robot would see and do if it looked now; changes nothing about the run (tested) |

### Algorithms (`crates/lcm-algorithms`, `crates/lcm-ring`)

| Algorithm | Menu | Checked by |
|---|---|---|
| Spreading, TwoTask (ported from the Python original) | Original simulator | Python fixtures replayed event by event |
| Two robots with unreliable compasses (fixed compass; changing compass in rounds; changing compass on their own) | Advanced gathering | Rule tables per sector; gathering across seeds, rigid and not |
| Randomized gathering (two variants of the coin) | Advanced gathering | Gathering across seeds and schedulers; no false "stalled" |
| Gathering despite crash faults (weak multiplicity) | Advanced gathering | 10 robots with 0, 3 or 9 crashed, under SSYNC, FSYNC and one at a time; robots in line still gather |
| Mutual visibility (obstructed visibility) | Other tasks | Lines and grids end with every robot seeing every other |
| Gathering on a ring (round robin, nodes) | Other tasks → its own page | 4,546 random starts gather within n − 3 rounds; unsolvable starts flagged; mutation-tested |

### The page (`web/`)

- **Algorithm menu** in four groups; *Advanced gathering* is one entry with its variant in the sidebar.
  The same menu is on the ring page (shared `web/menu.js`); choosing another algorithm there opens
  the simulator on it.
- **Algorithm brief** under the toolbar: what the algorithm does in plain words, chips for the model
  the run uses (settings the algorithm fixes carry a lock), and notes on where the settings leave
  what the algorithm was made for. It folds to one line while a run plays.
- **Settings in three tabs** (Robots, Model, Timing); locked settings show the reason.
- **Inspector**: click a robot for its state and, under *If it looked now*, what it sees and the
  rule it applies.
- **Play the adversary** (`A`) and the **scenario editor** (`D`); scenarios save and open as files.
- **Motion**: robots glide between frames (up to 4,000 robots), trails (`P`), a pulse when a robot
  picks a new target, a halo when the swarm finishes, compass roses for two-robot runs; a view the
  user has not moved widens to keep every robot in sight.
- **End of a run** says what happened: how many terminated (and that the rest are faulty), or for
  mutual visibility how many robots see every other one.
- **Ring page**: presets, click to place robots, turn order, speed slider, a key, plain wording of
  each robot's situation and move, robots sliding along the ring, CSV of every turn.
- Phone, tablet and short-laptop layouts, checked automatically.

### Findings worth reporting

1. **Two-robot compass gathering near its 30° bound is slow, not wrong.** Runs that seemed stuck at
   29° all gather when run longer (up to time 5.5 × 10⁷): the rotate rule turns by 178°, which nearly
   doubles the gap before the robots meet. Details: `docs/extras.md`, Findings.
2. **SqGathering's stall** with large stacks is a floating-point limit (halving distances fall
   below ε), not a flaw of the rule (`docs/sqgathering.md`).
3. **Mutual visibility needs a fine precision** on grids: at ε = 10⁻⁵ a 5 × 5 grid stops with 16 of 25
   robots seeing everyone because its moves fall below ε; at 10⁻⁹ all 25 do within two rounds.

## How it was checked

| Check | Result |
|---|---|
| `cargo test --workspace` | 229 passed, 1 ignored (the long compass run: passes in 15 s with `--release --ignored`) |
| `node scripts/check-wasm-parity.mjs` | browser build = native build, bit for bit; fixtures match the Python original |
| `node scripts/check-seq-parity.mjs` | SqGathering traces identical (native, browser batch, browser step by step) |
| `node scripts/gen-flowcharts.mjs --check` | flowcharts match the code |
| Headless Chrome: `extras-check`, `sqgathering-check`, `responsive-check`, `docs-check`, `embed-check`, `ui-check` | all pass on GitHub Actions, except `ui-check`'s 10,000-robot frame-rate targets, which depend on the machine and are reported, not enforced (they miss on `main` too) |

Every push to the fork runs all of the above on GitHub Actions.

## Known limits

- Gathering is about 7 % slower than `main` on a 2,000-robot run, with identical results. The code
  on the hot path is the same as `main`'s; the gap looks like code layout. Not fixed.
- Crash gathering's curved detour is approximated by one straight sideways step.
- Mutual visibility never stops by itself (true of the algorithm without lights).
- Not done: distinct gathering in the plane (rules not published), a ring algorithm for local
  multiplicity detection (the detection model is there, the algorithm is not).

## Branches

```
drdebmath/main (625a334)
        \
         extended-models ── extras ── ui-clean ── overnight ── integrate-main
         (models, adversary,  (compass, k-ASYNC,  (menu, motion,  (fixes, inspector, (overnight with
          editor, ports)       new algorithms,     phone layout)   brief, tabs, ring  main merged in,
                               ring)                               page, docs)        conflicts solved)
```

Each branch contains the ones before it. `integrate-main` is `overnight` with upstream `main`
(up to 625a334, the embed work) merged in, so it opens as a pull request with no conflicts.

## How to merge

### Recommended: one pull request from `integrate-main`

1. On GitHub, open a pull request from `TirthankerSinghIIT:integrate-main` into `drdebmath:main`.
2. Ask Samarth to review the core (`crates/lcm-core`), since the scheduler code is shared:
   `config.rs` (new fields, all defaulting to the old behaviour), `sim.rs` (rounds, k-ASYNC,
   frames, obstruction, explain), `algorithm.rs` (`occupied`, `randomized`, `stops_when_alone`,
   `explain`, all with defaults), and two new files moved out of SqGathering (`occupancy.rs`,
   `local.rs`). Everything else is new files or page code.
3. Wait for the checks on the pull request; they are the same as on the fork.

### Alternative: three smaller pull requests, in order

Easier to review, more work to keep in step. Open them one after another, each after the previous
one is merged, merging `main` into the next branch before opening it:

1. `extended-models` (core models, ports, adversary, editor)
2. `extras` (compass, k-ASYNC, local detection, the new algorithms, the ring)
3. `overnight` (the UI work and fixes; `ui-clean` is contained in it)

### If `main` moves again before the merge

```bash
git checkout integrate-main
git fetch origin
git merge origin/main
```

Then, for each conflict:

- **Generated files: never merge by hand; regenerate them.** `web/pkg/*` with
  `./scripts/build-wasm.sh`; `docs/flowcharts.md` and `web/docs.html` with
  `node scripts/gen-flowcharts.mjs`.
- **`web/app.js`, `index.html`**: keep both sides. Last time the overlap was the export menu (links
  and embedding from `main`, scenario files from here) and the algorithm menu (the grouped menu from
  `web/menu.js` replaces the flat list).
- **`.github/workflows/ci.yml`, `README.md`**: keep both sides' new steps and paragraphs.

Before pushing, run:

```bash
cargo test --workspace
node scripts/check-wasm-parity.mjs
node scripts/gen-flowcharts.mjs --check
```

and the headless-browser checks listed above; then push and wait for GitHub Actions.

## Every change, in order

| Commit | Change |
|---|---|
| c2d90b1 | Core: SSYNC and FSYNC, live await schedules, multiplicity models, private frames, obstructed visibility |
| 7f17875 | Spreading and TwoTask ported; every algorithm checked in private frames |
| e991422 | Page: the new settings, play the adversary, scenario editor, scenarios |
| 57d9785, 725950f | Toolbar fits on laptops; browser checks updated |
| 0047fb4 | Core: the async/sequential Look path kept as the original computes it |
| ecb47c7 | Core: unreliable compasses |
| cfa8d7c | Core: k-ASYNC scheduler |
| 5a04074 | Core: local multiplicity detection |
| 699a235 | Page: compass, k-ASYNC and local-detection settings; compass needles |
| 790bc18 | Two-robot compass gathering and randomized gathering |
| eb02ead, 919a38a | Mutual visibility |
| 99c02e0 | Gathering despite crash faults |
| d1bb6b4 | Ring gathering: `crates/lcm-ring`, `lcm ring`, `web/ring.html` |
| 21587fd | Examples for the new algorithms |
| d02452f | Docs for the extras |
| 478ed67, f221d21 | UI clean-up: grouped menu, Advanced gathering with locks, motion visuals, phone layout |
| 0c419c5 | How each algorithm's run ends; missing rule text |
| da9d9fe | CI checks for the ring page and the examples |
| 3cf6270 | Inspector explanations |
| 5bf9606 | Compass gathering at 29°: slow, not a counterexample |
| 605a364 | Ring page: big rings, speed slider, CSV |
| e6ac1e8 | `docs/whats-new.md` |
| 7d29c88 | Plain wording for every note and explanation |
| 055828e | Algorithm brief, settings tabs, gliding robots, compass rose, ring page refresh |
| 2acab82 | Merge of upstream `main` (case links, embedding) |
