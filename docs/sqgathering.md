# SqGathering: Algorithm 8 under a sequential scheduler

Owner: Tirthanker Singh. Source: *Universal pattern formation by oblivious robots under
sequential schedulers*, arXiv:2412.10733v3, Section 2 (model) and Section 6 (Gathering).

**This is not the simulator's generic `Gathering`.** That one is a centre-of-gravity rule run by
the continuous-time *asynchronous* event loop in `lcm_core::Simulation`. Algorithm 8 needs a
different model, so it has its own scheduler (`lcm_algorithms::seq`), its own CLI command
(`lcm seq`) and its own page (`web/sqgathering.html`). It is deliberately **not** in the
async `REGISTRY`, and `SeqConfig` rejects any other algorithm name, including `"Gathering"`.

## 1. The model it needs (Section 2)

| Model element | Paper | Here |
|---|---|---|
| Cycle | Look–Compute–Move, oblivious, identical, silent | `sq_gathering::decide` is a pure function of one observation |
| Frames | private origin (self), axes, unit, chirality | `LocalFrame`; `frames: random` gives every robot its own rotation, scale (0.25–4) and handedness |
| Scheduler | SEQ: exactly one robot per round, fair | `ScheduleSpec`: `round-robin`, `shuffled` (new permutation per epoch), `explicit` list then round-robin or repeat (a repeating list must name every robot) |
| Movement | non-rigid: the adversary may stop a robot after δ, and it always arrives if the destination is within δ | `MovementSpec::NonRigid { delta, stop }`; `stop` is `delta` (exactly δ, the slowest legal progress), `random` (uniform in [δ, L]) or `fraction`; an explicit activation can carry its own stop fraction. `rigid` is also available |
| Multiplicity | **weak**: one bit per occupied point, no count | `observe()` returns Q′(t) = `[(point, multiplicity bit)]`; a test checks 2 and 4 robots on a point give identical observations |
| Epoch | ends once every robot has been activated | counted per round in the trace |

Without multiplicity detection, Gathering is unsolvable under any sequential scheduler (Theorem 4).
With weak detection, Algorithm 8 solves it (Theorem 5).

## 2. The decision rule (Algorithm 8)

With κ the number of multiplicity points in Q′(t) and b the bit of the active robot's own point q:

| κ | b | Rule key | Lines | Destination |
|---|---|---|---|---|
| 1 | 0 | `k1-join-multiplicity` | 4–7 | the multiplicity point q_m |
| 1 | 1 | `k1-hold-multiplicity` | 3–5 | stay (q) |
| > 1 | 1 | `kmany-leave-multiplicity` | 9–13 | midpoint of q and its closest occupied point q′ |
| > 1 | 0 | `kmany-wait` | 3, 9–10 | stay |
| 0 | 0 | `k0-move-to-closest` | 14–16 | closest occupied point q′ |
| 0 | — | `alone` | 3 | stay (only when n = 1; not in the paper) |

Implementation notes:

- **The midpoint is always free.** q′ is the closest occupied point, so nothing occupied lies
  within |qq′|/2 of q. The whole segment q→z is free, wherever the adversary stops the robot.
  The same argument means a κ = 0 move never passes through another robot.
  `moves_never_land_on_or_cross_another_robot_except_joins` checks both over 150 random runs.
- **Ties** for "closest point" are broken by the robot's *own* coordinates (x, then y). Any
  deterministic choice is allowed. A disoriented robot's choice depends on its frame, and
  `ties_are_broken_in_the_robots_own_frame` shows it.
- **Exact destinations.** The decision names observed points (`Action::MoveTo(j)` /
  `Midpoint(j)`) rather than raw coordinates. So a decision taken in a rotated, scaled or
  reflected frame maps back to the exact global point, and co-location stays exact (no
  floating-point drift from frame round-trips).
- **Termination** is a property of the configuration, not of the robots: they never know they are
  done. The runner stops when |Q| = 1, because then every robot sees κ = 1, b = 1 and stays forever.

## 3. Hand traces

Each row records the active robot, what it observes (s = single, M = multiplicity, * = itself), κ
and b, the rule, the destination and the non-rigid stopping point. Every row below is asserted
exactly by `crates/lcm-algorithms/tests/sq_gathering.rs`, and each trace is a preset on the web page
and a fixture in `fixtures/sqgathering/`.

### Trace A — zero multiplicity points (`k0_no_multiplicity.json`)

Start r0 (0,0), r1 (4,0), r2 (4,3), r3 (10,0). δ = 1. Schedule: r0 stopped at ½, r1 unstopped, then round-robin.

| Round | Robot | Observes Q′(t) | κ | b | Rule | Destination | Stops at |
|---|---|---|---|---|---|---|---|
| 1 | r0 | *(0,0)s (4,0)s (4,3)s (10,0)s | 0 | 0 | closest: (4,0) at 4 (r2 at 5, r3 at 10) | (4,0) | (2,0): adversary stops it after 2 ≥ δ |
| 2 | r1 | (2,0)s *(4,0)s (4,3)s (10,0)s | 0 | 0 | closest: (2,0) at 2 (vs 3, 6) | (2,0) | (2,0), reached: **first multiplicity** |
| 3 | r0 | *(2,0)M (4,3)s (10,0)s | 1 | 1 | hold | (2,0) | stays |
| 5 | r2 | (2,0)M *(4,3)s (10,0)s | 1 | 0 | join | (2,0) | (3.4453, 2.1679): exactly δ along a path of length √13 |
| 6 | r3 | (2,0)M (3.4453,2.1679)s *(10,0)s | 1 | 0 | join | (2,0) | (9,0) |

Gathers at (2,0) after 34 rounds, with the adversary allowing only δ per move.

### Trace B — one multiplicity point (`k1_one_multiplicity.json`)

Start r0, r1 at (0,0) (a multiplicity), r2 (3,4), r3 (−6,0). δ = 2.

| Round | Robot | Observes | κ | b | Rule | Destination | Stops at |
|---|---|---|---|---|---|---|---|
| 1 | r2 | (0,0)M *(3,4)s (−6,0)s | 1 | 0 | join | (0,0) | (1.8, 2.4): distance 5 > δ, stopped after δ = 2 |
| 2 | r0 | *(0,0)M (1.8,2.4)s (−6,0)s | 1 | 1 | hold | (0,0) | stays |
| 3 | r3 | (0,0)M (1.8,2.4)s *(−6,0)s | 1 | 0 | join | (0,0) | (−3, 0): stopped at ½ (3 ≥ δ) |
| 4 | r2 | (0,0)M *(1.8,2.4)s (−3,0)s | 1 | 0 | join | (0,0) | (0.6, 0.8): distance 3, stopped after δ |
| 5 | r2 | (0,0)M *(0.6,0.8)s (−3,0)s | 1 | 0 | join | (0,0) | (0,0): distance 1 ≤ δ, must arrive |
| 6 | r3 | (0,0)M *(−3,0)s | 1 | 0 | join | (0,0) | (0,0) |

Gathered after 6 rounds. r1 was never activated, and it didn't need to be.

### Trace C — multiple multiplicity points (`kmany_two_multiplicities.json`)

Start r0, r1 at (0,0); r2, r3 at (6,0); r4 (0,8). δ = 1.

| Round | Robot | Observes | κ | b | Rule | Destination | Stops at |
|---|---|---|---|---|---|---|---|
| 1 | r4 | (0,0)M (6,0)M *(0,8)s | 2 | 0 | wait | (0,8) | stays |
| 2 | r0 | *(0,0)M (6,0)M (0,8)s | 2 | 1 | leave: q′ = (6,0) at 6 (vs (0,8) at 8), z = (3,0) | (3,0) | (1,0): after δ. (0,0) now holds only r1, so κ = 1 |
| 3 | r1 | (1,0)s *(0,0)s (6,0)M (0,8)s | 1 | 0 | join | (6,0) | (6,0) |
| 4 | r0 | *(1,0)s (6,0)M (0,8)s | 1 | 0 | join | (6,0) | (6,0) |
| 5 | r4 | (6,0)M *(0,8)s | 1 | 0 | join | (6,0) | (6,0): gathered |

### Trace D — a non-rigid stop creates a multiplicity (`nonrigid_collision.json`)

Start r0, r1 at (0,0), r2 (2,0), r3 (4,0). δ = 1, adversary always stops after δ (r3's first move
is stopped at ½).

| Round | Robot | Observes | κ | b | Rule | Destination | Stops at |
|---|---|---|---|---|---|---|---|
| 1 | r3 | (0,0)M (2,0)s *(4,0)s | 1 | 0 | join | (0,0) | (2,0): **on r2**, now κ = 2 |
| 2 | r0 | *(0,0)M (2,0)M | 2 | 1 | leave | (1,0) | (1,0) |
| 3 | r1 | (1,0)s *(0,0)s (2,0)M | 1 | 0 | join | (2,0) | (1,0): **on r0 again**, κ = 2 |
| 4 | r2 | (1,0)M *(2,0)M | 2 | 1 | leave | (1.5,0) | (1.5,0) |
| 5 | r3 | (1,0)M (1.5,0)s *(2,0)s | 1 | 0 | join | (1,0) | (1,0) |
| 8 | r2 | (1,0)M *(1.5,0)s | 1 | 0 | join | (1,0) | (1,0): gathered |

Only a join (κ = 1) can stop on another robot: the segment to q_m may contain singles. The
algorithm recovers through the κ > 1 rule. The proof of Theorem 5 in the paper does not spell this
case out. The random-run tests (300 seeds with random stops, schedules and frames) all gather.

### Finding: large multiplicities and floating-point precision

Under κ > 1, a robot leaving a multiplicity at q moves to the midpoint towards its closest point.
Once one robot has left, the closest point for the next one is the robot that just left. So robots
leaving the same multiplicity land at distances d/2, d/4, d/8, …. In exact arithmetic that never
ends, but a multiplicity of m robots needs about m halvings. With d ≈ 100, after about 36 halvings
the midpoint is closer to q than the 10⁻⁹ coincidence tolerance, and the robot can no longer leave.
Paper-level reals don't have this problem, but any implementation with finite precision (and any
physical robot) does. The runner detects it and stops with status **`stalled`**
(`STALLED (floating-point limit) …`) instead of looping forever. The test
`large_multiplicities_under_kappa_many_stall_at_the_precision_limit` reproduces it with two
multiplicities of 60 robots. Starts with κ ≥ 2 on the main page therefore use small
multiplicities.

## 4. Using it

```sh
cargo test -p lcm-algorithms --test sq_gathering        # hand traces + properties
cargo build --release -p lcm-cli
lcm seq fixtures/sqgathering/k1_one_multiplicity.json --trace -          # table
lcm seq --positions "0,0;0,0;6,0;6,0;0,8" --schedule "4,0,1@1" --delta 1 --stop delta --trace -
lcm seq --positions "0,0;9,9;3,-4" --schedule shuffled:7 --stop random:3 --frames random:5 \
        --max-rounds 500 --format json --trace trace.jsonl
lcm seq <config.json> --print-config                     # the resolved SeqConfig
```

Flags: `--algorithm` (only `SqGathering`), `--positions`, `--schedule` (`round-robin`,
`shuffled:SEED`, or `r,r@stop,…` with an optional `+repeat`), `--rigid` or `--delta D` with
`--stop delta|random:SEED|FRACTION`, `--frames global|random:SEED`, `--max-rounds N`,
`--trace -|FILE.jsonl`, `--format table|json`.

**Budget-limited runs** report `"status": "incomplete"`, print
`INCOMPLETE (budget-limited): …`, and exit with status 3 (0 = gathered, 2 = bad input).

**In the main simulator** (`/`): the algorithm menu lists the asynchronous algorithms, then
**Sequential ›**. Clicking it (or hovering, with a mouse) opens a flyout with SqGathering and the
sequential options:
- **starting κ** and **robots on each** (number fields): robots of the generated start are stacked
  onto κ points with that many robots each. Leaving "robots on each" empty picks a size: a third of
  the robots for κ = 1, 4 each for κ ≥ 2 (see the precision note below). Asking for more robots
  than exist gives a message. Every multiplicity point is drawn with an orange ring (up to 1,000
  robots).
- **movement** (rigid or non-rigid) and **δ**, which are the page's Rigid movement and Speed settings.
- **schedule**: shuffled each epoch, or round-robin.
- **where the adversary stops robots**: at random, after exactly δ, or halfway. This only applies
  to non-rigid movement.
- **robot frames**: random or shared.

Picking an algorithm or changing one of these options starts a new run at once, even while a run is
playing. SqGathering then runs
on the main canvas with the page's own start (robots, pattern, seed), via `WasmSeqSimulation`.
δ is the Speed setting, Rigid movement makes every move arrive, and the event limit is the round
budget. The stats card counts **Round** and **Gathered** (robots on the unique multiplicity).
Limited visibility and faults are refused, because Algorithm 8 doesn't model them.

Serve the page with `python scripts/serve.py` (http://localhost:8000/). Unlike
`python -m http.server`, it stops the browser from caching the page's JavaScript, which otherwise
mixes old and new files after an update. The step-by-step view is at `/web/sqgathering.html`,
linked from the Sequential flyout. It has one preset per multiplicity case plus the collision, random-frame and
budget runs. It shows step, back and play, a round slider, the active robot, the singleton or
multiplicity observation it saw, κ and b, the selected rule, and the highlighted lines of
Algorithm 8. Links like `#kmany_two_multiplicities@2` open a preset at a given round.

## 5. Verification

| Check | Command | Result |
|---|---|---|
| Algorithm tests (hand traces A–D, properties, menu mapping, κ starts, stall) | `cargo test -p lcm-algorithms --test sq_gathering` | 20 passed |
| Rule unit tests | `cargo test -p lcm-algorithms --lib` | included in the 43 lib tests |
| Whole workspace (incl. Python parity of the async core) | `cargo test --workspace` | all passed |
| Native vs Wasm, bit for bit, batch vs stepped | `node scripts/check-seq-parity.mjs` | 6/6 fixtures identical |
| Step-by-step page and the main simulator's Sequential menu (real mouse, κ = 0–3 runs), headless Chrome | `node scripts/sqgathering-check.mjs` | 52/52 |
