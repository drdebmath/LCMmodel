# SqGathering: Algorithm 8 on the sequential scheduler

Owner: Tirthanker Singh. Source: *Universal pattern formation by oblivious robots under
sequential schedulers*, arXiv:2412.10733v3, Section 2 (model) and Section 6 (Gathering).

SqGathering is an ordinary algorithm of the Rust core: it sits in the registry next to Gathering,
SEC and Uniform Circle Formation, and it runs on the **core's sequential scheduler**
(`scheduler: "sequential"`, chart 5 of the flowcharts). It has no scheduler of its own. It is
*not* the generic centre-of-gravity Gathering. Its flowchart is chart 6 of
[`web/docs.html`](../web/docs.html#sqgathering) (source: [`flowcharts.md`](flowcharts.md)).

| File | What it holds |
|---|---|
| [`crates/lcm-algorithms/src/sq_gathering.rs`](../crates/lcm-algorithms/src/sq_gathering.rs) | the rule (`observe`, `decide`), the `SqGathering` algorithm, its registry builder `plan` |
| [`crates/lcm-algorithms/src/sq_trace.rs`](../crates/lcm-algorithms/src/sq_trace.rs) | `Tracer`: watches a core simulation and records each turn (observation, κ, rule, stop) |
| [`crates/lcm-algorithms/tests/sq_gathering.rs`](../crates/lcm-algorithms/tests/sq_gathering.rs) | hand traces A–D, properties, refusals |
| [`crates/lcm-cli/src/seq.rs`](../crates/lcm-cli/src/seq.rs) | `lcm seq`: a SqGathering run with its turn-by-turn trace |
| [`web/sqgathering.html`](../web/sqgathering.html) | step-by-step page; the simulator page runs it too (Algorithm menu) |
| [`fixtures/sqgathering/`](../fixtures/sqgathering) | one core `SimConfig` per case, shared by the CLI, the parity check and the page |

## 1. The model (Section 2) and where it lives in the core

| Model element | Paper | Core |
|---|---|---|
| Cycle | Look–Compute–Move, oblivious, identical, silent | `SqGathering::compute` is a pure function of one snapshot |
| Scheduler | SEQ: one robot per round, fair | `scheduler: "sequential"`: round-robin, a reshuffle each epoch, or an explicit `schedule` |
| Movement | non-rigid: the adversary stops a robot after δ, never before | `rigid_movement: false`, `delta`, `stop_policy` (`delta`, `random`, `fraction`); a scheduled turn can name its own stop |
| Multiplicity | **weak**: one bit per occupied point | `observe()` builds Q′(t) from the snapshot; points closer than ε = 10^-precision are one |
| Epoch | every robot has been activated | the core's `epochs_completed` |
| Start | any configuration, multiplicities allowed | `initial_positions`, or `multiplicities` / `multiplicity_size` (stacked starts) |

The registry entry refuses what the paper's model does not have, with a message the page shows:
the **async scheduler** (with more than one robot active at once gathering is unsolvable even with
multiplicity detection, Lemma 8), **limited visibility** and **faults**. On the simulator page,
choosing SqGathering switches the scheduler to Sequential for you.

**Termination.** Robots in the paper never stop. In the core a run ends when every robot has
terminated, so a SqGathering robot terminates when it sees a single occupied point: everyone is
there, and every robot activated from then on would stay, so this changes nothing. Each robot
needs one more turn after the swarm has gathered; reports give both numbers ("gathered after 5
turns; the run ended after 10").

## 2. The decision rule (Algorithm 8)

With κ the number of multiplicity points in Q′(t) and b the bit of the active robot's point q:

| κ | b | Rule key | Lines | Destination |
|---|---|---|---|---|
| 1 | 0 | `k1-join-multiplicity` | 4–7 | the multiplicity point q_m |
| 1 | 1 | `k1-hold-multiplicity` | 3–5 | stay (q) |
| > 1 | 1 | `kmany-leave-multiplicity` | 9–13 | midpoint of q and its closest occupied point q′ |
| > 1 | 0 | `kmany-wait` | 3, 9–10 | stay |
| 0 | 0 | `k0-move-to-closest` | 14–16 | closest occupied point q′ |
| 0 | — | `alone` | 3 | stay (only when n = 1; not in the paper) |

- **The midpoint is always free.** q′ is the closest occupied point, so nothing occupied lies
  within |qq′|/2 of q, and the whole way to the midpoint is free wherever the robot is stopped.
  For the same reason a κ = 0 move never passes through another robot. A test checks both over
  150 random runs.
- **Ties** for "closest point" are broken by coordinates (x, then y). Any fixed choice is
  allowed. `decide_in_frame` checks that, without ties, the choice does not depend on a robot's
  rotation, scale or handedness.

## 3. Hand traces

Each row is one turn: the active robot, what it observes (s = single, M = multiplicity, * = itself),
κ and b, the rule, the destination and where the adversary stopped it. The tests in
`tests/sq_gathering.rs` assert every row below on the core scheduler (the fixture next to each
trace runs it with `lcm seq` or on the page).

### Trace A — no multiplicity (`k0_no_multiplicity.json`)

Start r0 (0,0), r1 (4,0), r2 (4,3), r3 (10,0). δ = 1. Schedule: r0 stopped at ½, r1 unstopped, then round-robin.

| Turn | Robot | Observes Q′(t) | κ | b | Rule | Destination | Stops at |
|---|---|---|---|---|---|---|---|
| 1 | r0 | *(0,0)s (4,0)s (4,3)s (10,0)s | 0 | 0 | closest: (4,0) at 4 (r2 at 5, r3 at 10) | (4,0) | (2,0): stopped after 2 ≥ δ |
| 2 | r1 | (2,0)s *(4,0)s (4,3)s (10,0)s | 0 | 0 | closest: (2,0) at 2 (vs 3, 6) | (2,0) | (2,0), reached: **first multiplicity** |
| 3 | r0 | *(2,0)M (4,3)s (10,0)s | 1 | 1 | hold | (2,0) | stays |
| 5 | r2 | (2,0)M *(4,3)s (10,0)s | 1 | 0 | join | (2,0) | (3.4453, 2.1679): exactly δ along √13 |

Gathered at (2,0) after 34 turns (only δ per move).

### Trace B — one multiplicity (`k1_one_multiplicity.json`)

Start r0, r1 at (0,0), r2 (3,4), r3 (−6,0). δ = 2.

| Turn | Robot | Observes | κ | b | Rule | Destination | Stops at |
|---|---|---|---|---|---|---|---|
| 1 | r2 | (0,0)M *(3,4)s (−6,0)s | 1 | 0 | join | (0,0) | (1.8, 2.4): distance 5 > δ, stopped after δ |
| 2 | r0 | *(0,0)M (1.8,2.4)s (−6,0)s | 1 | 1 | hold | (0,0) | stays |
| 3 | r3 | (0,0)M (1.8,2.4)s *(−6,0)s | 1 | 0 | join | (0,0) | (−3,0): stopped at ½ |
| 4 | r2 | (0,0)M *(1.8,2.4)s (−3,0)s | 1 | 0 | join | (0,0) | (0.6, 0.8) |
| 5 | r2 | (0,0)M *(0.6,0.8)s (−3,0)s | 1 | 0 | join | (0,0) | (0,0): within δ, must arrive |
| 6 | r3 | (0,0)M *(−3,0)s | 1 | 0 | join | (0,0) | (0,0): **gathered** |
| 7–10 | each | *(0,0)M | 1 | 1 | hold | (0,0) | terminates |

### Trace C — two multiplicities (`kmany_two_multiplicities.json`)

Start r0, r1 at (0,0); r2, r3 at (6,0); r4 (0,8). δ = 1.

| Turn | Robot | Observes | κ | b | Rule | Destination | Stops at |
|---|---|---|---|---|---|---|---|
| 1 | r4 | (0,0)M (6,0)M *(0,8)s | 2 | 0 | wait | (0,8) | stays |
| 2 | r0 | *(0,0)M (6,0)M (0,8)s | 2 | 1 | leave: q′ = (6,0), z = (3,0) | (3,0) | (1,0): (0,0) now holds only r1, κ = 1 |
| 3 | r1 | (1,0)s *(0,0)s (6,0)M (0,8)s | 1 | 0 | join | (6,0) | (6,0) |
| 4 | r0 | *(1,0)s (6,0)M (0,8)s | 1 | 0 | join | (6,0) | (6,0) |
| 5 | r4 | (6,0)M *(0,8)s | 1 | 0 | join | (6,0) | (6,0): gathered |

### Trace D — a non-rigid stop makes a multiplicity (`nonrigid_collision.json`)

Start r0, r1 at (0,0), r2 (2,0), r3 (4,0). δ = 1; the adversary stops robots right after δ, and r3's
first move at ½.

| Turn | Robot | Observes | κ | b | Rule | Destination | Stops at |
|---|---|---|---|---|---|---|---|
| 1 | r3 | (0,0)M (2,0)s *(4,0)s | 1 | 0 | join | (0,0) | (2,0): **on r2**, κ = 2 |
| 2 | r0 | *(0,0)M (2,0)M | 2 | 1 | leave | (1,0) | (1,0) |
| 3 | r1 | (1,0)s *(0,0)s (2,0)M | 1 | 0 | join | (2,0) | (1,0): **on r0 again**, κ = 2 |
| 4 | r2 | (1,0)M *(2,0)M | 2 | 1 | leave | (1.5,0) | (1.5,0) |
| 5 | r3 | (1,0)M (1.5,0)s *(2,0)s | 1 | 0 | join | (1,0) | (1,0) |
| 8 | r2 | (1,0)M *(1.5,0)s | 1 | 0 | join | (1,0) | (1,0): gathered |

Only a join (κ = 1) can stop on another robot, since the way to q_m may pass single robots. The
κ > 1 rule then separates them again; the proof of Theorem 5 does not spell this case out.

### Finding: large multiplicities and precision

Under κ > 1 a robot leaving a multiplicity at q goes to the midpoint towards its closest point,
and after the first leaver that closest point is the robot that left before it. Leavers land at
d/2, d/4, d/8, …; a multiplicity of m robots needs about m halvings. Once the midpoint is closer
than ε, the robot cannot leave, a whole epoch passes without progress, and the core ends the run
**Stalled**. Exact reals do not have this problem; any finite-precision implementation does. The
test `large_multiplicities_under_kappa_many_stall_at_the_precision_limit` reproduces it with two
multiplicities of 60. Stacked starts with κ ≥ 2 should use small multiplicities (the core's
default is up to 4 robots each).

## 4. Using it

```sh
cargo test -p lcm-algorithms --test sq_gathering       # hand traces, properties, refusals
cargo build --release -p lcm-cli
lcm seq fixtures/sqgathering/k1_one_multiplicity.json --trace -         # turn-by-turn table
lcm seq --positions "0,0;0,0;6,0;6,0;0,8" --schedule "4,0,1@1" --delta 1 --stop delta --trace -
lcm seq --robots 60 --multiplicities 3 --size 4 --schedule random --delta 2 --stop random --format json --trace t.jsonl
lcm run fixtures/sqgathering/k1_one_multiplicity.json   # the same run, core summary only
```

`lcm seq` takes an ordinary `SimConfig` file and/or flags: `--positions`, `--robots`, `--seed`,
`--schedule` (`round-robin`, `random`, or `r,r@stop,…` with an optional `+repeat`), `--rigid` or
`--delta D` with `--stop delta|random|FRACTION`, `--multiplicities K [--size S]`,
`--max-turns N`, `--trace -|FILE.jsonl`, `--format table|json`, `--print-config`. A run stopped by
a limit reports `INCOMPLETE (budget-limited)` and exits 3; a stalled run exits 4.

**In the browser** (`node scripts/serve.mjs`, then http://localhost:8000/):
- the simulator: Algorithm → *SqGathering*. The scheduler switches to
  Sequential; Settings → Start → *Stacked points* gives κ > 0 starts, Settings → Movement sets δ and
  where robots stop. Multiplicity points are drawn with a ring.
- the step-by-step view `/web/sqgathering.html`: a preset per multiplicity case, with the active
  robot, what it sees, κ and b, the rule and its lines of Algorithm 8, and the trace table.
  `#kmany_two_multiplicities@2` opens a preset at a turn.

## 5. Verification

All of these also run on GitHub's machines (`.github/workflows/ci.yml`), on every push and pull
request.

| Check | Command |
|---|---|
| Rust tests (whole workspace) | `cargo test --workspace` |
| Native vs Wasm, all fixtures and sequential runs (two SqGathering) | `node scripts/check-wasm-parity.mjs` |
| SqGathering traces: native vs Wasm, batch vs turn by turn | `node scripts/check-seq-parity.mjs` |
| Flowcharts match the code | `node scripts/gen-flowcharts.mjs --check` |
| Pages in headless Chrome | `node scripts/sqgathering-check.mjs`, `ui-check.mjs`, `responsive-check.mjs`, `docs-check.mjs` |
