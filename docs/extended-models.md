# Extended models: SSYNC/FSYNC, multiplicity detection, private frames, obstructed visibility

Additions to the Rust core and the page, building on the sequential scheduler (#8) and
SqGathering (#7). Everything is a `SimConfig` field (see `docs/core-schema.md`), so it works the
same in `lcm run`, in the browser and in tests. Defaults keep every earlier run unchanged: all
Python parity fixtures still replay event by event.

## 1. Schedulers

| `scheduler` | Who acts | Extra fields |
|---|---|---|
| `async` | every robot on its own random clock (the original) | `lambda_rate` |
| `sequential` | one robot per turn, a whole Look-Compute-Move | `activation_order`, `schedule`, `max_turns` |
| `ssync` | rounds: a non-empty set of robots looks at the same instant, then all move; the next round starts once all have stopped | `ssync_probability` (chance per robot, default 0.5), `ssync_fairness` (a robot left out this many rounds in a row is forced in, default 4), `rounds` (explicit sets, e.g. `[[0,1],[2,3]]`), `max_rounds` |
| `fsync` | rounds with every live robot | `max_rounds` |

All robots of a round compute from one snapshot (positions *and* flags as they were when the round
began), even if one of them terminates or moves instantly. `turn_gap` sets the pause between
rounds as between turns. Epochs (every live robot active at least once) and the stall check work
as for the sequential scheduler.

**The adversary, live.** `schedule_end: "await"` (with an empty `schedule`, or `rounds`) makes the
run wait: `advance` returns `AwaitingSchedule` until `push_turn(robot, stop)` or
`push_round(robots)` adds the next choice. `preview_target(i)` shows where robot `i` would go,
without changing the run. The page's **Adversary** button is built on this.

## 2. Robot model

- `multiplicity`: `none`, `weak` (default) or `strong`. Algorithms read it through
  `Look::occupied()`, which groups robots closer than ε into points, each with a multiplicity bit
  (weak) or count (strong). The ports of the Python algorithms read the raw view and ignore it.
- `frames`: `global` (default), `rotated` (at each Look, a private frame centred on the robot with
  a random rotation and unit length in [½, 2]) or `disoriented` (also a random mirror: no
  chirality). The robot computes in its frame; the core converts the destination (and any circle)
  back. Frames come from a generator of their own, so turning them on changes nothing else.
- `obstructed_visibility` with `robot_radius`: robot *j* is hidden from *i* if another robot stands
  on the segment between them (within the radius, or ε for point robots). O(n²) per Look.

**Which algorithms need a shared frame?** `tests/local_frames.rs` takes one Compute in the global
frame and one in a random private frame for 300 random configurations per algorithm:

| algorithm | rotated frame | mirrored frame |
|---|---|---|
| Gathering, SEC, Uniform Circle Formation, Go-To-Center, TwoTask, SqGathering | 100 % same | 100 % same |
| Spreading | 0 % | 0 % |

Spreading samples an axis-aligned grid over an axis-aligned box, so a disoriented robot computes a
different Voronoi centroid.

## 3. Section 6 of the paper, played out

`crates/lcm-algorithms/tests/synchronous.rs` (and the SqGathering examples on the page):

- **Lemma 8** (SSYNC): two multiplicities of two robots, 64 apart; rounds `[[0,1],[2,3]]`
  repeated. Each round both robots of one multiplicity take the κ > 1 rule to the same midpoint,
  so after every round there are again two multiplicities, half as far apart. The same start under
  the sequential scheduler gathers.
- **Theorem 4** (no multiplicity detection): robots 0 and 1 on one point, robot 2 alone; schedule
  0, 2, 1 repeated. Every robot sees two points and goes to the other, so after every turn there
  are again two points. With weak detection the same schedule gathers.
- **Rendezvous**: two robots gather in one turn (κ = 0).

SqGathering now runs under SSYNC/FSYNC, without detection, with limited or obstructed
visibility, so these cases can be shown; the page shows a note saying where the run leaves the
model of Theorem 5 (`lcm_algorithms::notes`). It still refuses the async scheduler and faults.

## 4. Ported algorithms

Spreading and TwoTask (the original simulator's red SEC / blue Gathering draw) are ported with 6 new
recorded runs of the original Python implementation (made before it was removed from the repository).
Every algorithm of the original simulator except PatternFormation is now in the Rust core.

## 5. Page

- **Settings:** SSYNC/FSYNC with their options; *Robot model* (multiplicity detection, frames);
  *Robots block the view*; an explicit schedule line (`0, 2, 1@0.5` or `0 1 | 2 3`).
- **Adversary** (`A`): click the robot that moves next, then click it again (or Enter) to let it
  arrive, or click on its path to stop it there (non-rigid, never before δ). Under SSYNC, click the
  robots of the round and press Enter. The choices so far can be copied, or kept as the run's
  schedule to replay.
- **Scenario editor** (`D`): click to add a robot, drag to move, Shift + click to stack another
  robot on it, Alt + click / right-click / Delete to remove, Ctrl + Z to undo. Starts from the robots
  on screen and writes Settings → Start → My own positions.
- **Scenarios:** Export menu → save the algorithm and all settings as JSON, open one, or copy a
  link (`#scenario=…`) that opens exactly this setup.
- **Examples** for SqGathering (rendezvous, Lemma 8, Theorem 4, limited visibility), Gathering
  (FSYNC, SSYNC, obstructed view) and Spreading (box, disoriented).

## 6. Limits

- Obstructed visibility costs O(n²) per Look; keep swarms in the hundreds.
- The scenario editor handles up to 2,000 robots and rounds positions to 0.001.
- The SqGathering trace (`lcm seq`, the step-by-step page) is for the sequential scheduler only.
