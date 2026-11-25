# More models and algorithms

What the core and the page add beyond the original simulator: schedulers, robot models,
algorithms and the page tools built on them. Every model is a `SimConfig` field, documented row by
row in [`core-schema.md`](core-schema.md) §2 (private axes in §7.1), so it works the same in
`lcm run`, in the browser and in tests. Defaults keep every earlier run unchanged: the recorded
Python runs still replay event by event.

## 1. Schedulers

| `scheduler` | Who acts | Extra fields |
|---|---|---|
| `async` | every robot on its own random clock (the original) | `lambda_rate` |
| `sequential` | one robot per turn, a whole Look-Compute-Move | `activation_order`, `schedule`, `max_turns` |
| `ssync` | rounds: a non-empty set of robots looks at the same instant, then all move; the next round starts once all have stopped | `ssync_probability`, `ssync_fairness`, `rounds`, `max_rounds` |
| `fsync` | rounds with every live robot | `max_rounds` |
| `kasync` | as `async`, but while one robot is in a cycle no other robot starts more than k Looks (Kirkpatrick et al., Distributed Computing 2024) | `k_async` |

All robots of a round compute from one snapshot (positions *and* flags as they were when the round
began). Epochs (every live robot active at least once) and the stall check work as for the
sequential scheduler. `tests/kasync.rs` checks with an outside observer that no cycle ever sees more
than k Looks of another robot, and that plain async breaks that.

**The adversary, live.** `schedule_end: "await"` (sequential with a `schedule`, or SSYNC with
`rounds`) makes the run wait: `advance` returns `AwaitingSchedule` until `push_turn(robot, stop)` or
`push_round(robots)` adds the next choice. `preview_target(i)` shows where robot `i` would go
without changing the run. The page's **Adversary** button is built on this.

## 2. Robot model

- **Multiplicity detection** (`multiplicity`): `none`, `local` (only whether its own point is
  shared; Kamei et al., SIROCCO 2011), `weak` (default) or `strong`. Algorithms read it through
  `Look::occupied()`; the ports of the Python algorithms read the raw view and ignore it.
- **Robots' axes** (`frames`): `global` (default), `rotated`, `disoriented` or `compass`
  (`compass_error`, `compass_static`; Izumi et al., SIAM J. Comput. 2012). What a `Look` then
  contains is in core-schema §7.1.
- **Robots block the view** (`obstructed_visibility`, `robot_radius`): robot *j* is hidden from *i*
  if another robot stands on the segment between them. O(n²) per Look, so keep swarms in the
  hundreds.

**Which algorithms need shared axes?** `tests/local_frames.rs` computes each algorithm once in the
global frame and once in a random private frame, over random configurations. Every registered
algorithm gives the same move except three, listed there with the reason: Spreading samples an
axis-aligned grid; CompassGathering reads its own compass on purpose (and refuses `rotated` and
`disoriented` axes, which break the paper's bounded deviation and shared handedness);
MutualVisibility picks the first of two equally wide gaps in its own angular order, and either is a
legal move in the paper.

## 3. Algorithms

| Algorithm (registry key) | Source | Checked by |
|---|---|---|
| `Spreading`, `TwoTask` | the original Python simulator | 6 recorded runs of the Python implementation (`fixtures/`) |
| `CompassGathering`: A_SS, A_SD, A_AD | Izumi et al., SIAM J. Comput. 2012 (arXiv:1111.1492) | `tests/compass_gathering.rs`: gathers below each rule set's bound, the exact rule boundaries, termination in rounds but not under async (Remark 2), k-ASYNC |
| `RandomizedGathering`, `RandomizedGatheringUniform` | Pattanayak, Augustine, Mandal, TCS 2021 (arXiv:1909.10004) | `tests/randomized_gathering.rs`: gathers under async, SSYNC, FSYNC; average Looks within Theorem 12's bound where its hypothesis holds |
| `MutualVisibility` | Di Luna et al., Inf. Comput. 2017 (arXiv:1503.04347v2, §6.7) | lines and grids: everyone sees everyone after one epoch, and still after six |
| `CrashGathering` | Pattanayak, Mondal, Ramesh H., Mandal (arXiv:1608.02432), Algorithm 3 | `tests/crash_gathering.rs`: gathers with crashed robots, never makes a second stack (Lemma 4) |

Choices where a paper leaves something open are stated in each module's header and in its notes
on the page:

- **CrashGathering:** the paper's blocked robots follow circular arcs; here a blocked robot takes
  one short straight step off its path, then goes straight. Crashes happen at the start of the run
  here; the paper allows any time.
- **MutualVisibility:** "a small amount" is half the distance to the band around the nearest line
  through two visible robots, along the middle of the widest free angle. The robots never
  terminate, as in the paper.
- **Randomized gathering:** two robots only.
- **CompassGathering under async:** robots do not terminate (Remark 2 of the paper).

Two core hooks these needed, both off by default: `Algorithm::randomized()` skips the "an epoch
without progress means stalled" check, and `Algorithm::stops_when_alone()` lets an algorithm turn
off the original rule that a robot seeing no other active robot terminates.

**A_AD near its bound is slow, not wrong.** Theorem 6 says A_AD gathers for every φ < 30°. At
φ = 29° a move to the other robot turned by 2π/3 + 2φ ≈ 178° lands the robot almost opposite, so
the gap can double several times before a Look lands the robots together. Run long enough, every
such run gathers: `a_ad_at_29_degrees_is_slow_but_gathers` (ignored by default, about 15 s in
release) checks the two slowest runs found.

## 4. SqGathering and Section 6 of the paper

SqGathering runs only under the sequential scheduler with unlimited visibility and no faults, as in
the paper. Under no or local multiplicity detection, or when robots block the view, it runs with a
note saying the run leaves the paper's model. `crates/lcm-algorithms/tests/synchronous.rs` plays
out the impossibility results on the core:

- **Lemma 8** (SSYNC): two stacks of two robots, rounds `[[0,1],[2,3]]` repeated: after every round
  there are again two stacks, half as far apart. The same start under the sequential scheduler
  gathers.
- **Theorem 4** (no multiplicity detection): robots 0 and 1 on one point, robot 2 alone, schedule
  0, 2, 1 repeated: there are always two points. With weak detection the same schedule gathers.
  The page's SqGathering examples show this one.

## 5. On the page

- **Settings** in three tabs: *Robots* (swarm, start), *Model* (movement, visibility, robot model,
  faults), *Timing* (scheduler, clock and precision). Each setting has an ⓘ card; the longer
  explanations are here and in core-schema §2.
- **Advanced gathering…** in the Algorithm menu: the variant (compasses, randomized, crash faults)
  is chosen in the sidebar, which locks the settings the variant fixes, with the reason.
- **Adversary** (`A`): click the robot that moves next, then click it again (or Enter) to let it
  arrive, or click on its path to stop it there (non-rigid, never before δ). Under SSYNC, click the
  robots of the round and press Enter. The choices so far can be copied, or kept as the run's
  schedule to replay.
- **Scenario editor** (`D`): click to add a robot, drag to move, Shift + click to stack another
  robot on it, Alt + click / right-click / Delete to remove, Ctrl + Z to undo. Up to 2,000 robots;
  positions are rounded to 0.001.
- **Scenarios:** Export menu → save the algorithm and all settings as JSON, open one, or copy a
  link (`#scenario=…`) that opens exactly this setup.
- **Motion trails** (`P`, off by default: they cost a path per robot), compass needles, and a
  model-notes panel that says when a run leaves its algorithm's model.

## 6. Not done

- Frei & Wada's distinct gathering in the plane (only a brief announcement; rules not published).
- Kamei et al.'s ring algorithm (local detection is modelled, their algorithm is not).
