"""Headless regression harness for the LCM simulator.

Runs the scheduler without the browser across every algorithm and asserts:
  * EVERY run TERMINATES (regression guard for the endless-simulation / tiny-move
    bug -- this is the universal invariant);
  * point-convergence algorithms (Gathering, Go-To-Center) actually cluster.

Run:  ./lcm/bin/python tests/test_convergence.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import robot
import scheduler
from scheduler import Scheduler
from robot import Algorithm, RobotState


class _Quiet:
    def info(self, *a): pass
    def warning(self, *a): pass
    def error(self, *a): pass


robot.Robot._logger = _Quiet()
scheduler.Scheduler._logger = _Quiet()

# algorithm -> (needs world box?, gathers to a point?)
ALGOS = {
    Algorithm.GATHERING:    (False, True),
    Algorithm.SEC:          (False, False),
    Algorithm.GO_TO_CENTER: (False, True),
    Algorithm.CIRCLE:       (False, False),
    Algorithm.SPREADING:    (True,  False),
    Algorithm.PATTERN:      (False, False),
}


def run(algo, n, seed, precision, max_events=250000):
    needs_box, _ = ALGOS[algo]
    rng = np.random.default_rng(seed)
    positions = rng.uniform(-50, 50, size=(n, 2)).tolist()
    kw = dict(
        seed=seed, num_of_robots=n, initial_positions=positions,
        robot_speeds=3.0, algorithm=algo, visibility_radius=None,
        threshold_precision=precision, sampling_rate=0.5,
        labmda_rate=5.0, multiplicity_detection=True,
    )
    if needs_box:
        kw.update(width_bound=300.0, height_bound=300.0)
    sch = Scheduler(**kw)
    events = 0
    while not sch.terminate and events < max_events:
        code, _, _ = sch.handle_event()
        events += 1
        if code == -1:
            break
    pts = [(r.coordinates.x, r.coordinates.y)
           for r in sch.robots if r.state != RobotState.CRASH]
    spread = max(
        (((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
         for i, a in enumerate(pts) for b in pts[i + 1:]),
        default=0.0,
    )
    return sch.terminate, events, spread


def main():
    fails = []
    total = 0
    for algo, (_, gathers) in ALGOS.items():
        for n in (5, 10):
            for seed in (1, 42, 7):
                total += 1
                term, ev, spread = run(algo, n, seed, precision=5)
                ok = term
                if gathers:
                    ok = ok and spread < 1e-2          # collapsed to ~a point
                if not ok:
                    fails.append((algo, n, seed, term, ev, spread))

    if fails:
        print(f"FAIL ({len(fails)}/{total} configs):")
        for algo, n, seed, term, ev, spread in fails:
            print(f"  {algo:16} n={n} seed={seed} term={term} "
                  f"events={ev} spread={spread:.2e}")
        return 1
    print(f"PASS: all {total} configs across {len(ALGOS)} algorithms terminated; "
          f"point-convergence algorithms collapsed to a point.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
