"""Headless regression harness for the LCM simulator.

Runs the scheduler without the browser and asserts every configuration
TERMINATES (regression guard for the endless-simulation / tiny-move bug) and
that point-convergence algorithms actually cluster.

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


def run(algo, n, seed, precision, max_events=50000):
    """Run one simulation to completion; return (terminated, events, spread, t)."""
    rng = np.random.default_rng(seed)
    positions = rng.uniform(-50, 50, size=(n, 2)).tolist()
    sch = Scheduler(
        seed=seed, num_of_robots=n, initial_positions=positions,
        robot_speeds=1.0, algorithm=algo, visibility_radius=None,
        threshold_precision=precision, sampling_rate=0.2,
        labmda_rate=5.0, multiplicity_detection=True,
    )
    events = 0
    t = 0.0
    while not sch.terminate and events < max_events:
        code, t, _ = sch.handle_event()
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
    return sch.terminate, events, spread, t


def main():
    fails = []
    for algo in (Algorithm.GATHERING, Algorithm.SEC):
        for n in (3, 7, 15):
            for seed in (1, 42, 12345):
                for prec in (5, 9):  # high precision => tiny moves => stresses the bug
                    term, ev, spread, t = run(algo, n, seed, prec)
                    # Gathering must cluster within ~threshold; SEC just settles.
                    tol = 10 ** -(prec - 1) * max(1, n)
                    ok = term and (algo != Algorithm.GATHERING or spread <= tol)
                    if not ok:
                        fails.append((algo, n, seed, prec, term, ev, spread))

    if fails:
        print(f"FAIL ({len(fails)} configs):")
        for algo, n, seed, prec, term, ev, spread in fails:
            print(f"  {algo} n={n} seed={seed} prec={prec} "
                  f"term={term} events={ev} spread={spread:.2e}")
        return 1
    print("PASS: all 36 configs terminated; Gathering converged within threshold.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
