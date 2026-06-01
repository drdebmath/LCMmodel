"""Integration check of the JS <-> Python (Pyodide) contract.

main.js calls run.setup_simulation / run_simulation_step / stop_simulation with a
JSON params blob and renders the returned JSON. This exercises that exact path for
every algorithm offered in the dropdown, without a browser, asserting the response
shape the canvas renderer relies on.

Run:  ./lcm/bin/python tests/test_integration.py
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import robot
import scheduler
import run as sim


class _Quiet:
    def info(self, *a): pass
    def warning(self, *a): pass
    def error(self, *a): pass


robot.Robot._logger = _Quiet()
scheduler.Scheduler._logger = _Quiet()
sim.log_info = lambda *a: None
sim.log_error = lambda *a: None

# every value offered in the index.html algorithm <select>
ALGORITHMS = ["Gathering", "SEC", "GoToCenter", "CircleFormation",
              "Spreading", "PatternFormation", "TwoTask"]

# keys main.js reads off each robot record
ROBOT_KEYS = ("id", "x", "y", "state", "frozen", "terminated", "crashed",
              "sec", "target_x", "target_y", "color", "visibility_radius")


def js_params(algo):
    """Mirror exactly what main.js params() sends."""
    return {
        "algorithm": algo,
        "num_of_robots": 8,
        "robot_speeds": 2.0,
        "visibility_radius": None,          # "Inf" toggle on
        "num_of_faults": 0,
        "rigid_movement": True,
        "width_bound": 600,
        "height_bound": 600,
        "lambda_rate": 5.0,
        "sampling_rate": 0.2,
        "threshold_precision": 5,
        "random_seed": 42,
        "initial_positions": [],
    }


def main():
    fails = []
    for algo in ALGORITHMS:
        setup = json.loads(sim.setup_simulation(json.dumps(js_params(algo))))
        if setup.get("status") != "initialized":
            fails.append(f"{algo}: setup status={setup.get('status')} msg={setup.get('message')}")
            continue
        robots = setup.get("robots") or []
        if len(robots) != 8:
            fails.append(f"{algo}: expected 8 robots, got {len(robots)}")
            continue
        missing = [k for k in ROBOT_KEYS if k not in robots[0]]
        if missing:
            fails.append(f"{algo}: robot record missing keys {missing}")
            continue

        ended = saw_running = err = False
        for _ in range(20000):
            step = json.loads(sim.run_simulation_step())
            st = step.get("status")
            if st == "error":
                err = True
                fails.append(f"{algo}: runtime error: {step.get('message')}")
                break
            if st == "running":
                saw_running = True
            if st == "ended":
                ended = True
                break
        if not err and not ended:
            fails.append(f"{algo}: did not end within step budget")
        # TwoTask should colour robots red/blue; others leave colour None
        if algo == "TwoTask":
            colours = {r["color"] for r in setup["robots"]}
            if not colours & {"red", "blue"}:
                fails.append(f"{algo}: expected red/blue robot colours, got {colours}")
        print(f"  {algo:16} setup+run ok (ended={ended}, saw_running={saw_running})")

    if fails:
        print("\nFAIL:")
        for f in fails:
            print("  -", f)
        return 1
    print(f"\nINTEGRATION PASS: all {len(ALGORITHMS)} dropdown algorithms run end-to-end "
          f"and return the renderer's expected JSON shape.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
