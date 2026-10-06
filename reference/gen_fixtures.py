"""Generate parity fixtures for lcm-core from the unmodified Python simulator.

The only substitution is numpy's generator, replaced by PortableRng so that
Rust can reproduce the same random streams. Each fixture holds the config
(in SimConfig JSON form), the first RECORDED events, and the final state.

Run from the repository root (needs numpy, Python >= 3.12):
    python reference/gen_fixtures.py
"""
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from portable_rng import PortableRng

np.random.default_rng = lambda seed=None: PortableRng(seed)

import io, contextlib
with contextlib.redirect_stdout(io.StringIO()):
    import robot
    import scheduler
    import run


class _Quiet:
    def info(self, *a): pass
    def warning(self, *a): pass
    def error(self, *a): pass


robot.Robot._logger = _Quiet()
scheduler.Scheduler._logger = _Quiet()
run.log_info = lambda *a: None
run.log_error = lambda *a: None

RECORDED = 1500
MAX_EVENTS = 200_000
STATE = {"WAIT": 0, "LOOK": 1, "MOVE": 2, "CRASH": 3}
KIND = {"CRASH": 0, "LOOK": 1, "WAIT": 2, "VISUALIZE": 3}


def config(**over):
    c = {
        "algorithm": "Gathering",
        "num_of_robots": 8,
        "initial_positions": None,
        "robot_speeds": 1.0,
        "visibility_radius": None,
        "num_of_faults": 0,
        "fault_type": "crash",
        "rigid_movement": True,
        "width_bound": 600,
        "height_bound": 600,
        "lambda_rate": 5.0,
        "sampling_rate": 0.1,
        "threshold_precision": 5,
        "random_seed": 1,
        "multiplicity_detection": False,
        "max_events": None,
        "max_time": None,
    }
    c.update(over)
    return c


CASES = {
    # Gathering drives these; together they cover every core behaviour.
    "gathering_8": config(),
    "gathering_20_vis150": config(num_of_robots=20, visibility_radius=150, random_seed=2),
    "gathering_8_nonrigid": config(rigid_movement=False, random_seed=3),
    "fault_crash": config(num_of_robots=10, num_of_faults=3, fault_type="crash", random_seed=12),
    "fault_byzantine": config(num_of_robots=10, num_of_faults=2, fault_type="byzantine", random_seed=13),
    "fault_omission": config(num_of_robots=10, num_of_faults=3, fault_type="omission", random_seed=14),
    "fault_delay": config(num_of_robots=10, num_of_faults=3, fault_type="delay", random_seed=15),
    "fault_mixed": config(num_of_robots=12, num_of_faults=6, fault_type="mixed", random_seed=16),
    "explicit_positions": config(num_of_robots=4, random_seed=17,
                                 initial_positions=[[0, 0], [10, 0], [0, 10], [10, 10]]),
    # SEC cases.
    "sec_8":               config(algorithm="SEC"),
    "sec_20":              config(algorithm="SEC", num_of_robots=20, random_seed=4),
    "sec_fault_crash":     config(algorithm="SEC", num_of_robots=10,
                                  num_of_faults=3, fault_type="crash", random_seed=20),
}


def run_params(c):
    p = dict(c)
    p["initial_positions"] = c["initial_positions"] or []
    return json.dumps(p)


def pt(p):
    return None if p is None else [float(p.x), float(p.y)]


def robot_record(r):
    return {
        "pos": pt(r.coordinates),
        "target": pt(r.calculated_position),
        "state": STATE[r.state],
        "frozen": r.frozen,
        "terminated": r.terminated,
        "light": r.current_light,
        "circle": None if r.sec is None else [float(r.sec.center.x), float(r.sec.center.y), float(r.sec.radius)],
        "travelled": float(r.travelled_distance),
        "speed": float(r.speed),
        "fault": r.fault_type,
        "color": r.color,
    }


def generate(name, c):
    setup = json.loads(run.setup_simulation(run_params(c)))
    assert setup["status"] == "initialized", setup
    s = run.scheduler_instance
    initial = [robot_record(r) for r in s.robots]
    events = []
    count = 0
    while not s.terminate and count < MAX_EVENTS:
        t, rid, kind = s.priority_queue[0]
        code, _, _ = s.handle_event()
        count += 1
        if count <= RECORDED:
            rec = [float(t), int(rid), KIND[kind], int(code)]
            if rid >= 0:
                r = s.robots[rid]
                shown = r.calculated_position if kind == "LOOK" else r.coordinates
                rec += [float(shown.x) if shown is not None else None,
                        float(shown.y) if shown is not None else None,
                        STATE[r.state] | (4 if r.frozen else 0) | (8 if r.terminated else 0)]
            events.append(rec)
    return {
        "name": name,
        "config": c,
        "initial": initial,
        "events": events,
        "final": {
            "event_count": count,
            "time": float(s.current_time),
            "ended": bool(s.terminate),
            "robots": [robot_record(r) for r in s.robots],
        },
    }


def main():
    out_dir = os.path.join(ROOT, "fixtures")
    os.makedirs(out_dir, exist_ok=True)
    for name, c in CASES.items():
        fx = generate(name, c)
        with open(os.path.join(out_dir, f"{name}.json"), "w") as f:
            json.dump(fx, f, separators=(",", ":"))
        fin = fx["final"]
        print(f"{name:22} events={fin['event_count']:>6} ended={fin['ended']} t={fin['time']:.2f}")


if __name__ == "__main__":
    main()
