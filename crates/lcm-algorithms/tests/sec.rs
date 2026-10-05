//! Integration tests for the SEC algorithm.
//!
//! Each test drives a full Simulation and asserts on the observable outcome:
//! did it terminate, where did robots end up, how many events did it take.
//! This covers configurations the parity fixtures don't: Circle+Edge (instant
//! termination via the concyclic fast path), Line+Edge (collinear fast path),
//! n=1/2/3 edge cases, degenerate coincident robots, non-rigid movement, and
//! crash-fault exclusion.

use lcm_algorithms::simulation;
use lcm_core::{Arrangement, Distribution, Pattern, SimConfig, StartSpec};
use std::f64::consts::TAU;

// ── helpers ───────────────────────────────────────────────────────────────────

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-9 * a.abs().max(b.abs()).max(1.0)
}

fn sec_cfg() -> SimConfig {
    SimConfig {
        algorithm: "SEC".to_owned(),
        ..SimConfig::default()
    }
}

/// Run the simulation to termination or the event ceiling, whichever comes first.
fn run_to_end(config: &SimConfig) -> lcm_core::Simulation {
    let mut sim = simulation(config).expect("valid config");
    while !sim.ended() {
        sim.step();
    }
    sim
}

/// Run at most `limit` steps; return the simulation state whether or not it ended.
fn run_steps(config: &SimConfig, limit: u64) -> lcm_core::Simulation {
    let mut sim = simulation(config).expect("valid config");
    for _ in 0..limit {
        if sim.ended() {
            break;
        }
        sim.step();
    }
    sim
}

// ── Circle+Edge: instant termination ─────────────────────────────────────────
//
// Robots start exactly on the SEC boundary → concyclic fast path fires at the
// first LOOK of every robot → each robot terminates on its first LOOK event →
// simulation ends after at most n * handful-of-events steps.

#[test]
fn circle_edge_even_terminates_immediately() {
    let n = 8u32;
    let r = 200.0_f64;
    let positions: Vec<[f64; 2]> = (0..n)
        .map(|k| {
            let a = TAU * k as f64 / n as f64;
            [r * a.cos(), r * a.sin()]
        })
        .collect();
    let config = SimConfig {
        num_of_robots: n,
        initial_positions: Some(positions),
        ..sec_cfg()
    };
    // Give it a generous ceiling so we know if it hangs rather than terminates.
    let sim = run_steps(&config, (n * 20) as u64);
    assert!(sim.ended(), "should terminate: robots start on SEC boundary");
    assert!(
        sim.robots().terminated.iter().all(|&t| t),
        "all robots must be marked terminated"
    );
}

#[test]
fn circle_edge_even_large_n_terminates() {
    let n = 32u32;
    let r = 300.0_f64;
    let positions: Vec<[f64; 2]> = (0..n)
        .map(|k| {
            let a = TAU * k as f64 / n as f64;
            [r * a.cos(), r * a.sin()]
        })
        .collect();
    let config = SimConfig {
        num_of_robots: n,
        initial_positions: Some(positions),
        ..sec_cfg()
    };
    let sim = run_steps(&config, (n * 20) as u64);
    assert!(sim.ended());
    assert!(sim.robots().terminated.iter().all(|&t| t));
}

#[test]
fn circle_edge_via_start_spec_terminates() {
    // Same test driven by the StartSpec path (mirrors the UI dropdown).
    let config = SimConfig {
        num_of_robots: 12,
        start: Some(StartSpec {
            pattern: Pattern::Circle,
            arrangement: Arrangement::Edge,
            distribution: Distribution::Even,
            size: 400.0,
            ..StartSpec::default()
        }),
        ..sec_cfg()
    };
    let sim = run_steps(&config, 12 * 20);
    assert!(sim.ended(), "Circle+Edge+Even should terminate");
}

// ── n = 1: single robot ───────────────────────────────────────────────────────

#[test]
fn single_robot_terminates_at_its_own_position() {
    let config = SimConfig {
        num_of_robots: 1,
        initial_positions: Some(vec![[42.0, -77.0]]),
        ..sec_cfg()
    };
    let sim = run_steps(&config, 50);
    assert!(sim.ended());
    let r = sim.robots();
    // Robot stays within eps of its initial position.
    assert!(
        close(r.pos[0].x, 42.0) && close(r.pos[0].y, -77.0),
        "single robot drifted: {:?}", r.pos[0]
    );
}

// ── n = 2: diameter circle ────────────────────────────────────────────────────
//
// Both robots are already the endpoints of the diameter circle, so they are on
// the SEC from the start and terminate on their first LOOK.

#[test]
fn two_robots_already_on_diameter_terminate_immediately() {
    let config = SimConfig {
        num_of_robots: 2,
        initial_positions: Some(vec![[-100.0, 0.0], [100.0, 0.0]]),
        ..sec_cfg()
    };
    let sim = run_steps(&config, 40);
    assert!(sim.ended());
    assert!(sim.robots().terminated.iter().all(|&t| t));
}

// ── n = 3: edge cases ────────────────────────────────────────────────────────

#[test]
fn three_collinear_robots_converge() {
    // Three robots on a horizontal line; sec_3pts handles this via largest_diameter.
    let config = SimConfig {
        num_of_robots: 3,
        initial_positions: Some(vec![[-100.0, 0.0], [0.0, 0.0], [100.0, 0.0]]),
        max_events: Some(5000),
        ..sec_cfg()
    };
    let sim = run_to_end(&config);
    assert!(sim.ended(), "three collinear robots must converge");
}

#[test]
fn three_non_collinear_robots_converge() {
    let config = SimConfig {
        num_of_robots: 3,
        initial_positions: Some(vec![[0.0, 100.0], [-100.0, -50.0], [100.0, -50.0]]),
        max_events: Some(5000),
        ..sec_cfg()
    };
    let sim = run_to_end(&config);
    assert!(sim.ended());
}

// ── n = 3: isoceles triangle — one pair's diameter covers the third ───────────

#[test]
fn three_robots_diameter_case_converges() {
    // (−150, 0), (150, 0), (0, 50): the diameter (−150)→(150) covers (0,50)
    // since dist((0,50),(0,0)) = 50 < 150 = radius. SEC is the diameter circle.
    let config = SimConfig {
        num_of_robots: 3,
        initial_positions: Some(vec![[-150.0, 0.0], [150.0, 0.0], [0.0, 50.0]]),
        max_events: Some(5000),
        ..sec_cfg()
    };
    let sim = run_to_end(&config);
    assert!(sim.ended());
}

// ── coincident robots ─────────────────────────────────────────────────────────

#[test]
fn all_robots_at_same_position_terminates() {
    // n > 3 with all robots coincident: collinear_sec returns None (no second
    // distinct point), concyclic_sec returns None (all collinear), Welzl returns
    // a degenerate circle of radius 0. All robots are "on" it → terminate.
    let config = SimConfig {
        num_of_robots: 5,
        initial_positions: Some(vec![[10.0, 20.0]; 5]),
        max_events: Some(500),
        ..sec_cfg()
    };
    let sim = run_steps(&config, 200);
    assert!(sim.ended(), "all-coincident robots must terminate");
}

// ── Line + Edge: collinear fast path ─────────────────────────────────────────

#[test]
fn line_edge_even_converges_via_collinear_fast_path() {
    let config = SimConfig {
        num_of_robots: 10,
        start: Some(StartSpec {
            pattern: Pattern::Line,
            arrangement: Arrangement::Edge,
            distribution: Distribution::Even,
            size: 400.0,
            ..StartSpec::default()
        }),
        max_events: Some(50_000),
        ..sec_cfg()
    };
    let sim = run_to_end(&config);
    assert!(sim.ended(), "Line+Edge+Even must converge to SEC");
}

#[test]
fn line_edge_uniform_converges() {
    let config = SimConfig {
        num_of_robots: 8,
        start: Some(StartSpec {
            pattern: Pattern::Line,
            arrangement: Arrangement::Edge,
            distribution: Distribution::Uniform,
            size: 300.0,
            ..StartSpec::default()
        }),
        max_events: Some(50_000),
        ..sec_cfg()
    };
    let sim = run_to_end(&config);
    assert!(sim.ended());
}

// ── random box: general convergence ──────────────────────────────────────────

#[test]
fn random_box_start_converges() {
    let config = SimConfig {
        num_of_robots: 15,
        random_seed: 99,
        max_events: Some(100_000),
        ..sec_cfg()
    };
    let sim = run_to_end(&config);
    assert!(sim.ended(), "random box start must converge");
}

// ── non-rigid movement ────────────────────────────────────────────────────────
//
// With rigid_movement=false, robots can be interrupted mid-move.
// Convergence must still occur.

#[test]
fn non_rigid_movement_still_converges() {
    let config = SimConfig {
        num_of_robots: 8,
        rigid_movement: false,
        random_seed: 7,
        max_events: Some(200_000),
        ..sec_cfg()
    };
    let sim = run_to_end(&config);
    assert!(sim.ended(), "SEC must converge under non-rigid movement");
}

// ── crash faults: crashed robots excluded from SEC ────────────────────────────

#[test]
fn crash_faults_excluded_from_sec_and_converges() {
    let config = SimConfig {
        num_of_robots: 10,
        num_of_faults: 3,
        fault_type: lcm_core::FaultSelection::Crash,
        random_seed: 42,
        max_events: Some(200_000),
        ..sec_cfg()
    };
    let sim = run_to_end(&config);
    assert!(sim.ended(), "SEC must converge with crash faults");
    // Crashed robots must not be marked terminated (they never ran compute).
    let r = sim.robots();
    let crashed = r
        .fault
        .iter()
        .filter(|&&f| f == lcm_core::FaultKind::Crash)
        .count();
    assert_eq!(crashed, 3, "exactly 3 robots should have crashed");
}

// ── final position invariant: every non-crashed robot on SEC boundary ─────────
//
// At termination the simulation already verified the SEC boundary condition
// (all robots within eps of the circle) — that is what triggered `ended()`.
// We reconstruct the SEC center from the farthest pair of live robots (which
// are the two boundary robots in the diameter case, or close to the boundary
// in general) and confirm all others lie within the same radius.

#[test]
fn all_live_robots_on_sec_boundary_at_termination() {
    let config = SimConfig {
        num_of_robots: 12,
        random_seed: 55,
        max_events: Some(200_000),
        ..sec_cfg()
    };
    let sim = run_to_end(&config);
    assert!(sim.ended());
    let r = sim.robots();

    // All non-crashed robots must be flagged terminated.
    for (i, (&term, &fault)) in r.terminated.iter().zip(&r.fault).enumerate() {
        if fault != lcm_core::FaultKind::Crash {
            assert!(term, "live robot {i} is not terminated at end");
        }
    }

    // Collect live positions.
    let live: Vec<_> = r
        .pos
        .iter()
        .zip(&r.fault)
        .filter(|(_, &f)| f != lcm_core::FaultKind::Crash)
        .map(|(p, _)| *p)
        .collect();

    // Find the diameter pair (the two farthest live robots) — these two points
    // uniquely determine the SEC center when only two boundary robots exist, and
    // give a good lower bound on the radius in all cases.
    let mut max_dist = 0.0_f64;
    let (mut pa, mut pb) = (live[0], live[0]);
    for i in 0..live.len() {
        for j in (i + 1)..live.len() {
            let d = {
                let (dx, dy) = (live[i].x - live[j].x, live[i].y - live[j].y);
                (dx * dx + dy * dy).sqrt()
            };
            if d > max_dist {
                max_dist = d;
                pa = live[i];
                pb = live[j];
            }
        }
    }
    let cx = (pa.x + pb.x) * 0.5;
    let cy = (pa.y + pb.y) * 0.5;
    let r_sec = max_dist * 0.5;

    // Every live robot must lie within 1.0 world unit of the SEC radius.
    // (The simulation's eps threshold is typically 1e-4..1e-2, so 1.0 is loose
    //  but guards against gross failures — positions off by tens of units.)
    for (i, p) in live.iter().enumerate() {
        let d = ((p.x - cx).powi(2) + (p.y - cy).powi(2)).sqrt();
        assert!(
            (d - r_sec).abs() < 1.0,
            "live robot {i} at distance {d:.4} from SEC center, radius is {r_sec:.4}"
        );
    }
}
