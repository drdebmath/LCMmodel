//! Integration tests for Uniform Circle Formation.
//!
//! Each test drives a full `Simulation` and asserts on the observable outcome:
//! the swarm converges to a regular polygon (everyone on a common circle at
//! near-uniform angular spacing). Because the algorithm uses no randomness and
//! reads only the snapshot, it is scheduler-agnostic — every convergence test
//! runs under BOTH the async and the sequential scheduler.

use lcm_algorithms::simulation;
use lcm_core::{
    ActivationOrder, FaultKind, FaultSelection, RobotState, SchedulerKind, SimConfig, Simulation,
    TurnGap,
};
use std::f64::consts::TAU;

// ── helpers ─────────────────────────────────────────────────────────────────

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-9 * a.abs().max(b.abs()).max(1.0)
}

/// A base async config for UCF with a generous event ceiling.
fn ucf_cfg() -> SimConfig {
    SimConfig {
        algorithm: "CircleFormation".to_owned(),
        max_events: Some(500_000),
        ..SimConfig::default()
    }
}

/// The same config under the sequential scheduler (round-robin, random gap).
fn sequential(mut config: SimConfig) -> SimConfig {
    config.scheduler = SchedulerKind::Sequential;
    config.activation_order = ActivationOrder::RoundRobin;
    config.turn_gap = TurnGap::Random;
    // Sequential turns are cheaper per event; a turn ceiling guards against hangs.
    config.max_turns = Some(200_000);
    config
}

/// Run to termination or the event ceiling, whichever comes first.
fn run_to_end(config: &SimConfig) -> Simulation {
    let mut sim = simulation(config).expect("valid config");
    while !sim.ended() {
        sim.step();
    }
    sim
}

/// Positions of the robots that are neither crashed nor terminated-and-moved.
/// Here we take all non-crashed robots (the live swarm).
fn live_positions(sim: &Simulation) -> Vec<lcm_core::Point> {
    let r = sim.robots();
    (0..r.len())
        .filter(|&i| r.state[i] != RobotState::Crash && r.fault[i] != FaultKind::Crash)
        .map(|i| r.pos[i])
        .collect()
}

/// Assert the live robots sit on a common circle at near-uniform angular gaps.
/// `gap_tol` is the fraction of the ideal gap the largest deviation may reach
/// (the algorithm itself terminates below 0.06; we allow a little slack).
fn assert_uniform_circle(sim: &Simulation, gap_tol: f64) {
    let pts = live_positions(sim);
    let n = pts.len();
    assert!(n >= 2, "need at least two live robots, got {n}");

    // Reference circle from the final positions.
    let inv_n = 1.0 / n as f64;
    let ox = pts.iter().map(|p| p.x).sum::<f64>() * inv_n;
    let oy = pts.iter().map(|p| p.y).sum::<f64>() * inv_n;
    let rad = pts.iter().map(|p| (p.x - ox).hypot(p.y - oy)).sum::<f64>() * inv_n;
    assert!(rad > 1e-6, "degenerate radius {rad}");

    // Everyone on the circle (within 2% of the radius, matching _circle_terminal).
    for p in &pts {
        let d = (p.x - ox).hypot(p.y - oy);
        assert!(
            (d - rad).abs() <= rad * 2e-2,
            "robot off the circle: d={d}, rad={rad}"
        );
    }

    // Angular gaps near the ideal 2π/n.
    let mut angs: Vec<f64> = pts
        .iter()
        .map(|p| (p.y - oy).atan2(p.x - ox).rem_euclid(TAU))
        .collect();
    angs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let target_gap = TAU / n as f64;
    let max_dev = (0..n)
        .map(|i| ((angs[(i + 1) % n] - angs[i]).rem_euclid(TAU) - target_gap).abs())
        .fold(0.0_f64, f64::max);
    assert!(
        max_dev < target_gap * gap_tol,
        "gaps not uniform: max deviation {max_dev} >= {}",
        target_gap * gap_tol
    );
}

/// Run the same config under both schedulers and assert both converge to a
/// uniform circle. This is the core "switch schedulers anytime" guarantee.
fn converges_on_both_schedulers(base: SimConfig) {
    let async_sim = run_to_end(&base);
    assert!(async_sim.ended(), "async did not converge: {base:?}");
    assert_uniform_circle(&async_sim, 0.08);

    let seq_sim = run_to_end(&sequential(base.clone()));
    assert!(seq_sim.ended(), "sequential did not converge: {base:?}");
    assert!(seq_sim.epochs_completed() > 0, "no epochs ran: {base:?}");
    assert_uniform_circle(&seq_sim, 0.08);
}

// ── convergence on both schedulers ──────────────────────────────────────────

#[test]
fn random_start_converges_on_both_schedulers() {
    for (n, seed) in [(8u32, 1u64), (12, 7), (20, 42)] {
        let config = SimConfig {
            num_of_robots: n,
            random_seed: seed,
            ..ucf_cfg()
        };
        converges_on_both_schedulers(config);
    }
}

#[test]
fn non_rigid_movement_converges_on_both_schedulers() {
    // Robots can be interrupted mid-move; Phase 1 re-projects them next look.
    let config = SimConfig {
        num_of_robots: 10,
        random_seed: 3,
        rigid_movement: false,
        delta: Some(20.0),
        ..ucf_cfg()
    };
    converges_on_both_schedulers(config);
}

// ── already-formed starts terminate quickly ─────────────────────────────────

#[test]
fn a_regular_polygon_start_terminates_immediately() {
    for scheduler in [SchedulerKind::Async, SchedulerKind::Sequential] {
        let n = 10u32;
        let r = 250.0_f64;
        let positions: Vec<[f64; 2]> = (0..n)
            .map(|k| {
                let a = TAU * k as f64 / n as f64;
                [r * a.cos(), r * a.sin()]
            })
            .collect();
        let mut config = SimConfig {
            num_of_robots: n,
            initial_positions: Some(positions),
            ..ucf_cfg()
        };
        if scheduler == SchedulerKind::Sequential {
            config = sequential(config);
        }
        let sim = run_to_end(&config);
        assert!(
            sim.ended(),
            "regular polygon must terminate ({scheduler:?})"
        );
        assert!(
            sim.robots().terminated.iter().all(|&t| t),
            "all robots must terminate ({scheduler:?})"
        );
        // It was already uniform, so it should not have had to rearrange much.
        assert_uniform_circle(&sim, 0.07);
    }
}

// ── crash faults: crashed robots excluded from the formation ─────────────────

#[test]
fn crash_faults_excluded_and_live_robots_still_form_a_circle() {
    for seq in [false, true] {
        let mut config = SimConfig {
            num_of_robots: 12,
            num_of_faults: 3,
            fault_type: FaultSelection::Crash,
            random_seed: 99,
            ..ucf_cfg()
        };
        if seq {
            config = sequential(config);
        }
        let sim = run_to_end(&config);
        assert!(sim.ended(), "must converge with crash faults (seq={seq})");

        let r = sim.robots();
        // Every live (non-crashed) robot is terminated.
        for i in 0..r.len() {
            if r.fault[i] != FaultKind::Crash {
                assert!(r.terminated[i], "live robot {i} not terminated (seq={seq})");
            }
        }
        // And the live robots form a uniform circle among themselves.
        assert_uniform_circle(&sim, 0.08);
    }
}

// ── registry / dropdown wiring ──────────────────────────────────────────────

#[test]
fn circle_formation_is_registered_under_its_ui_key() {
    let entry = lcm_algorithms::REGISTRY
        .iter()
        .find(|e| e.key == "CircleFormation")
        .expect("CircleFormation must be registered");
    assert_eq!(entry.name, "Uniform Circle Formation");
    // Builds a valid simulation from a plain config.
    let config = SimConfig {
        algorithm: "CircleFormation".to_owned(),
        ..SimConfig::default()
    };
    assert!(simulation(&config).is_ok());
}

// ── sanity: the reference circle is the centroid + mean radius ───────────────

#[test]
fn two_robots_meet_on_their_shared_diameter() {
    // n = 2: centroid is the midpoint, mean radius is half their separation, so
    // both robots are already exactly on the reference circle -> immediate finish.
    let config = SimConfig {
        num_of_robots: 2,
        initial_positions: Some(vec![[-30.0, 0.0], [30.0, 0.0]]),
        ..ucf_cfg()
    };
    let sim = run_to_end(&config);
    assert!(sim.ended());
    let pts = live_positions(&sim);
    let sep = (pts[0].x - pts[1].x).hypot(pts[0].y - pts[1].y);
    assert!(close(sep, 60.0), "separation changed: {sep}");
}
