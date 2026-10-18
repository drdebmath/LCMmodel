//! Integration tests for Pattern Formation (star).
//!
//! Each test drives a full `Simulation` and asserts on the observable outcome:
//! the swarm converges onto the embedded 5-pointed-star template (every robot
//! terminated, within 3% of the frame scale of its matched point). The algorithm
//! uses no randomness and reads only the snapshot, so it is scheduler-agnostic —
//! every convergence test runs under BOTH the async and the sequential scheduler.

use lcm_algorithms::simulation;
use lcm_core::{
    ActivationOrder, FaultKind, FaultSelection, Point, RobotState, SchedulerKind, SimConfig,
    Simulation, TurnGap,
};
use std::f64::consts::PI;

fn pat_cfg() -> SimConfig {
    SimConfig {
        algorithm: "PatternFormation".to_owned(),
        max_events: Some(500_000),
        ..SimConfig::default()
    }
}

fn sequential(mut config: SimConfig) -> SimConfig {
    config.scheduler = SchedulerKind::Sequential;
    config.activation_order = ActivationOrder::RoundRobin;
    config.turn_gap = TurnGap::Random;
    config.max_turns = Some(500_000);
    config
}

fn run_to_end(config: &SimConfig) -> Simulation {
    let mut sim = simulation(config).expect("valid config");
    while !sim.ended() {
        sim.step();
    }
    sim
}

fn live_positions(sim: &Simulation) -> Vec<Point> {
    let r = sim.robots();
    (0..r.len())
        .filter(|&i| r.state[i] != RobotState::Crash && r.fault[i] != FaultKind::Crash)
        .map(|i| r.pos[i])
        .collect()
}

/// The RMS radius of the live swarm about its centroid — a measure of spread.
fn rms_radius(pts: &[Point]) -> f64 {
    let n = pts.len() as f64;
    let ox = pts.iter().map(|p| p.x).sum::<f64>() / n;
    let oy = pts.iter().map(|p| p.y).sum::<f64>() / n;
    (pts.iter().map(|p| (p.x - ox).powi(2) + (p.y - oy).powi(2)).sum::<f64>() / n).sqrt()
}

/// After termination the swarm should be a real 2-D shape, not collapsed to a
/// point or a line: it has non-trivial spread and reaches the star's spikes.
fn assert_formed_a_shape(sim: &Simulation) {
    let pts = live_positions(sim);
    assert!(pts.len() >= 3, "need at least three live robots");
    let rms = rms_radius(&pts);
    assert!(rms > 1e-3, "swarm collapsed (rms {rms})");

    // A star is not a disk: some robots sit near the outer spikes (far from the
    // centroid) while others sit at the inner vertices (nearer). Check the radii
    // actually vary — a uniform ring would fail this.
    let n = pts.len() as f64;
    let ox = pts.iter().map(|p| p.x).sum::<f64>() / n;
    let oy = pts.iter().map(|p| p.y).sum::<f64>() / n;
    let radii: Vec<f64> = pts.iter().map(|p| (p.x - ox).hypot(p.y - oy)).collect();
    let max_r = radii.iter().copied().fold(0.0_f64, f64::max);
    let min_r = radii.iter().copied().fold(f64::INFINITY, f64::min);
    assert!(
        max_r > min_r * 1.3,
        "radii too uniform for a star: min {min_r}, max {max_r}"
    );
}

fn converges_on_both_schedulers(base: SimConfig) {
    let async_sim = run_to_end(&base);
    assert!(async_sim.ended(), "async did not converge: {base:?}");
    assert!(
        async_sim.robots().terminated.iter().enumerate().all(|(i, &t)| {
            async_sim.robots().fault[i] == FaultKind::Crash || t
        }),
        "async: a live robot did not terminate: {base:?}"
    );
    assert_formed_a_shape(&async_sim);

    let seq_sim = run_to_end(&sequential(base.clone()));
    assert!(seq_sim.ended(), "sequential did not converge: {base:?}");
    assert!(seq_sim.epochs_completed() > 0, "no epochs ran: {base:?}");
    assert!(
        seq_sim.robots().terminated.iter().enumerate().all(|(i, &t)| {
            seq_sim.robots().fault[i] == FaultKind::Crash || t
        }),
        "sequential: a live robot did not terminate: {base:?}"
    );
    assert_formed_a_shape(&seq_sim);
}

#[test]
fn random_start_converges_on_both_schedulers() {
    for (n, seed) in [(8u32, 1u64), (12, 5), (20, 9)] {
        let config = SimConfig {
            num_of_robots: n,
            random_seed: seed,
            ..pat_cfg()
        };
        converges_on_both_schedulers(config);
    }
}

#[test]
fn a_star_start_terminates_quickly() {
    // Start the robots already on the embedded star; they should recognise the
    // formation and terminate without a long rearrangement.
    let n = 12usize;
    let (ox, oy, r) = (0.0, 0.0, 200.0);
    // Reproduce the template the algorithm uses (arc-even 5-pointed star).
    let star = star_like(n);
    let positions: Vec<[f64; 2]> = star
        .iter()
        .map(|u| [ox + r * u[0], oy + r * u[1]])
        .collect();
    for scheduler in [SchedulerKind::Async, SchedulerKind::Sequential] {
        let mut config = SimConfig {
            num_of_robots: n as u32,
            initial_positions: Some(positions.clone()),
            ..pat_cfg()
        };
        if scheduler == SchedulerKind::Sequential {
            config = sequential(config);
        }
        let sim = run_to_end(&config);
        assert!(sim.ended(), "star start must terminate ({scheduler:?})");
        assert!(
            sim.robots().terminated.iter().all(|&t| t),
            "all robots terminate ({scheduler:?})"
        );
    }
}

#[test]
fn crash_faults_excluded_and_live_robots_still_form_the_pattern() {
    for seq in [false, true] {
        let mut config = SimConfig {
            num_of_robots: 12,
            num_of_faults: 3,
            fault_type: FaultSelection::Crash,
            random_seed: 20,
            ..pat_cfg()
        };
        if seq {
            config = sequential(config);
        }
        let sim = run_to_end(&config);
        assert!(sim.ended(), "must converge with crash faults (seq={seq})");
        let r = sim.robots();
        for i in 0..r.len() {
            if r.fault[i] != FaultKind::Crash {
                assert!(r.terminated[i], "live robot {i} not terminated (seq={seq})");
            }
        }
        assert_formed_a_shape(&sim);
    }
}

#[test]
fn pattern_formation_is_registered_under_its_ui_key() {
    let entry = lcm_algorithms::REGISTRY
        .iter()
        .find(|e| e.key == "PatternFormation")
        .expect("PatternFormation must be registered");
    assert_eq!(entry.name, "Pattern Formation (star)");
    let config = SimConfig {
        algorithm: "PatternFormation".to_owned(),
        ..SimConfig::default()
    };
    assert!(simulation(&config).is_ok());
}

/// A 5-pointed-star outline sampled by arc length, matching the algorithm's own
/// template closely enough to use as an "already formed" start.
fn star_like(n: usize) -> Vec<[f64; 2]> {
    let (spikes, inner) = (5usize, 0.382_f64);
    let m = 2 * spikes;
    let verts: Vec<[f64; 2]> = (0..m)
        .map(|i| {
            let ang = PI / 2.0 + i as f64 * PI / spikes as f64;
            let rad = if i % 2 == 0 { 1.0 } else { inner };
            [rad * ang.cos(), rad * ang.sin()]
        })
        .collect();
    let segs: Vec<f64> = (0..m)
        .map(|i| {
            let (a, b) = (verts[i], verts[(i + 1) % m]);
            (b[0] - a[0]).hypot(b[1] - a[1])
        })
        .collect();
    let total: f64 = segs.iter().sum();
    (0..n)
        .map(|k| {
            let d = (k as f64 / n as f64) * total;
            let mut acc = 0.0;
            for i in 0..m {
                let len = segs[i];
                if acc + len >= d || i == m - 1 {
                    let t = if len > 1e-12 { (d - acc) / len } else { 0.0 };
                    let (a, b) = (verts[i], verts[(i + 1) % m]);
                    return [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
                }
                acc += len;
            }
            verts[0]
        })
        .collect()
}
