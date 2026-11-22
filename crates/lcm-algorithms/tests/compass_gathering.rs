//! Izumi et al. 2012: two robots with unreliable compasses gather when the
//! compass error is below the bound of the rule set used.

use lcm_core::geom::dist;
use lcm_core::{Budget, Frames, Rng, SchedulerKind, SimConfig};

fn run(scheduler: SchedulerKind, error: f64, fixed: bool, seed: u64, rigid: bool) -> f64 {
    run_for(scheduler, error, fixed, seed, rigid, 20_000.0)
}

fn run_for(
    scheduler: SchedulerKind,
    error: f64,
    fixed: bool,
    seed: u64,
    rigid: bool,
    time: f64,
) -> f64 {
    let mut rng = Rng::new(seed);
    let mut point = || [rng.uniform(-200.0, 200.0), rng.uniform(-200.0, 200.0)];
    let config = SimConfig {
        algorithm: "CompassGathering".to_owned(),
        num_of_robots: 2,
        initial_positions: Some(vec![point(), point()]),
        random_seed: seed,
        scheduler,
        frames: Frames::Compass,
        compass_error: error,
        compass_static: fixed,
        rigid_movement: rigid,
        delta: (!rigid).then_some(5.0),
        robot_speeds: 20.0,
        // Near the bound A_AD can need tens of thousands of time units: its
        // rotate rule turns by almost 180°, nearly doubling the gap first.
        max_time: Some(time),
        ..SimConfig::default()
    };
    let mut sim = lcm_algorithms::simulation(&config).unwrap();
    sim.advance(Budget {
        max_events: 1_000_000_000,
        until_time: f64::INFINITY,
    });
    let p = &sim.robots().pos;
    dist(p[0], p[1])
}

#[test]
fn static_compass_gathers_below_ninety_degrees() {
    for scheduler in [
        SchedulerKind::Ssync,
        SchedulerKind::Sequential,
        SchedulerKind::Async,
    ] {
        for seed in 0..20 {
            for rigid in [true, false] {
                let gap = run(scheduler, 85.0, true, seed, rigid);
                assert!(gap < 1e-5, "{scheduler:?} seed {seed} rigid {rigid}: {gap}");
            }
        }
    }
}

#[test]
fn dynamic_compass_synchronous_gathers_below_forty_five_degrees() {
    for scheduler in [
        SchedulerKind::Ssync,
        SchedulerKind::Sequential,
        SchedulerKind::Fsync,
    ] {
        for seed in 0..20 {
            let gap = run(scheduler, 44.0, false, seed, false);
            assert!(gap < 1e-5, "{scheduler:?} seed {seed}: {gap}");
        }
    }
}

#[test]
fn dynamic_compass_asynchronous_gathers_at_twenty_five_degrees() {
    // Theorem 6 claims every phi < 30 degrees; see the ignored test below for
    // what happens at 29.
    for seed in 0..10 {
        for rigid in [true, false] {
            let gap = run(SchedulerKind::Async, 25.0, false, seed, rigid);
            assert!(gap < 1e-5, "rigid {rigid}, seed {seed}: {gap}");
        }
    }
}

/// Theorem 6: A_AD gathers two asynchronous robots with dynamic compasses for
/// every phi < 30 degrees, rigid or not. Near the bound it is very slow: the
/// rotate rule turns by 2*pi/3 + 2*phi (178 degrees at 29), which nearly
/// doubles the gap, and each move then takes gap / speed time. These two runs
/// looked like counterexamples within 500,000 time units; they gather at
/// 2.1e6 (rigid seed 8, gap up to 6.8e5) and 5.5e7 (non-rigid seed 4, gap up
/// to 3.1e7). See docs/extras.md, Findings.
#[test]
#[ignore = "about 15 s in release, far longer in debug: run with --ignored --release"]
fn a_ad_at_29_degrees_is_slow_but_gathers() {
    assert!(
        run_for(SchedulerKind::Async, 29.0, false, 8, true, 3e6) < 1e-5,
        "rigid seed 8"
    );
    assert!(
        run_for(SchedulerKind::Async, 29.0, false, 4, false, 6e7) < 1e-5,
        "non-rigid seed 4"
    );
}

#[test]
fn only_two_robots() {
    let config = SimConfig {
        algorithm: "CompassGathering".to_owned(),
        num_of_robots: 3,
        ..SimConfig::default()
    };
    assert!(lcm_algorithms::simulation(&config).is_err());
}
