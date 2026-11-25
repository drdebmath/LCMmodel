//! Izumi et al. 2012: two robots with unreliable compasses gather when the
//! compass error is below the bound of the rule set used.

use core::f64::consts::{FRAC_PI_2, PI};
use lcm_algorithms::compass_gathering::{action, Action, Rules};
use lcm_core::geom::dist;
use lcm_core::{
    Budget, ConfigError, Frames, Rng, SchedulerKind, SimConfig, Simulation, StopReason,
};

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
/// to 3.1e7).
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

// ---- Review of #17, fix B2.

fn two(frames: Frames, scheduler: SchedulerKind) -> SimConfig {
    SimConfig {
        algorithm: "CompassGathering".to_owned(),
        num_of_robots: 2,
        initial_positions: Some(vec![[-60.0, 25.0], [80.0, -40.0]]),
        scheduler,
        frames,
        robot_speeds: 20.0,
        ..SimConfig::default()
    }
}

/// Rotated and disoriented frames turn each robot's axes by an unbounded
/// random angle at every Look (and disoriented ones mirror them): no bounded
/// compass error, no shared chirality, so the paper does not apply.
#[test]
fn rotated_and_disoriented_frames_are_refused() {
    for frames in [Frames::Rotated, Frames::Disoriented] {
        match lcm_algorithms::simulation(&two(frames, SchedulerKind::Ssync)) {
            Err(ConfigError::Unsupported(m)) => assert!(m.contains("Robots' axes"), "{m}"),
            Err(e) => panic!("{frames:?}: expected Unsupported, got {e}"),
            Ok(_) => panic!("{frames:?} should be refused"),
        }
    }
    for frames in [Frames::Global, Frames::Compass] {
        assert!(lcm_algorithms::simulation(&two(frames, SchedulerKind::Ssync)).is_ok());
    }
    let exact = lcm_algorithms::notes(
        "CompassGathering",
        &two(Frames::Global, SchedulerKind::Ssync),
    );
    assert_eq!(exact.len(), 1);
    assert!(exact[0].contains("exact"), "{exact:?}");
}

/// The next angle above `a`.
fn above(a: f64) -> f64 {
    f64::from_bits(a.to_bits() + 1)
}

/// The sector boundaries of the module docs (Izumi et al., Algorithms 1-3),
/// each tested exactly on the boundary and just past it: which side is open
/// or closed decides what a robot does when its compass reads exactly that.
#[test]
fn every_rule_set_is_exact_at_its_boundaries() {
    let phi = PI / 9.0; // 20°
    let ss = |a| action(Rules::StaticCompass, phi, a);
    // A_SS: (0, π] go, (π, 3π/2 + φ] west, otherwise stay.
    assert_eq!(ss(0.0), Action::Stay);
    assert_eq!(ss(above(0.0)), Action::Go);
    assert_eq!(ss(PI), Action::Go);
    assert_eq!(ss(above(PI)), Action::West);
    assert_eq!(ss(3.0 * FRAC_PI_2 + phi), Action::West);
    assert_eq!(ss(above(3.0 * FRAC_PI_2 + phi)), Action::Stay);

    let sd = |a| action(Rules::DynamicSynchronous, phi, a);
    // A_SD: (π/2 + φ, 3π/2 - φ] go, (-π/2 + φ, π/2 - φ] stay, otherwise turn.
    assert_eq!(sd(FRAC_PI_2 + phi), Action::Turn);
    assert_eq!(sd(above(FRAC_PI_2 + phi)), Action::Go);
    assert_eq!(sd(3.0 * FRAC_PI_2 - phi), Action::Go);
    assert_eq!(sd(above(3.0 * FRAC_PI_2 - phi)), Action::Turn);
    assert_eq!(sd(FRAC_PI_2 - phi), Action::Stay);
    assert_eq!(sd(above(FRAC_PI_2 - phi)), Action::Turn);
    assert_eq!(sd(3.0 * FRAC_PI_2 + phi), Action::Turn);
    assert_eq!(sd(above(3.0 * FRAC_PI_2 + phi)), Action::Stay);
    assert_eq!(sd(0.0), Action::Stay);

    let ad = |a| action(Rules::DynamicAsynchronous, phi, a);
    // A_AD: [2π/3 + φ, 3π/2) go, [3π/2, 2π) or [0, π/3 - φ] stay, otherwise turn.
    assert_eq!(ad(2.0 * PI / 3.0 + phi), Action::Go);
    assert_eq!(
        ad(f64::from_bits((2.0 * PI / 3.0 + phi).to_bits() - 1)),
        Action::Turn
    );
    assert_eq!(ad(3.0 * FRAC_PI_2), Action::Stay);
    assert_eq!(
        ad(f64::from_bits((3.0 * FRAC_PI_2).to_bits() - 1)),
        Action::Go
    );
    assert_eq!(ad(PI / 3.0 - phi), Action::Stay);
    assert_eq!(ad(above(PI / 3.0 - phi)), Action::Turn);
    assert_eq!(ad(0.0), Action::Stay);
}

fn play(config: &SimConfig) -> (StopReason, Simulation) {
    let mut sim = lcm_algorithms::simulation(config).unwrap();
    let reason = sim.advance(Budget {
        max_events: 10_000_000,
        until_time: f64::INFINITY,
    });
    (reason, sim)
}

/// Under the sequential and SSYNC schedulers a robot that finds the other on
/// its point terminates and the run ends; under async it must not (Remark 2
/// of the paper: the other may still be about to move), so the robots stay
/// together until the time limit.
#[test]
fn robots_terminate_in_rounds_but_not_under_async() {
    for (fixed, error) in [(true, 60.0), (false, 30.0)] {
        for scheduler in [SchedulerKind::Sequential, SchedulerKind::Ssync] {
            let config = SimConfig {
                compass_error: error,
                compass_static: fixed,
                max_time: Some(100_000.0),
                ..two(Frames::Compass, scheduler)
            };
            let (reason, sim) = play(&config);
            assert_eq!(reason, StopReason::Ended, "{scheduler:?} fixed {fixed}");
            assert!(sim.robots().terminated.iter().all(|&t| t));
        }
        let config = SimConfig {
            compass_error: error.min(25.0),
            compass_static: fixed,
            max_time: Some(20_000.0),
            ..two(Frames::Compass, SchedulerKind::Async)
        };
        let (reason, sim) = play(&config);
        assert_eq!(reason, StopReason::MaxTime, "async fixed {fixed}");
        let p = &sim.robots().pos;
        assert!(dist(p[0], p[1]) < 1e-5, "async fixed {fixed}: gathered");
        assert!(sim.robots().terminated.iter().all(|&t| !t));
    }
}

/// k-ASYNC counts as robots acting on their own clocks: a dynamic compass
/// takes A_AD, and the robots gather without terminating.
#[test]
fn k_async_takes_the_asynchronous_rules() {
    for seed in 0..10 {
        let config = SimConfig {
            compass_error: 20.0,
            compass_static: false,
            k_async: 2,
            random_seed: seed,
            max_time: Some(20_000.0),
            ..two(Frames::Compass, SchedulerKind::KAsync)
        };
        let (reason, sim) = play(&config);
        assert_eq!(reason, StopReason::MaxTime, "seed {seed}");
        let p = &sim.robots().pos;
        assert!(dist(p[0], p[1]) < 1e-5, "seed {seed}: {:?}", p);
        assert!(sim.robots().terminated.iter().all(|&t| !t));
    }
    let notes = lcm_algorithms::notes(
        "CompassGathering",
        &SimConfig {
            compass_error: 20.0,
            compass_static: false,
            ..two(Frames::Compass, SchedulerKind::KAsync)
        },
    );
    assert!(notes.iter().any(|n| n.contains("Remark 2")), "{notes:?}");
}
