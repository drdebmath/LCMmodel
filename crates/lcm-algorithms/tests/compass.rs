//! Unreliable compasses (Izumi et al. 2012): `frames: "compass"` turns each
//! robot's axes by a deviation of at most `compass_error` degrees, drawn once
//! per robot (static) or at every Look (dynamic).

use lcm_core::{Budget, Frames, SchedulerKind, SimConfig, Simulation, TurnGap};

fn config(frames: Frames, error: f64, fixed: bool) -> SimConfig {
    SimConfig {
        algorithm: "Gathering".to_owned(),
        num_of_robots: 6,
        initial_positions: Some(vec![
            [0.0, 0.0],
            [40.0, 5.0],
            [-30.0, 25.0],
            [10.0, -45.0],
            [55.0, 60.0],
            [-50.0, -20.0],
        ]),
        scheduler: SchedulerKind::Sequential,
        turn_gap: TurnGap::None,
        frames,
        compass_error: error,
        compass_static: fixed,
        ..SimConfig::default()
    }
}

fn sim(c: &SimConfig) -> Simulation {
    lcm_algorithms::simulation(c).unwrap()
}

#[test]
fn deviations_stay_within_the_error_and_a_static_compass_keeps_its_own() {
    let mut s = sim(&config(Frames::Compass, 30.0, true));
    let first = s.compass_deviations().to_vec();
    assert_eq!(first.len(), 6);
    let limit = 30f64.to_radians();
    assert!(first.iter().all(|d| d.abs() <= limit));
    assert!(first.iter().any(|d| d.abs() > 1e-6), "deviations are drawn");
    for _ in 0..200 {
        s.step();
    }
    assert_eq!(
        s.compass_deviations(),
        first.as_slice(),
        "static: never redrawn"
    );
}

#[test]
fn a_dynamic_compass_draws_a_new_deviation_at_each_look() {
    let mut s = sim(&config(Frames::Compass, 30.0, false));
    let first = s.compass_deviations().to_vec();
    for _ in 0..200 {
        s.step();
    }
    let later = s.compass_deviations();
    assert!(later.iter().all(|d| d.abs() <= 30f64.to_radians()));
    assert_ne!(later, first.as_slice());
}

#[test]
fn an_exact_compass_is_the_global_frame_up_to_rounding() {
    let (mut a, mut b) = (
        sim(&config(Frames::Global, 0.0, true)),
        sim(&config(Frames::Compass, 0.0, true)),
    );
    let budget = Budget {
        max_events: 100_000,
        until_time: f64::INFINITY,
    };
    // The same run, up to the rounding of centring the frame on the robot.
    assert_eq!(a.advance(budget), b.advance(budget));
    assert_eq!(a.event_count(), b.event_count());
    for (p, q) in a.robots().pos.iter().zip(&b.robots().pos) {
        assert!((p.x - q.x).abs() < 1e-9 && (p.y - q.y).abs() < 1e-9);
    }
}

#[test]
fn a_frame_invariant_algorithm_ignores_the_compass() {
    let budget = Budget {
        max_events: 100_000,
        until_time: f64::INFINITY,
    };
    let mut a = sim(&config(Frames::Global, 0.0, true));
    let mut b = sim(&config(Frames::Compass, 170.0, false));
    a.advance(budget);
    b.advance(budget);
    for (p, q) in a.robots().pos.iter().zip(&b.robots().pos) {
        assert!((p.x - q.x).abs() < 1e-6 && (p.y - q.y).abs() < 1e-6);
    }
}

#[test]
fn the_error_must_be_an_angle() {
    assert!(config(Frames::Compass, 200.0, true).validate().is_err());
    assert!(config(Frames::Compass, -1.0, true).validate().is_err());
}
