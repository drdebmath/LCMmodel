//! The inspector's sentences: what a robot would see and the rule it would
//! apply if it looked now, and that asking (or previewing its destination)
//! changes nothing about the run.

use lcm_core::{Frames, MultiplicityModel, SchedulerKind, SimConfig, Simulation};

fn sim(algorithm: &str, positions: &[[f64; 2]], tweak: impl FnOnce(&mut SimConfig)) -> Simulation {
    let mut config = SimConfig {
        algorithm: algorithm.to_owned(),
        num_of_robots: u32::try_from(positions.len()).unwrap(),
        initial_positions: Some(positions.to_vec()),
        ..SimConfig::default()
    };
    tweak(&mut config);
    lcm_algorithms::simulation(&config).unwrap()
}

fn says(text: Option<String>, parts: &[&str]) {
    let text = text.expect("an explanation");
    for part in parts {
        assert!(text.contains(part), "{part:?} not in {text:?}");
    }
}

#[test]
fn gathering_explains_its_rule() {
    let mut s = sim("Gathering", &[[0.0, 0.0], [10.0, 0.0]], |_| {});
    says(
        s.explain(0),
        &["2 robots", "centre of gravity, 5 units away"],
    );
}

/// A run whose compasses draw a new deviation at every Look.
fn dynamic_compass() -> Simulation {
    sim(
        "Gathering",
        &[[-120.0, 40.0], [110.0, -60.0], [30.0, 90.0]],
        |c| {
            c.frames = Frames::Compass;
            c.compass_error = 40.0;
            c.compass_static = false;
            c.scheduler = SchedulerKind::Ssync;
        },
    )
}

#[test]
fn explaining_changes_nothing_about_the_run() {
    let (mut plain, mut asked) = (dynamic_compass(), dynamic_compass());
    for _ in 0..2000 {
        plain.step();
        for i in 0..3 {
            asked.explain(i);
        }
        asked.step();
    }
    assert_eq!(plain.robots().pos, asked.robots().pos);
    assert_eq!(plain.compass_deviations(), asked.compass_deviations());
}

/// `preview_target` draws a deviation for its own Look; the robot's stored
/// deviation must be the same afterwards (review of #17, fix A2).
#[test]
fn previewing_a_target_keeps_a_dynamic_compass() {
    let mut s = dynamic_compass();
    for _ in 0..50 {
        s.step();
    }
    for i in 0..3 {
        let before = s.compass_deviations()[i];
        assert!(s.preview_target(i).is_some());
        assert_eq!(before, s.compass_deviations()[i], "robot {i}");
    }
}

#[test]
fn previewing_changes_nothing_about_the_run() {
    let (mut plain, mut asked) = (dynamic_compass(), dynamic_compass());
    for _ in 0..2000 {
        plain.step();
        for i in 0..3 {
            let _ = asked.preview_target(i);
        }
        asked.step();
    }
    assert_eq!(plain.robots().pos, asked.robots().pos);
    assert_eq!(plain.compass_deviations(), asked.compass_deviations());
}

#[test]
fn sq_gathering_names_kappa_b_and_the_rule() {
    let mut s = sim("SqGathering", &[[0.0, 0.0], [0.0, 0.0], [10.0, 0.0]], |c| {
        c.scheduler = SchedulerKind::Sequential;
    });
    says(
        s.explain(2),
        &[
            "2 occupied points",
            "κ = 1",
            "alone (b = 0)",
            "move to the multiplicity point",
            "moves 10 units",
        ],
    );
    says(s.explain(0), &["on a stack (b = 1)", "do not move"]);
}

#[test]
fn compass_gathering_names_the_bearing_and_what_it_does() {
    let mut s = sim("CompassGathering", &[[0.0, 0.0], [10.0, 0.0]], |c| {
        c.frames = Frames::Compass;
        c.compass_error = 0.0;
        c.compass_static = true;
    });
    // Robot 0 sees the other due east (0°): A_SS waits there; robot 1 sees it due west.
    says(
        s.explain(0),
        &[
            "10 units away",
            "0° counter-clockwise",
            "fixed compass",
            "it waits",
        ],
    );
    says(s.explain(1), &["180°", "moves onto the other robot"]);
}

#[test]
fn crash_gathering_names_its_destination() {
    let mut s = sim(
        "CrashGathering",
        &[[0.0, 0.0], [0.0, 0.0], [10.0, 0.0], [0.0, 10.0]],
        |c| {
            c.scheduler = SchedulerKind::Ssync;
            c.multiplicity = MultiplicityModel::Weak;
        },
    );
    says(
        s.explain(2),
        &["3 occupied points", "a multiplicity", "10 units away"],
    );
    says(
        s.explain(0),
        &["stands on the multiplicity point", "terminates"],
    );
    // The circle's centre is (10, 0); the robot at (5, 0) stands between robot 0 and it.
    let mut s = sim(
        "CrashGathering",
        &[[0.0, 0.0], [5.0, 0.0], [20.0, 0.0]],
        |c| {
            c.scheduler = SchedulerKind::Ssync;
            c.multiplicity = MultiplicityModel::Weak;
        },
    );
    says(
        s.explain(0),
        &[
            "none of them a multiplicity",
            "radius 10",
            "10 units away",
            "in its way",
        ],
    );
}

#[test]
fn randomized_gathering_explains_its_coin() {
    let mut s = sim("RandomizedGathering", &[[0.0, 0.0], [10.0, 0.0]], |_| {});
    says(s.explain(0), &["10 units away", "λ ∈ {0, ½, 1}"]);
}

/// Compass gathering with a dynamic compass draws at each Look: explaining
/// must not use up the draws.
#[test]
fn explaining_compass_gathering_changes_nothing_about_the_run() {
    let make = || {
        sim("CompassGathering", &[[-120.0, 40.0], [110.0, -60.0]], |c| {
            c.frames = Frames::Compass;
            c.compass_error = 40.0;
            c.compass_static = false;
            c.scheduler = SchedulerKind::Ssync;
        })
    };
    let (mut plain, mut asked) = (make(), make());
    for _ in 0..2000 {
        plain.step();
        asked.explain(0);
        asked.explain(1);
        asked.step();
    }
    assert_eq!(plain.robots().pos, asked.robots().pos);
    assert_eq!(plain.compass_deviations(), asked.compass_deviations());
}
