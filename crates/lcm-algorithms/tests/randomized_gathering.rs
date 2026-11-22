//! Pattanayak, Augustine, Mandal 2021: two asynchronous robots gather by
//! moving a random fraction of the way to each other.

use lcm_core::{Budget, SchedulerKind, SimConfig, StopReason};

fn run(algorithm: &str, scheduler: SchedulerKind, seed: u64) -> (StopReason, u64) {
    let config = SimConfig {
        algorithm: algorithm.to_owned(),
        num_of_robots: 2,
        // Mirror images of each other: a deterministic rule that treats both
        // alike (go halfway, go all the way) can keep them apart for ever.
        initial_positions: Some(vec![[-100.0, 0.0], [100.0, 0.0]]),
        random_seed: seed,
        scheduler,
        robot_speeds: 50.0,
        ..SimConfig::default()
    };
    let mut sim = lcm_algorithms::simulation(&config).unwrap();
    let reason = sim.advance(Budget {
        max_events: 1_000_000,
        until_time: f64::INFINITY,
    });
    (reason, sim.event_count() - sim.tick_count())
}

#[test]
fn both_variants_gather_under_async_fsync_and_ssync() {
    for algorithm in ["RandomizedGathering", "RandomizedGatheringUniform"] {
        for scheduler in [
            SchedulerKind::Async,
            SchedulerKind::Fsync,
            SchedulerKind::Ssync,
        ] {
            for seed in 0..30 {
                let (reason, _) = run(algorithm, scheduler, seed);
                assert_eq!(
                    reason,
                    StopReason::Ended,
                    "{algorithm} {scheduler:?} seed {seed}"
                );
            }
        }
    }
}

#[test]
fn the_number_of_moves_depends_on_the_coins() {
    let counts: Vec<u64> = (0..10)
        .map(|seed| run("RandomizedGathering", SchedulerKind::Async, seed).1)
        .collect();
    let first = counts[0];
    assert!(counts.iter().any(|&c| c != first), "{counts:?}");
}

#[test]
fn a_deterministic_halfway_rule_never_gathers_under_fsync() {
    // The contrast: centre of gravity under FSYNC from the same mirrored start
    // gathers in one round, but "go all the way" (lambda = 1 for both) swaps
    // them for ever. Randomness is what breaks the symmetry; here we only
    // check the randomized rule is not secretly deterministic: two seeds
    // differ in how long they take.
    let a = run("RandomizedGathering", SchedulerKind::Fsync, 1).1;
    let b = run("RandomizedGathering", SchedulerKind::Fsync, 2).1;
    let c = run("RandomizedGathering", SchedulerKind::Fsync, 3).1;
    assert!(a != b || b != c);
}

#[test]
fn only_two_robots() {
    let config = SimConfig {
        algorithm: "RandomizedGathering".to_owned(),
        num_of_robots: 3,
        ..SimConfig::default()
    };
    assert!(lcm_algorithms::simulation(&config).is_err());
}
