//! Pattanayak, Augustine, Mandal 2021: two asynchronous robots gather by
//! moving a random fraction of the way to each other.

use lcm_core::{Budget, EventKind, SchedulerKind, SimConfig, StopReason};

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

/// Renamed from `a_deterministic_halfway_rule_never_gathers_under_fsync`,
/// which did not test that (review of #17, B4): the randomized rule is not
/// secretly deterministic, since three seeds do not all take the same number
/// of events under FSYNC from the mirrored start.
#[test]
fn seeds_change_how_long_fsync_takes() {
    let a = run("RandomizedGathering", SchedulerKind::Fsync, 1).1;
    let b = run("RandomizedGathering", SchedulerKind::Fsync, 2).1;
    let c = run("RandomizedGathering", SchedulerKind::Fsync, 3).1;
    assert!(a != b || b != c);
}

/// Looks of both robots until they gather from `start`, under `scheduler`.
fn looks(start: [[f64; 2]; 2], scheduler: SchedulerKind, seed: u64) -> u64 {
    let config = SimConfig {
        algorithm: "RandomizedGathering".to_owned(),
        num_of_robots: 2,
        initial_positions: Some(start.to_vec()),
        random_seed: seed,
        scheduler,
        robot_speeds: 50.0,
        ..SimConfig::default()
    };
    let mut sim = lcm_algorithms::simulation(&config).unwrap();
    let mut looks = 0;
    loop {
        match sim.step_checked() {
            Ok(info) => looks += u64::from(info.kind == Some(EventKind::Look)),
            Err(reason) => {
                assert_eq!(reason, StopReason::Ended, "seed {seed}");
                return looks;
            }
        }
    }
}

/// Theorem 12 (proof): when the wait plus compute time of every cycle
/// exceeds some τ > 0, both robots together make at most 18 (log2(δ/τ) + 1)
/// Looks in expectation, δ the starting distance. The paper's τ is that time
/// bound, used in the proof as a distance (robots of unit speed); here it is
/// taken as ε, the distance at which the robots count as gathered.
///
/// The simulator cannot meet the hypothesis: its async waits are
/// exponential, with no positive lower bound, and Compute takes no time
/// (notes() says so). So this is a consistency check of the averages, not a
/// test of the theorem: the mean over many seeds must stay well within the
/// bound, under async and under the sequential scheduler (an async schedule
/// in which cycles never overlap).
#[test]
fn average_looks_stay_within_theorem_12s_bound() {
    let start = [[-100.0, 0.0], [100.0, 0.0]];
    let bound = 18.0 * (libm::log2(200.0 / 1e-5) + 1.0);
    for scheduler in [SchedulerKind::Async, SchedulerKind::Sequential] {
        let seeds = 2_000;
        let total: u64 = (0..seeds).map(|seed| looks(start, scheduler, seed)).sum();
        #[allow(clippy::cast_precision_loss)]
        let mean = total as f64 / seeds as f64;
        println!("{scheduler:?}: {mean:.2} Looks on average, bound {bound:.0}");
        assert!(mean < bound, "{scheduler:?}: {mean} >= {bound}");
    }
    let notes = lcm_algorithms::notes(
        "RandomizedGathering",
        &SimConfig {
            algorithm: "RandomizedGathering".to_owned(),
            num_of_robots: 2,
            ..SimConfig::default()
        },
    );
    assert!(notes.iter().any(|n| n.contains("Theorem 12")), "{notes:?}");
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
