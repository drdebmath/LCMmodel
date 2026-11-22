//! k-Async (Kirkpatrick et al. 2024): while one robot is in a cycle, any
//! other robot is activated at most k times. The checker here watches the run
//! from outside, event by event, and does not share code with the scheduler.

use lcm_core::{
    Budget, EventKind, RobotState, SchedulerKind, SimConfig, Simulation, StepOutcome, StopReason,
};

fn config(k: u32, n: u32, seed: u64) -> SimConfig {
    SimConfig {
        algorithm: "Gathering".to_owned(),
        num_of_robots: n,
        random_seed: seed,
        scheduler: SchedulerKind::KAsync,
        k_async: k,
        // Slow robots and frequent activations: cycles overlap a lot.
        robot_speeds: 0.5,
        lambda_rate: 20.0,
        ..SimConfig::default()
    }
}

fn sim(c: &SimConfig) -> Simulation {
    lcm_algorithms::simulation(c).unwrap()
}

/// Runs up to `events` events and checks the bound after every one:
/// returns the largest number of Looks any robot started during another's
/// single cycle.
fn worst_overlap(c: &SimConfig, events: u64) -> usize {
    let mut s = sim(c);
    let n = s.robots().len();
    let mut active = vec![false; n];
    // looks[j][i]: Looks robot i started during robot j's current cycle.
    let mut looks = vec![vec![0usize; n]; n];
    let mut worst = 0;
    for _ in 0..events {
        let Ok(info) = s.step_checked() else { break };
        if info.kind == Some(EventKind::Look)
            && info.robot >= 0
            && info.outcome != StepOutcome::Ignored
        {
            let i = usize::try_from(info.robot).unwrap();
            for j in 0..n {
                if active[j] && j != i {
                    looks[j][i] += 1;
                    worst = worst.max(looks[j][i]);
                }
            }
            active[i] = true;
            looks[i].fill(0);
        }
        for (j, on) in active.iter_mut().enumerate() {
            let state = s.robots().state[j];
            if *on && state != RobotState::Look && state != RobotState::Move {
                *on = false;
            }
        }
    }
    worst
}

#[test]
fn no_robot_is_activated_more_than_k_times_during_another_cycle() {
    for k in [1, 2, 3] {
        for seed in 0..5 {
            let worst = worst_overlap(&config(k, 12, seed), 20_000);
            assert!(
                worst <= k as usize,
                "k = {k}, seed {seed}: {worst} Looks in one cycle"
            );
        }
    }
}

#[test]
fn plain_async_breaks_the_bound_on_the_same_runs() {
    // Without the bound the same settings let a robot look many times during
    // one long move: the test above is not passing by accident.
    let asynchronous = SimConfig {
        scheduler: SchedulerKind::Async,
        ..config(1, 12, 0)
    };
    assert!(worst_overlap(&asynchronous, 20_000) > 1);
}

#[test]
fn a_large_k_is_plain_async() {
    let budget = Budget {
        max_events: 50_000,
        until_time: f64::INFINITY,
    };
    let mut a = sim(&SimConfig {
        scheduler: SchedulerKind::Async,
        ..config(1, 10, 3)
    });
    let mut b = sim(&config(1_000_000, 10, 3));
    assert_eq!(a.advance(budget), b.advance(budget));
    assert_eq!(a.robots().pos, b.robots().pos);
    assert_eq!(a.event_count(), b.event_count());
    assert_eq!(b.deferred_looks(), 0);
}

#[test]
fn k_one_puts_looks_off_and_still_gathers() {
    let mut s = sim(&config(1, 15, 7));
    let reason = s.advance(Budget {
        max_events: 5_000_000,
        until_time: f64::INFINITY,
    });
    assert_eq!(reason, StopReason::Ended);
    assert!(s.deferred_looks() > 0, "the bound had to act");
}

#[test]
fn k_must_be_positive() {
    assert!(config(0, 5, 1).validate().is_err());
}
