//! Pattanayak et al. (Gather_k): under SSYNC the robots that have not crashed
//! gather, from legal starts (at most one multiplicity point).

use lcm_core::occupancy::occupancy;
use lcm_core::{Budget, FaultSelection, RobotState, SchedulerKind, SimConfig, StopReason};

fn run(config: &SimConfig) -> (StopReason, usize, usize) {
    let mut sim = lcm_algorithms::simulation(config).unwrap();
    let reason = sim.advance(Budget {
        max_events: 200_000,
        until_time: f64::INFINITY,
    });
    let r = sim.robots();
    let working: Vec<_> = (0..r.len())
        .filter(|&i| r.state[i] != RobotState::Crash)
        .map(|i| r.pos[i])
        .collect();
    let (points, _, _) = occupancy(&working, 1e-5);
    let crashed = r.len() - working.len();
    (reason, points.len(), crashed)
}

fn config(n: u32, faults: u32, seed: u64, scheduler: SchedulerKind) -> SimConfig {
    SimConfig {
        algorithm: "CrashGathering".to_owned(),
        num_of_robots: n,
        num_of_faults: faults,
        fault_type: FaultSelection::Crash,
        random_seed: seed,
        scheduler,
        robot_speeds: 20.0,
        ..SimConfig::default()
    }
}

#[test]
fn working_robots_gather_despite_crashes() {
    for scheduler in [
        SchedulerKind::Ssync,
        SchedulerKind::Sequential,
        SchedulerKind::Fsync,
    ] {
        for seed in 0..15 {
            for faults in [0, 3, 9] {
                // The lone-robot case runs to the event budget: a few seeds do.
                if faults == 9 && seed >= 3 {
                    continue;
                }
                let (reason, points, crashed) = run(&config(10, faults, seed, scheduler));
                // A lone working robot has gathered but cannot know it: it
                // waits on the centre for robots that may still come (one
                // terminating there alone could be left behind when the
                // circle's centre moves). Two or more terminate together.
                if faults < 9 {
                    assert_eq!(
                        reason,
                        StopReason::Ended,
                        "{scheduler:?} seed {seed} faults {faults}"
                    );
                }
                assert_eq!(points, 1, "{scheduler:?} seed {seed} faults {faults}");
                assert_eq!(crashed, faults as usize);
            }
        }
    }
}

#[test]
fn a_start_with_one_multiplicity_gathers_on_it() {
    for seed in 0..10 {
        let c = SimConfig {
            multiplicities: 1,
            multiplicity_size: 3,
            ..config(9, 2, seed, SchedulerKind::Ssync)
        };
        let (reason, points, _) = run(&c);
        assert_eq!((reason, points), (StopReason::Ended, 1), "seed {seed}");
    }
}

#[test]
fn robots_in_line_behind_each_other_still_gather() {
    // All on one ray from the centre: everyone but the front robot is blocked
    // and must curve off the ray.
    let c = SimConfig {
        initial_positions: Some(vec![
            [10.0, 0.0],
            [20.0, 0.0],
            [30.0, 0.0],
            [40.0, 0.0],
            [-40.0, 0.0],
            [0.0, 35.0],
        ]),
        ..config(6, 0, 1, SchedulerKind::Ssync)
    };
    let (reason, points, _) = run(&c);
    assert_eq!((reason, points), (StopReason::Ended, 1));
}

#[test]
fn async_is_refused() {
    assert!(lcm_algorithms::simulation(&config(5, 0, 1, SchedulerKind::Async)).is_err());
}
