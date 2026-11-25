//! Pattanayak et al. (Gather_k): under SSYNC the robots that have not crashed
//! gather, from legal starts (at most one multiplicity point).

use lcm_core::geom::dist;
use lcm_core::occupancy::occupancy;
use lcm_core::{
    Budget, FaultSelection, Point, Rng, RobotState, SchedulerKind, SimConfig, StopReason,
};

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

// ---- Review of #17, fix B1: never a second multiplicity point (Lemma 4).

/// Robots per occupied point at the end of a run from `start` (crashed
/// robots included: they are points like any other).
fn end_groups(start: &[[f64; 2]], scheduler: SchedulerKind) -> Vec<usize> {
    let config = SimConfig {
        algorithm: "CrashGathering".into(),
        num_of_robots: u32::try_from(start.len()).unwrap(),
        initial_positions: Some(start.to_vec()),
        scheduler,
        robot_speeds: 20.0,
        ..SimConfig::default()
    };
    let mut sim = lcm_algorithms::simulation(&config).unwrap();
    sim.advance(Budget {
        max_events: 200_000,
        until_time: f64::INFINITY,
    });
    occupancy(&sim.robots().pos, 1e-5).1
}

const SCHEDULERS: [SchedulerKind; 3] = [
    SchedulerKind::Sequential,
    SchedulerKind::Ssync,
    SchedulerKind::Fsync,
];

/// Case 1 of the review: every robot not yet on p* lies on one line through
/// it, so the robot-free sector is the whole plane. The old fallback walked
/// straight through the robot in front, and the line robots leapfrogged each
/// other for ever, ending as two stacks 1e-5 apart.
#[test]
fn robots_on_one_line_through_the_multiplicity_gather() {
    let start = [
        [30.0, 0.0],
        [40.0, 40.0],
        [10.0, 0.0],
        [0.0, 0.0],
        [20.0, 0.0],
        [30.0, 0.0],
    ];
    for scheduler in SCHEDULERS {
        assert_eq!(end_groups(&start, scheduler), vec![6], "{scheduler:?}");
    }
}

/// Case 2 of the review: a robot 8e-6 off the path counted as blocking (ε =
/// 1e-5) but not as on the path's ray when choosing the arc, so the arc
/// landed within ε of it and made a second stack.
#[test]
fn a_robot_almost_on_the_path_does_not_make_a_second_stack() {
    let start = [[10.0, 0.0], [5.0, 8e-6], [0.0, 0.0], [0.0, 0.0], [0.0, 7.0]];
    for scheduler in SCHEDULERS {
        assert_eq!(end_groups(&start, scheduler), vec![5], "{scheduler:?}");
    }
}

/// A random legal start (at most one multiplicity point) on a coarse grid,
/// so that robots often share lines through each other and through p*.
fn grid_start(rng: &mut Rng) -> Vec<[f64; 2]> {
    loop {
        let n = 3 + rng.index(6);
        let start: Vec<[f64; 2]> = (0..n)
            .map(|_| {
                #[allow(clippy::cast_precision_loss)]
                let at = |k: usize| k as f64 * 10.0;
                [at(rng.index(5)), at(rng.index(5))]
            })
            .collect();
        let points: Vec<Point> = start.iter().map(|p| Point::new(p[0], p[1])).collect();
        let (groups, counts, _) = occupancy(&points, 1e-9);
        if groups.len() >= 2 && counts.iter().filter(|&&c| c >= 2).count() <= 1 {
            return start;
        }
    }
}

/// How a run from a grid start ended.
#[derive(Debug, PartialEq)]
enum End {
    /// Every robot on one point.
    Gathered,
    /// A second multiplicity point appeared at this event (Lemma 4 broken).
    SecondStack(u64),
    /// Ended with these robots per point, one of them a stack.
    Apart(Vec<usize>),
    /// Ended with every robot alone, all within this distance of each other:
    /// each is within ε of the centre of their circle, so none moves, but
    /// they are more than ε apart, so no stack forms (see `random_grid_search`).
    Close(f64),
}

/// Plays `start` to the end, checking after every event that there is at
/// most one multiplicity point.
fn play_checked(start: &[[f64; 2]], scheduler: SchedulerKind, seed: u64) -> End {
    let config = SimConfig {
        algorithm: "CrashGathering".into(),
        num_of_robots: u32::try_from(start.len()).unwrap(),
        initial_positions: Some(start.to_vec()),
        scheduler,
        robot_speeds: 20.0,
        random_seed: seed,
        max_events: Some(200_000),
        ..SimConfig::default()
    };
    let mut sim = lcm_algorithms::simulation(&config).unwrap();
    while sim.step_checked().is_ok() {
        let (_, counts, _) = occupancy(&sim.robots().pos, 1e-5);
        if counts.iter().filter(|&&c| c >= 2).count() > 1 {
            return End::SecondStack(sim.event_count());
        }
    }
    let pos = &sim.robots().pos;
    let (_, counts, _) = occupancy(pos, 1e-5);
    if counts.len() == 1 {
        End::Gathered
    } else if counts.iter().any(|&c| c >= 2) {
        End::Apart(counts)
    } else {
        let spread = pos
            .iter()
            .flat_map(|a| pos.iter().map(move |b| dist(*a, *b)))
            .fold(0.0, f64::max);
        End::Close(spread)
    }
}

/// Lemma 4 of arXiv:1608.02432: from a legal start there is never a second
/// multiplicity point, at any moment of the run, not only at the end.
#[test]
fn there_is_never_a_second_multiplicity_point() {
    // Four sequential runs of `random_grid_search` that made a second stack
    // before the fix (run seeds 235, 634, 959 and 1201).
    let found: [(u64, &[[f64; 2]]); 4] = [
        (
            235,
            &[
                [0.0, 40.0],
                [10.0, 0.0],
                [20.0, 10.0],
                [10.0, 10.0],
                [40.0, 0.0],
                [40.0, 40.0],
            ],
        ),
        (
            634,
            &[
                [0.0, 40.0],
                [10.0, 10.0],
                [30.0, 0.0],
                [30.0, 20.0],
                [10.0, 40.0],
                [10.0, 20.0],
            ],
        ),
        (
            959,
            &[
                [20.0, 40.0],
                [20.0, 30.0],
                [0.0, 0.0],
                [0.0, 30.0],
                [0.0, 0.0],
                [0.0, 20.0],
            ],
        ),
        (
            1201,
            &[
                [0.0, 40.0],
                [10.0, 40.0],
                [30.0, 10.0],
                [20.0, 40.0],
                [30.0, 0.0],
            ],
        ),
    ];
    for (seed, start) in found {
        for scheduler in SCHEDULERS {
            let end = play_checked(start, scheduler, seed);
            assert_eq!(end, End::Gathered, "{scheduler:?} seed {seed}");
        }
    }
    let mut rng = Rng::new(2017);
    for scheduler in SCHEDULERS {
        for seed in 0..150 {
            let start = grid_start(&mut rng);
            let end = play_checked(&start, scheduler, seed);
            assert!(
                !matches!(end, End::SecondStack(_) | End::Apart(_)),
                "{scheduler:?} seed {seed} from {start:?}: {end:?}"
            );
        }
    }
}

/// The review's random search: 1,800 runs from grid starts per scheduler (it
/// found 3 failures in 1,800 sequential runs before the fix).
/// `cargo test --release -p lcm-algorithms --test crash_gathering -- --ignored
/// --nocapture` prints the counts.
///
/// Runs that end `Close` are not counted as failures of Lemma 4: with no
/// stack, a robot heads for the centre of the smallest enclosing circle, and
/// under the sequential scheduler that centre moves after every move, so a
/// few robots can close in on it without ever meeting exactly, and stop once
/// each is within ε of it (all within about 2ε of each other). This is a
/// precision limit of the stop rule, left as it is (see the PR description).
#[test]
#[ignore = "about ten seconds in release: run with --ignored --release"]
fn random_grid_search() {
    let mut rng = Rng::new(160_802_432);
    let mut broken = 0;
    for scheduler in SCHEDULERS {
        let runs = 1_800;
        let (mut gathered, mut close, mut worst) = (0, 0, 0.0_f64);
        let mut failures = Vec::new();
        for seed in 0..runs {
            let start = grid_start(&mut rng);
            match play_checked(&start, scheduler, seed) {
                End::Gathered => gathered += 1,
                End::Close(spread) => {
                    close += 1;
                    worst = worst.max(spread);
                }
                other => failures.push((seed, start, other)),
            }
        }
        println!(
            "{scheduler:?}: {runs} runs, {gathered} gathered, {} with a second stack or ending apart, {close} stopped with no stack within {worst:.1e} of each other",
            failures.len()
        );
        for f in failures.iter().take(5) {
            println!("  {f:?}");
        }
        broken += failures.len();
    }
    assert_eq!(broken, 0);
}
