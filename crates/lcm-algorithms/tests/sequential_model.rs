//! The sequential scheduler's model features: non-rigid moves that stop after
//! δ, explicit schedules, and the "nothing can move any more" ending.

use lcm_core::{
    Activation, Algorithm, Budget, ConfigError, Decision, EventKind, Look, OverlayUpdate, Plan,
    Point, Rng, RobotState, ScheduleEnd, SchedulerKind, Scratch, SimConfig, Simulation,
    StepOutcome, StopPolicy, StopReason, TurnGap,
};

/// Two robots 600 apart; Gathering sends each to the midpoint, 300 away.
fn two(delta: Option<f64>, policy: StopPolicy) -> SimConfig {
    SimConfig {
        num_of_robots: 2,
        initial_positions: Some(vec![[-300.0, 0.0], [300.0, 0.0]]),
        scheduler: SchedulerKind::Sequential,
        turn_gap: TurnGap::None,
        rigid_movement: false,
        delta,
        stop_policy: policy,
        robot_speeds: 1000.0,
        ..SimConfig::default()
    }
}

/// Where robot 0 stands once its first move is over.
fn after_first_move(config: &SimConfig) -> Point {
    let mut sim = lcm_algorithms::simulation(config).unwrap();
    loop {
        let s = sim.step();
        if s.kind == Some(EventKind::Wait) && s.robot == 0 {
            return sim.position_at(0, s.time);
        }
        assert!(!sim.ended(), "robot 0 never finished a move");
    }
}

fn x_moved(p: Point) -> f64 {
    p.x + 300.0
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-6
}

#[test]
fn a_robot_stopped_right_after_delta_covers_exactly_delta() {
    let p = after_first_move(&two(Some(1.0), StopPolicy::Delta));
    assert!(close(x_moved(p), 1.0), "moved {}", x_moved(p));
    assert!(close(p.y, 0.0));
}

#[test]
fn a_fraction_stops_that_far_along_but_never_before_delta() {
    let half = SimConfig {
        stop_fraction: 0.5,
        ..two(Some(1.0), StopPolicy::Fraction)
    };
    assert!(close(x_moved(after_first_move(&half)), 150.0));
    let tiny = SimConfig {
        stop_fraction: 0.001, // 0.3 of the way is less than delta = 1
        ..two(Some(1.0), StopPolicy::Fraction)
    };
    assert!(close(x_moved(after_first_move(&tiny)), 1.0));
}

#[test]
fn a_random_stop_lies_between_delta_and_the_destination_and_repeats_with_the_seed() {
    let moved = |seed: u64| {
        let config = SimConfig {
            random_seed: seed,
            ..two(Some(1.0), StopPolicy::Random)
        };
        x_moved(after_first_move(&config))
    };
    let a: Vec<f64> = (1..=20).map(moved).collect();
    assert!(a.iter().all(|d| (1.0..=300.0).contains(d)), "{a:?}");
    assert_eq!(a, (1..=20).map(moved).collect::<Vec<_>>());
    assert!(
        a.windows(2).any(|w| w[0] != w[1]),
        "every seed stopped alike"
    );
}

#[test]
fn a_stopped_robot_still_shows_the_destination_it_computed() {
    let mut sim = lcm_algorithms::simulation(&two(Some(1.0), StopPolicy::Delta)).unwrap();
    // Robot 0 looks and starts walking to the midpoint, 300 away.
    loop {
        let s = sim.step();
        if s.kind == Some(EventKind::Look) && s.outcome == StepOutcome::Moved {
            break;
        }
    }
    let r = sim.robots();
    assert_eq!(r.target[0], Some(Point::new(0.0, 0.0)));
    assert!(close(r.stop[0].unwrap().x, -299.0), "{:?}", r.stop[0]);
    assert_eq!(r.goal(0), r.stop[0]);
    // After the move it stands at the stopping point, and the move is forgotten.
    let end = after_first_move(&two(Some(1.0), StopPolicy::Delta));
    assert!(close(end.x, -299.0));
}

#[test]
fn a_robot_whose_destination_is_within_delta_arrives() {
    let near = SimConfig {
        initial_positions: Some(vec![[-0.5, 0.0], [0.5, 0.0]]),
        ..two(Some(1.0), StopPolicy::Delta)
    };
    let p = after_first_move(&near);
    assert!(close(p.x, 0.0), "stopped at {}", p.x);
}

#[test]
fn rigid_movement_and_a_missing_delta_keep_the_original_behaviour() {
    let rigid = SimConfig {
        rigid_movement: true,
        ..two(Some(1.0), StopPolicy::Delta)
    };
    assert!(close(x_moved(after_first_move(&rigid)), 300.0));
    let no_delta = two(None, StopPolicy::Delta);
    assert!(close(x_moved(after_first_move(&no_delta)), 300.0));
}

fn three() -> SimConfig {
    SimConfig {
        num_of_robots: 3,
        initial_positions: Some(vec![[-300.0, 0.0], [300.0, 0.0], [0.0, 400.0]]),
        scheduler: SchedulerKind::Sequential,
        turn_gap: TurnGap::None,
        robot_speeds: 1000.0,
        ..SimConfig::default()
    }
}

fn looks(config: &SimConfig, count: usize) -> Vec<i32> {
    let mut sim = lcm_algorithms::simulation(config).unwrap();
    let mut out = Vec::new();
    while out.len() < count && !sim.ended() {
        let s = sim.step();
        if s.kind == Some(EventKind::Look) {
            out.push(s.robot);
        }
    }
    out
}

fn robots(list: &[usize]) -> Vec<Activation> {
    list.iter().map(|&r| Activation::Robot(r)).collect()
}

#[test]
fn an_explicit_schedule_is_followed_then_round_robin_carries_on() {
    let config = SimConfig {
        schedule: Some(robots(&[1, 1, 0])),
        ..three()
    };
    assert_eq!(looks(&config, 8), [1, 1, 0, 0, 1, 2, 0, 1]);
}

#[test]
fn a_repeating_schedule_plays_again() {
    let config = SimConfig {
        schedule: Some(robots(&[0, 1, 2, 1])),
        schedule_end: ScheduleEnd::Repeat,
        ..three()
    };
    assert_eq!(looks(&config, 8), [0, 1, 2, 1, 0, 1, 2, 1]);
}

#[test]
fn a_turn_can_name_where_the_adversary_stops_the_robot() {
    let config = SimConfig {
        rigid_movement: false,
        delta: Some(1.0),
        stop_policy: StopPolicy::Delta,
        schedule: Some(vec![Activation::Stopped {
            robot: 0,
            stop: 0.5,
        }]),
        ..two(Some(1.0), StopPolicy::Delta)
    };
    // Half of the 300 to the midpoint, not the delta of 1 the policy would give.
    assert!(close(x_moved(after_first_move(&config)), 150.0));
}

#[test]
fn an_epoch_of_an_explicit_schedule_ends_when_every_robot_has_had_a_turn() {
    let config = SimConfig {
        schedule: Some(robots(&[0, 0, 0, 1, 2])),
        ..three()
    };
    let mut sim = lcm_algorithms::simulation(&config).unwrap();
    let mut epochs_at_turn = Vec::new();
    while epochs_at_turn.len() < 7 {
        let before = sim.turn_count();
        sim.step();
        if sim.turn_count() > before {
            epochs_at_turn.push(sim.epochs_completed());
        }
    }
    // Counted when the turn after the last robot's first one begins.
    assert_eq!(epochs_at_turn, [0, 0, 0, 0, 0, 1, 1]);
}

#[test]
fn turns_taken_are_counted() {
    let mut sim = lcm_algorithms::simulation(&three()).unwrap();
    let mut looks = 0;
    while !sim.ended() {
        if sim.step().kind == Some(EventKind::Look) {
            looks += 1;
        }
    }
    assert_eq!(sim.turn_count(), looks);
    assert!(looks > 3);
}

fn rejected(config: &SimConfig) -> String {
    match lcm_algorithms::simulation(config) {
        Err(ConfigError::Schedule(message)) => message,
        Err(other) => panic!("wrong error: {other}"),
        Ok(_) => panic!("accepted {config:?}"),
    }
}

#[test]
fn a_schedule_that_cannot_work_is_refused() {
    let with = |schedule: Vec<Activation>| SimConfig {
        schedule: Some(schedule),
        ..three()
    };
    assert!(rejected(&with(robots(&[0, 3]))).contains("robot 3"));
    assert!(rejected(&with(vec![])).contains("at least one"));
    let stop = |stop| Activation::Stopped { robot: 0, stop };
    assert!(rejected(&with(vec![stop(1.5)])).contains("(0, 1]"));
    // A stopping point needs non-rigid movement and a delta.
    assert!(rejected(&with(vec![stop(0.5)])).contains("delta"));
    let repeat = SimConfig {
        schedule_end: ScheduleEnd::Repeat,
        ..with(robots(&[0, 1]))
    };
    assert!(rejected(&repeat).contains("robot 2"));
    let asynchronous = SimConfig {
        scheduler: SchedulerKind::Async,
        ..with(robots(&[0, 1, 2]))
    };
    assert!(rejected(&asynchronous).contains("sequential"));
}

#[test]
fn delta_and_the_stop_fraction_must_be_sensible() {
    let bad_delta = SimConfig {
        delta: Some(0.0),
        ..SimConfig::default()
    };
    assert!(matches!(
        lcm_algorithms::simulation(&bad_delta),
        Err(ConfigError::NotPositive { field: "delta", .. })
    ));
    let bad_fraction = SimConfig {
        stop_fraction: 0.0,
        ..SimConfig::default()
    };
    assert!(rejected(&bad_fraction).contains("stop_fraction"));
}

/// A robot that never moves and never terminates.
struct StayPut;

impl Algorithm for StayPut {
    fn key(&self) -> &'static str {
        "StayPut"
    }

    fn compute(&self, look: &Look<'_>, _: &mut Rng, _: &mut Scratch) -> Decision {
        Decision {
            target: look.position,
            terminate: false,
            overlay: OverlayUpdate::Keep,
        }
    }
}

fn stay_put(config: &SimConfig) -> Simulation {
    Simulation::new(config, Plan::single(Box::new(StayPut))).unwrap()
}

fn run_to_the_end(sim: &mut Simulation) -> StopReason {
    sim.advance(Budget {
        max_events: 1_000_000,
        until_time: f64::INFINITY,
    })
}

#[test]
fn robots_that_never_move_stall_instead_of_running_forever() {
    for order in [
        lcm_core::ActivationOrder::RoundRobin,
        lcm_core::ActivationOrder::Random,
    ] {
        for gap in [TurnGap::None, TurnGap::Random] {
            let config = SimConfig {
                activation_order: order,
                turn_gap: gap,
                ..three()
            };
            let mut sim = stay_put(&config);
            assert_eq!(run_to_the_end(&mut sim), StopReason::Stalled, "{config:?}");
            assert!(sim.stalled() && !sim.ended());
            assert_eq!(sim.epochs_completed(), 1);
            assert_eq!(sim.turn_count(), 3, "one silent epoch, then stop");
            // Stepping a stalled run does nothing more.
            let events = sim.event_count();
            assert_eq!(sim.step().outcome, StepOutcome::Ended);
            assert_eq!(sim.event_count(), events);
        }
    }
}

#[test]
fn a_schedule_of_robots_that_never_move_stalls_too() {
    let config = SimConfig {
        schedule: Some(robots(&[0, 1, 2, 1])),
        schedule_end: ScheduleEnd::Repeat,
        ..three()
    };
    let mut sim = stay_put(&config);
    assert_eq!(run_to_the_end(&mut sim), StopReason::Stalled);
    // The epoch ends once robots 0, 1 and 2 have each had a turn: 3 turns.
    assert_eq!(sim.turn_count(), 3);
}

#[test]
fn a_run_that_is_still_changing_is_not_stalled() {
    let mut sim = lcm_algorithms::simulation(&three()).unwrap();
    assert_eq!(run_to_the_end(&mut sim), StopReason::Ended);
    assert!(!sim.stalled());
    assert!(sim.robots().terminated.iter().all(|t| *t));
    assert!((0..3).all(|i| sim.robots().state[i] != RobotState::Move));
}

#[test]
fn faults_switch_the_stall_check_off() {
    // An omission fault skips moves at random, so a silent epoch proves nothing.
    let config = SimConfig {
        num_of_faults: 1,
        fault_type: lcm_core::FaultSelection::Omission,
        max_events: Some(5_000),
        ..three()
    };
    let mut sim = stay_put(&config);
    assert_eq!(run_to_the_end(&mut sim), StopReason::MaxEvents);
    assert!(!sim.stalled());
}

// ---- starting multiplicities ----

fn spread(n: u32) -> SimConfig {
    SimConfig {
        num_of_robots: n,
        scheduler: SchedulerKind::Sequential,
        turn_gap: TurnGap::None,
        ..SimConfig::default()
    }
}

/// How many robots stand on each occupied point, most crowded first.
fn crowds(config: &SimConfig) -> Vec<usize> {
    let points = config.resolve_positions();
    let mut counts: Vec<usize> = Vec::new();
    let mut seen: Vec<Point> = Vec::new();
    for p in points {
        match seen.iter().position(|q| *q == p) {
            Some(i) => counts[i] += 1,
            None => {
                seen.push(p);
                counts.push(1);
            }
        }
    }
    counts.sort_unstable_by(|a, b| b.cmp(a));
    counts
}

#[test]
fn robots_can_start_stacked_on_a_given_number_of_points() {
    let config = SimConfig {
        multiplicities: 2,
        multiplicity_size: 4,
        ..spread(30)
    };
    let counts = crowds(&config);
    assert_eq!(&counts[..3], [4, 4, 1], "{counts:?}");
    assert_eq!(counts.len(), 30 - 2 * 3, "24 distinct points");
    // Everything else stays where the start put it.
    let plain = spread(30).resolve_positions();
    let stacked = config.resolve_positions();
    assert_eq!(
        plain.iter().zip(&stacked).filter(|(a, b)| a != b).count(),
        6
    );
}

#[test]
fn stacking_is_repeatable_and_follows_the_seed() {
    let at = |seed: u64| {
        SimConfig {
            multiplicities: 1,
            multiplicity_size: 5,
            random_seed: seed,
            ..spread(20)
        }
        .resolve_positions()
    };
    assert_eq!(at(7), at(7));
    assert_ne!(at(7), at(8));
}

#[test]
fn an_unspecified_size_is_chosen_for_you() {
    let one = SimConfig {
        multiplicities: 1,
        ..spread(30)
    };
    assert_eq!(crowds(&one)[0], 10, "a third of the robots");
    let several = SimConfig {
        multiplicities: 3,
        ..spread(30)
    };
    assert_eq!(&crowds(&several)[..3], [4, 4, 4]);
}

#[test]
fn impossible_stackings_are_refused() {
    let refused = |config: SimConfig| match lcm_algorithms::simulation(&config) {
        Err(ConfigError::Start(message)) => message,
        Err(other) => panic!("wrong error: {other}"),
        Ok(_) => panic!("accepted {config:?}"),
    };
    let single = SimConfig {
        multiplicities: 2,
        multiplicity_size: 1,
        ..spread(30)
    };
    assert!(refused(single).contains("at least 2"));
    let too_many = SimConfig {
        multiplicities: 7,
        multiplicity_size: 5,
        ..spread(30)
    };
    assert!(refused(too_many).contains("need 35 robots, there are 30"));
}

#[test]
fn a_swarm_that_starts_stacked_still_runs() {
    let config = SimConfig {
        multiplicities: 2,
        multiplicity_size: 4,
        ..spread(30)
    };
    let mut sim = lcm_algorithms::simulation(&config).unwrap();
    assert_eq!(run_to_the_end(&mut sim), StopReason::Ended);
}

// ---- the turn limit ----

#[test]
fn a_run_stops_after_max_turns() {
    for turns in [1, 5, 9] {
        let config = SimConfig {
            max_turns: Some(turns),
            ..three()
        };
        let mut sim = lcm_algorithms::simulation(&config).unwrap();
        assert_eq!(run_to_the_end(&mut sim), StopReason::MaxTurns);
        assert_eq!(sim.turn_count(), turns);
        assert!(sim.turn_limit_reached() && !sim.ended() && !sim.stalled());
        // The robot whose turn was the last still finished its move.
        assert!((0..3).all(|i| sim.robots().state[i] != RobotState::Move));
        let events = sim.event_count();
        assert_eq!(sim.step().outcome, StepOutcome::Ended);
        assert_eq!(sim.event_count(), events);
    }
}

#[test]
fn epochs_are_counted_up_to_the_turn_limit() {
    // Three robots: after 6 turns, two whole epochs.
    let config = SimConfig {
        max_turns: Some(6),
        ..three()
    };
    let mut sim = lcm_algorithms::simulation(&config).unwrap();
    run_to_the_end(&mut sim);
    assert_eq!(sim.epochs_completed(), 2);
}

#[test]
fn a_run_that_finishes_first_is_not_cut_short() {
    let config = SimConfig {
        max_turns: Some(100_000),
        ..three()
    };
    let mut sim = lcm_algorithms::simulation(&config).unwrap();
    assert_eq!(run_to_the_end(&mut sim), StopReason::Ended);
    assert!(!sim.turn_limit_reached());
}

#[test]
fn a_turn_limit_needs_the_sequential_scheduler_and_a_positive_number() {
    let asynchronous = SimConfig {
        scheduler: SchedulerKind::Async,
        max_turns: Some(5),
        ..three()
    };
    assert!(rejected(&asynchronous).contains("sequential"));
    let zero = SimConfig {
        max_turns: Some(0),
        ..three()
    };
    assert!(rejected(&zero).contains("at least 1"));
}

// ---- the registry builds from the config ----

#[test]
fn the_registry_builds_a_plan_from_the_run_config() {
    let config = SimConfig::default();
    assert!(lcm_algorithms::plan("Gathering", &config).is_ok());
    assert!(lcm_algorithms::plan("SEC", &config).is_ok());
    assert!(matches!(
        lcm_algorithms::plan("NoSuchAlgorithm", &config),
        Err(ConfigError::UnknownAlgorithm(name)) if name == "NoSuchAlgorithm"
    ));
}
