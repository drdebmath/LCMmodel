//! The SSYNC and FSYNC schedulers, the live `await` schedule, private frames
//! and obstructed visibility, and the two impossibility results of Section 6
//! of the sequential-schedulers paper played out on the core:
//!
//! - Theorem 4: without multiplicity detection, a sequential adversary keeps
//!   robots on two points for ever.
//! - Lemma 8: under SSYNC, activating a whole multiplicity at once keeps two
//!   multiplicities apart for ever, even with multiplicity detection.

use lcm_core::occupancy::occupancy;
use lcm_core::{
    Activation, Budget, Frames, MultiplicityModel, Point, ScheduleEnd, SchedulerKind, SimConfig,
    Simulation, StopReason, TurnGap,
};

fn base(algorithm: &str, positions: &[[f64; 2]], scheduler: SchedulerKind) -> SimConfig {
    SimConfig {
        algorithm: algorithm.to_owned(),
        num_of_robots: u32::try_from(positions.len()).unwrap(),
        initial_positions: Some(positions.to_vec()),
        scheduler,
        turn_gap: TurnGap::None,
        width_bound: None,
        height_bound: None,
        robot_speeds: 5.0,
        ..SimConfig::default()
    }
}

fn sim(config: &SimConfig) -> Simulation {
    lcm_algorithms::simulation(config).expect("valid configuration")
}

fn run(sim: &mut Simulation) -> StopReason {
    sim.advance(Budget {
        max_events: 5_000_000,
        until_time: f64::INFINITY,
    })
}

/// Plays until `rounds` rounds are over (the next has been queued) or the
/// run stops. Positions are then those at the end of round `rounds`.
fn play_rounds(sim: &mut Simulation, rounds: u64) -> Option<StopReason> {
    while sim.round_count() <= rounds {
        if let Err(reason) = sim.step_checked() {
            return Some(reason);
        }
    }
    None
}

/// Distinct points and how many robots share each, in first-seen order.
fn census(sim: &Simulation) -> Vec<usize> {
    let (_, counts, _) = occupancy(&sim.robots().pos, 1e-9);
    counts
}

fn close(a: Point, b: Point) -> bool {
    (a.x - b.x).abs() <= 1e-9 && (a.y - b.y).abs() <= 1e-9
}

#[test]
fn fsync_robots_all_look_at_the_same_snapshot() {
    // Centre of gravity under FSYNC with rigid moves: everyone computes the
    // same centroid from the same snapshot, so one round gathers them. Had
    // any robot seen another mid-move, they would not land together.
    let positions = [[0.0, 0.0], [10.0, 0.0], [0.0, 6.0], [7.0, 9.0], [-3.0, 4.0]];
    let mut s = sim(&base("Gathering", &positions, SchedulerKind::Fsync));
    assert_eq!(s.round_members(), &[0, 1, 2, 3, 4]);
    play_rounds(&mut s, 1);
    let c = Point::new(14.0 / 5.0, 19.0 / 5.0);
    assert!(s.robots().pos.iter().all(|p| close(*p, c)));
    assert_eq!(run(&mut s), StopReason::Ended);
    assert_eq!(
        s.round_count(),
        2,
        "the second round sees one point and terminates"
    );
}

#[test]
fn fsync_is_the_same_whatever_the_gap_between_rounds() {
    let positions = [[0.0, 0.0], [10.0, 0.0], [3.0, 8.0]];
    let mut a = base("SEC", &positions, SchedulerKind::Fsync);
    a.max_rounds = Some(5);
    let mut b = a.clone();
    b.turn_gap = TurnGap::Random;
    let (mut a, mut b) = (sim(&a), sim(&b));
    assert_eq!(run(&mut a), run(&mut b));
    assert_eq!(a.round_count(), b.round_count());
    for (p, q) in a.robots().pos.iter().zip(&b.robots().pos) {
        assert!(close(*p, *q));
    }
}

#[test]
fn ssync_rounds_are_never_empty_and_stay_fair() {
    let positions: Vec<[f64; 2]> = (0..12)
        .map(|k| {
            let a = f64::from(k) * 0.5;
            [100.0 * libm::cos(a), 80.0 * libm::sin(a)]
        })
        .collect();
    let mut config = base("Gathering", &positions, SchedulerKind::Ssync);
    config.ssync_probability = 0.1;
    config.ssync_fairness = 3;
    config.max_rounds = Some(200);
    let mut s = sim(&config);
    let mut idle = [0u32; 12];
    let mut sizes = Vec::new();
    let mut seen_round = 0;
    loop {
        if s.round_count() != seen_round {
            seen_round = s.round_count();
            let members = s.round_members().to_vec();
            assert!(!members.is_empty(), "round {seen_round} activates nobody");
            sizes.push(members.len());
            for (r, k) in idle.iter_mut().enumerate() {
                if members.contains(&r) {
                    *k = 0;
                } else {
                    *k += 1;
                    assert!(*k <= 3, "robot {r} left out of more than 3 rounds");
                }
            }
        }
        if s.step_checked().is_err() {
            break;
        }
    }
    assert!(sizes.len() > 20);
    assert!(sizes.iter().any(|&k| k < 12), "SSYNC must not be FSYNC");
}

#[test]
fn ssync_and_the_rest_are_reproducible_from_the_seed() {
    let positions = [[0.0, 0.0], [10.0, 0.0], [3.0, 8.0], [-6.0, 2.0]];
    let mut config = base("GoToCenter", &positions, SchedulerKind::Ssync);
    config.max_rounds = Some(30);
    let (mut a, mut b) = (sim(&config), sim(&config));
    run(&mut a);
    run(&mut b);
    assert_eq!(a.robots().pos, b.robots().pos);
    assert_eq!(a.event_count(), b.event_count());
}

#[test]
fn explicit_rounds_are_played_in_order_then_repeat() {
    let positions = [[0.0, 0.0], [10.0, 0.0], [3.0, 8.0]];
    let mut config = base("Gathering", &positions, SchedulerKind::Ssync);
    config.rounds = Some(vec![vec![2], vec![0, 1]]);
    config.schedule_end = ScheduleEnd::Repeat;
    config.max_rounds = Some(4);
    let mut s = sim(&config);
    let mut played = vec![s.round_members().to_vec()];
    while s.step_checked().is_ok() {
        if s.round_count() as usize > played.len() {
            played.push(s.round_members().to_vec());
        }
    }
    assert_eq!(played, vec![vec![2], vec![0, 1], vec![2], vec![0, 1]]);
}

#[test]
fn bad_round_settings_are_refused() {
    let positions = [[0.0, 0.0], [10.0, 0.0]];
    let mut c = base("Gathering", &positions, SchedulerKind::Ssync);
    c.rounds = Some(vec![vec![]]);
    assert!(c.validate().is_err());
    c.rounds = Some(vec![vec![5]]);
    assert!(c.validate().is_err());
    c.rounds = Some(vec![vec![0]]);
    c.schedule_end = ScheduleEnd::Repeat;
    assert!(c.validate().unwrap_err().to_string().contains("fair"));
    let mut c = base("Gathering", &positions, SchedulerKind::Sequential);
    c.rounds = Some(vec![vec![0]]);
    assert!(c.validate().is_err(), "rounds need SSYNC");
    let mut c = base("Gathering", &positions, SchedulerKind::Async);
    c.max_rounds = Some(3);
    assert!(c.validate().is_err(), "max_rounds needs rounds");
    let mut c = base("Gathering", &positions, SchedulerKind::Ssync);
    c.ssync_probability = 0.0;
    assert!(c.validate().is_err());
}

/// Lemma 8: two multiplicities of two robots each. The SSYNC adversary
/// activates a whole multiplicity at once; both robots take the κ > 1 rule
/// to the same midpoint, so there are again two multiplicities, half as far
/// apart. The sequential scheduler, on the same start, gathers.
#[test]
fn lemma_8_ssync_keeps_two_multiplicities_apart() {
    let positions = [[0.0, 0.0], [0.0, 0.0], [64.0, 0.0], [64.0, 0.0]];
    let mut config = base("SqGathering", &positions, SchedulerKind::Ssync);
    config.rounds = Some(vec![vec![0, 1], vec![2, 3]]);
    config.schedule_end = ScheduleEnd::Repeat;
    config.max_rounds = Some(10);
    let mut s = sim(&config);
    let mut gap = 64.0;
    for round in 1..=10 {
        play_rounds(&mut s, round);
        gap /= 2.0;
        assert_eq!(census(&s), vec![2, 2], "round {round}");
        let p = &s.robots().pos;
        assert!((libm::fabs(p[0].x - p[2].x) - gap).abs() < 1e-9);
    }
    assert_eq!(run(&mut s), StopReason::MaxRounds);

    let sequential = SimConfig {
        scheduler: SchedulerKind::Sequential,
        rounds: None,
        schedule_end: ScheduleEnd::RoundRobin,
        max_rounds: None,
        ..config
    };
    let mut s = sim(&sequential);
    assert_eq!(run(&mut s), StopReason::Ended);
    assert_eq!(census(&s), vec![4]);
}

/// Theorem 4: robots 0 and 1 share A, robot 2 is alone at B. Without
/// multiplicity detection every robot sees two points and goes to the other
/// one; the fair schedule 0, 2, 1 (repeated) always moves a robot off the
/// shared point, so after every turn there are again two points. With weak
/// detection the same schedule gathers.
#[test]
fn theorem_4_no_multiplicity_detection_never_gathers() {
    let positions = [[0.0, 0.0], [0.0, 0.0], [5.0, 0.0]];
    let mut config = base("SqGathering", &positions, SchedulerKind::Sequential);
    config.multiplicity = MultiplicityModel::None;
    config.schedule = Some(vec![
        Activation::Robot(0),
        Activation::Robot(2),
        Activation::Robot(1),
    ]);
    config.schedule_end = ScheduleEnd::Repeat;
    config.max_turns = Some(300);
    let mut s = sim(&config);
    while s.step_checked().is_ok() {
        let mut c = census(&s);
        c.sort_unstable();
        // Mid-move a robot is between the points: count only settled turns.
        if s.robots()
            .state
            .iter()
            .all(|st| *st != lcm_core::RobotState::Move)
        {
            assert_eq!(c, vec![1, 2], "turn {}", s.turn_count());
        }
    }
    assert!(s.turn_limit_reached());

    config.multiplicity = MultiplicityModel::Weak;
    let mut s = sim(&config);
    assert_eq!(run(&mut s), StopReason::Ended);
    assert_eq!(census(&s), vec![3]);
}

#[test]
fn an_await_schedule_waits_for_each_turn() {
    let positions = [[0.0, 0.0], [8.0, 0.0], [0.0, 6.0]];
    let mut config = base("Gathering", &positions, SchedulerKind::Sequential);
    config.schedule = Some(vec![]);
    config.schedule_end = ScheduleEnd::Await;
    let mut s = sim(&config);
    assert_eq!(run(&mut s), StopReason::AwaitingSchedule);
    assert!(s.awaiting_schedule());
    let before = s.robots().pos.clone();
    let preview = s.preview_target(1).unwrap();
    assert_eq!(s.robots().pos, before, "a preview changes nothing");
    s.push_turn(1, None).unwrap();
    assert_eq!(run(&mut s), StopReason::AwaitingSchedule);
    assert!(close(s.robots().pos[1], preview));
    assert_eq!(s.robots().pos[0], before[0]);
    assert!(s.push_turn(9, None).is_err());
    assert!(s.push_round(vec![0]).is_err(), "not an SSYNC run");
}

#[test]
fn an_await_schedule_can_name_the_stopping_point() {
    let positions = [[0.0, 0.0], [10.0, 0.0]];
    let mut config = base("SqGathering", &positions, SchedulerKind::Sequential);
    config.schedule = Some(vec![]);
    config.schedule_end = ScheduleEnd::Await;
    config.rigid_movement = false;
    config.delta = Some(1.0);
    let mut s = sim(&config);
    run(&mut s);
    s.push_turn(0, Some(0.25)).unwrap();
    assert_eq!(run(&mut s), StopReason::AwaitingSchedule);
    assert!(close(s.robots().pos[0], Point::new(2.5, 0.0)));
}

#[test]
fn an_await_ssync_run_plays_the_rounds_it_is_given() {
    let positions = [[0.0, 0.0], [0.0, 0.0], [64.0, 0.0], [64.0, 0.0]];
    let mut config = base("SqGathering", &positions, SchedulerKind::Ssync);
    config.rounds = Some(vec![]);
    config.schedule_end = ScheduleEnd::Await;
    let mut s = sim(&config);
    assert_eq!(run(&mut s), StopReason::AwaitingSchedule);
    s.push_round(vec![0, 1]).unwrap();
    assert_eq!(run(&mut s), StopReason::AwaitingSchedule);
    assert!(close(s.robots().pos[0], Point::new(32.0, 0.0)));
    assert!(close(s.robots().pos[1], Point::new(32.0, 0.0)));
    assert_eq!(s.round_count(), 1);
}

#[test]
fn private_frames_do_not_change_a_frame_invariant_run() {
    // The centre of gravity does not depend on the frame: a disoriented run
    // takes the same events and ends in the same place, up to rounding.
    let positions = [[0.0, 0.0], [10.0, 0.0], [0.0, 6.0], [7.0, 9.0], [-3.0, 4.0]];
    let global = base("Gathering", &positions, SchedulerKind::Sequential);
    for frames in [Frames::Rotated, Frames::Disoriented] {
        let local = SimConfig {
            frames,
            ..global.clone()
        };
        let (mut a, mut b) = (sim(&global), sim(&local));
        for _ in 0..200 {
            let (x, y) = (a.step(), b.step());
            assert_eq!((x.robot, x.kind, x.outcome), (y.robot, y.kind, y.outcome));
        }
        for (p, q) in a.robots().pos.iter().zip(&b.robots().pos) {
            assert!((p.x - q.x).abs() < 1e-6 && (p.y - q.y).abs() < 1e-6);
        }
    }
}

#[test]
fn a_robot_in_line_hides_the_one_behind_it() {
    let positions = [[0.0, 0.0], [5.0, 0.0], [10.0, 0.0], [5.0, 5.0]];
    let mut config = base("Gathering", &positions, SchedulerKind::Sequential);
    config.schedule = Some(vec![]);
    config.schedule_end = ScheduleEnd::Await;
    let mut open = sim(&config);
    // Robot 0 sees all four: centroid (5, 1.25).
    assert!(close(
        open.preview_target(0).unwrap(),
        Point::new(5.0, 1.25)
    ));
    config.obstructed_visibility = true;
    let mut blocked = sim(&config);
    // Robot 1 hides robot 2: robot 0 sees 0, 1, 3.
    assert!(close(
        blocked.preview_target(0).unwrap(),
        Point::new(10.0 / 3.0, 5.0 / 3.0)
    ));
    // A point robot just off the line does not block; a disc of radius 0.5 does.
    config.initial_positions = Some(vec![[0.0, 0.0], [5.0, 0.4], [10.0, 0.0], [5.0, 5.0]]);
    let mut points = sim(&config);
    assert!(close(
        points.preview_target(0).unwrap(),
        Point::new(5.0, 5.4 / 4.0)
    ));
    config.robot_radius = 0.5;
    let mut discs = sim(&config);
    assert!(close(
        discs.preview_target(0).unwrap(),
        Point::new(10.0 / 3.0, 5.4 / 3.0)
    ));
}

#[test]
fn strong_detection_counts_and_none_hides_multiplicities() {
    use lcm_core::occupancy::perceive;
    let p = |x, y| Point::new(x, y);
    let positions = [p(0.0, 0.0), p(0.0, 0.0), p(0.0, 0.0), p(4.0, 0.0)];
    let (strong, me) = perceive(&positions, 3, 1e-9, MultiplicityModel::Strong);
    assert_eq!(me, 1);
    assert_eq!(
        strong.iter().map(|o| o.count).collect::<Vec<_>>(),
        vec![Some(3), Some(1)]
    );
    let (weak, _) = perceive(&positions, 3, 1e-9, MultiplicityModel::Weak);
    assert_eq!(
        weak.iter()
            .map(|o| (o.multiple, o.count))
            .collect::<Vec<_>>(),
        vec![(true, None), (false, None)]
    );
    let (none, _) = perceive(&positions, 3, 1e-9, MultiplicityModel::None);
    assert!(none.iter().all(|o| !o.multiple && o.count.is_none()));
}

#[test]
fn local_detection_sees_only_its_own_point() {
    use lcm_core::occupancy::perceive;
    let p = |x, y| Point::new(x, y);
    let positions = [
        p(0.0, 0.0),
        p(0.0, 0.0),
        p(4.0, 0.0),
        p(4.0, 0.0),
        p(9.0, 0.0),
    ];
    let (from_pair, _) = perceive(&positions, 0, 1e-9, MultiplicityModel::Local);
    assert_eq!(
        from_pair.iter().map(|o| o.multiple).collect::<Vec<_>>(),
        vec![true, false, false],
        "its own point only"
    );
    let (from_single, _) = perceive(&positions, 4, 1e-9, MultiplicityModel::Local);
    assert!(from_single.iter().all(|o| !o.multiple && o.count.is_none()));
}

/// SqGathering with only local detection: robots on each of two
/// multiplicities see kappa = 1 and hold, so the two never merge, while weak
/// detection gathers the same start.
#[test]
fn sq_gathering_with_local_detection_keeps_two_multiplicities() {
    let positions = [[0.0, 0.0], [0.0, 0.0], [64.0, 0.0], [64.0, 0.0]];
    let mut config = base("SqGathering", &positions, SchedulerKind::Sequential);
    config.multiplicity = MultiplicityModel::Local;
    let mut s = sim(&config);
    assert_eq!(run(&mut s), StopReason::Stalled);
    assert_eq!(census(&s), vec![2, 2]);
    config.multiplicity = MultiplicityModel::Weak;
    let mut s = sim(&config);
    assert_eq!(run(&mut s), StopReason::Ended);
}
