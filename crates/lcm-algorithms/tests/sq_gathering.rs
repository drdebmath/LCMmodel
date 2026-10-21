//! Algorithm 8 (`SqGathering`) on the core's sequential scheduler.
//!
//! The first tests are the hand traces of `docs/sqgathering.md`, turn by
//! turn: κ = 0, κ = 1, κ > 1, and a non-rigid stop that lands on a robot.
//! The rest are properties over many seeded random runs, and the settings
//! the algorithm must refuse.

use lcm_algorithms::sq_gathering::{decide, decide_in_frame, observe, LocalFrame, Rule};
use lcm_algorithms::sq_trace::{Report, Round, Status, Tracer};
use lcm_core::geom::dist;
use lcm_core::{
    Activation, ActivationOrder, ConfigError, Point, Rng, ScheduleEnd, SchedulerKind, SimConfig,
    StopPolicy, TurnGap,
};

fn close(a: [f64; 2], b: [f64; 2]) -> bool {
    (a[0] - b[0]).abs() <= 1e-12 && (a[1] - b[1]).abs() <= 1e-12
}

/// A sequential SqGathering run: these positions, this explicit schedule
/// (then round-robin), non-rigid moves stopped right after δ unless a turn
/// names its own stopping point.
fn config(positions: &[[f64; 2]], schedule: &[Activation], delta: f64) -> SimConfig {
    SimConfig {
        algorithm: "SqGathering".to_owned(),
        num_of_robots: u32::try_from(positions.len()).unwrap(),
        initial_positions: Some(positions.to_vec()),
        scheduler: SchedulerKind::Sequential,
        turn_gap: TurnGap::None,
        rigid_movement: false,
        delta: Some(delta),
        stop_policy: StopPolicy::Delta,
        schedule: (!schedule.is_empty()).then(|| schedule.to_vec()),
        schedule_end: ScheduleEnd::RoundRobin,
        max_turns: Some(100_000),
        ..SimConfig::default()
    }
}

const fn at(robot: usize, stop: f64) -> Activation {
    Activation::Stopped { robot, stop }
}

const fn r(robot: usize) -> Activation {
    Activation::Robot(robot)
}

fn trace(config: &SimConfig) -> (Vec<Round>, Report) {
    let mut tracer = Tracer::new(config).expect("valid config");
    let mut rounds = Vec::new();
    let report = tracer.run_with(|r| rounds.push(r.clone()));
    (rounds, report)
}

/// Checks one traced turn: robot, κ, b, rule, destination, stop, reached.
#[allow(clippy::too_many_arguments)]
fn expect(
    round: &Round,
    robot: usize,
    kappa: usize,
    b: bool,
    rule: Rule,
    destination: [f64; 2],
    stop: [f64; 2],
    reached: bool,
) {
    let n = round.turn;
    assert_eq!(round.robot, robot, "turn {n}: active robot");
    assert_eq!(round.kappa, kappa, "turn {n}: κ");
    assert_eq!(round.on_multiplicity, b, "turn {n}: b");
    assert_eq!(round.rule, rule.key(), "turn {n}: rule");
    assert!(
        close(round.destination, destination),
        "turn {n}: destination {:?}",
        round.destination
    );
    assert!(close(round.stop, stop), "turn {n}: stop {:?}", round.stop);
    assert_eq!(round.reached, reached, "turn {n}: reached");
}

#[test]
fn hand_trace_a_no_multiplicity() {
    let (rounds, report) = trace(&config(
        &[[0.0, 0.0], [4.0, 0.0], [4.0, 3.0], [10.0, 0.0]],
        &[at(0, 0.5), at(1, 1.0)],
        1.0,
    ));
    let a = &rounds[0];
    assert!(a.observation.iter().all(|p| !p.multiplicity));
    expect(
        a,
        0,
        0,
        false,
        Rule::MoveToClosest,
        [4.0, 0.0],
        [2.0, 0.0],
        false,
    );
    assert_eq!((a.distinct_after, a.multiplicities_after), (4, 0));
    expect(
        &rounds[1],
        1,
        0,
        false,
        Rule::MoveToClosest,
        [2.0, 0.0],
        [2.0, 0.0],
        true,
    );
    assert_eq!(
        (rounds[1].distinct_after, rounds[1].multiplicities_after),
        (3, 1)
    );
    // From here on κ = 1: robots 0 and 1 hold, the others join.
    assert!(rounds[2..].iter().all(|r| r.kappa == 1));
    expect(
        &rounds[4],
        2,
        1,
        false,
        Rule::JoinMultiplicity,
        [2.0, 0.0],
        [4.0 - 2.0 / 13f64.sqrt(), 3.0 - 3.0 / 13f64.sqrt()],
        false,
    );
    assert_eq!(report.status, Status::Gathered);
    assert!(close(report.gathered_at.unwrap(), [2.0, 0.0]));
}

#[test]
fn hand_trace_b_one_multiplicity() {
    let (rounds, report) = trace(&config(
        &[[0.0, 0.0], [0.0, 0.0], [3.0, 4.0], [-6.0, 0.0]],
        &[r(2), r(0), at(3, 0.5), r(2), r(2), at(3, 1.0)],
        2.0,
    ));
    let first = &rounds[0];
    assert_eq!(
        first.observation.len(),
        3,
        "the two robots at (0,0) are one point"
    );
    assert!(first.observation[0].multiplicity && first.observation[0].count == 2);
    expect(
        first,
        2,
        1,
        false,
        Rule::JoinMultiplicity,
        [0.0, 0.0],
        [1.8, 2.4],
        false,
    );
    expect(
        &rounds[1],
        0,
        1,
        true,
        Rule::HoldMultiplicity,
        [0.0, 0.0],
        [0.0, 0.0],
        true,
    );
    expect(
        &rounds[2],
        3,
        1,
        false,
        Rule::JoinMultiplicity,
        [0.0, 0.0],
        [-3.0, 0.0],
        false,
    );
    expect(
        &rounds[3],
        2,
        1,
        false,
        Rule::JoinMultiplicity,
        [0.0, 0.0],
        [0.6, 0.8],
        false,
    );
    expect(
        &rounds[4],
        2,
        1,
        false,
        Rule::JoinMultiplicity,
        [0.0, 0.0],
        [0.0, 0.0],
        true,
    );
    expect(
        &rounds[5],
        3,
        1,
        false,
        Rule::JoinMultiplicity,
        [0.0, 0.0],
        [0.0, 0.0],
        true,
    );
    assert_eq!(report.gathered_after, Some(6));
    // Then every robot looks once more, sees one point, and terminates.
    assert!(rounds[6..]
        .iter()
        .all(|r| r.terminated && r.rule == Rule::HoldMultiplicity.key()));
    assert_eq!((report.status, report.turns), (Status::Gathered, 10));
}

#[test]
fn hand_trace_c_two_multiplicities() {
    let (rounds, report) = trace(&config(
        &[[0.0, 0.0], [0.0, 0.0], [6.0, 0.0], [6.0, 0.0], [0.0, 8.0]],
        &[r(4), r(0), at(1, 1.0), at(0, 1.0), at(4, 1.0)],
        1.0,
    ));
    expect(
        &rounds[0],
        4,
        2,
        false,
        Rule::WaitForSeparation,
        [0.0, 8.0],
        [0.0, 8.0],
        true,
    );
    expect(
        &rounds[1],
        0,
        2,
        true,
        Rule::LeaveMultiplicity,
        [3.0, 0.0],
        [1.0, 0.0],
        false,
    );
    assert_eq!(
        rounds[1].multiplicities_after, 1,
        "(0,0) is now a single robot"
    );
    expect(
        &rounds[2],
        1,
        1,
        false,
        Rule::JoinMultiplicity,
        [6.0, 0.0],
        [6.0, 0.0],
        true,
    );
    expect(
        &rounds[3],
        0,
        1,
        false,
        Rule::JoinMultiplicity,
        [6.0, 0.0],
        [6.0, 0.0],
        true,
    );
    expect(
        &rounds[4],
        4,
        1,
        false,
        Rule::JoinMultiplicity,
        [6.0, 0.0],
        [6.0, 0.0],
        true,
    );
    assert_eq!(report.gathered_after, Some(5));
    assert_eq!(report.status, Status::Gathered);
    assert!(close(report.gathered_at.unwrap(), [6.0, 0.0]));
}

#[test]
fn hand_trace_d_nonrigid_stop_creates_a_second_multiplicity() {
    let (rounds, report) = trace(&config(
        &[[0.0, 0.0], [0.0, 0.0], [2.0, 0.0], [4.0, 0.0]],
        &[at(3, 0.5), r(0), at(1, 1.0)],
        1.0,
    ));
    // Robot 3 heads for (0,0) and the adversary stops it on robot 2.
    expect(
        &rounds[0],
        3,
        1,
        false,
        Rule::JoinMultiplicity,
        [0.0, 0.0],
        [2.0, 0.0],
        false,
    );
    assert_eq!(rounds[0].multiplicities_after, 2);
    // κ = 2: robot 0 leaves (0,0) for the midpoint towards (2,0).
    expect(
        &rounds[1],
        0,
        2,
        true,
        Rule::LeaveMultiplicity,
        [1.0, 0.0],
        [1.0, 0.0],
        true,
    );
    assert_eq!(rounds[1].multiplicities_after, 1);
    // κ = 1 again: robot 1 (now alone at (0,0)) and robot 0 join (2,0).
    expect(
        &rounds[2],
        1,
        1,
        false,
        Rule::JoinMultiplicity,
        [2.0, 0.0],
        [2.0, 0.0],
        true,
    );
    expect(
        &rounds[3],
        0,
        1,
        false,
        Rule::JoinMultiplicity,
        [2.0, 0.0],
        [2.0, 0.0],
        true,
    );
    assert_eq!(report.gathered_after, Some(4));
    assert!(close(report.gathered_at.unwrap(), [2.0, 0.0]));
}

#[test]
fn minimal_progress_can_collide_twice_and_still_gathers() {
    // Same start, but the adversary always stops after exactly δ: robot 1
    // also lands on a single robot, making κ = 2 a second time.
    let (rounds, report) = trace(&config(
        &[[0.0, 0.0], [0.0, 0.0], [2.0, 0.0], [4.0, 0.0]],
        &[at(3, 0.5)],
        1.0,
    ));
    let kappas: Vec<usize> = rounds[..8].iter().map(|r| r.kappa).collect();
    assert_eq!(kappas, [1, 2, 1, 2, 1, 1, 1, 1]);
    expect(
        &rounds[2],
        1,
        1,
        false,
        Rule::JoinMultiplicity,
        [2.0, 0.0],
        [1.0, 0.0],
        false,
    );
    expect(
        &rounds[3],
        2,
        2,
        true,
        Rule::LeaveMultiplicity,
        [1.5, 0.0],
        [1.5, 0.0],
        true,
    );
    assert_eq!(report.gathered_after, Some(8));
    assert!(close(report.gathered_at.unwrap(), [1.0, 0.0]));
}

#[test]
fn rendezvous_of_two_robots_under_minimal_progress() {
    let (rounds, report) = trace(&config(&[[0.0, 0.0], [10.5, 0.0]], &[], 1.0));
    assert!(rounds[..11]
        .iter()
        .all(|r| r.rule == Rule::MoveToClosest.key()));
    // Each turn closes the gap by exactly δ until it is at most δ: 10 + 1.
    assert_eq!(report.gathered_after, Some(11));
    assert_eq!(report.status, Status::Gathered);
}

#[test]
fn a_single_robot_and_a_gathered_start_end_at_once() {
    for positions in [vec![[3.0, 3.0]], vec![[1.0, 1.0]; 4]] {
        let (rounds, report) = trace(&config(&positions, &[], 1.0));
        assert_eq!(report.status, Status::Gathered);
        assert_eq!(report.gathered_after, Some(0));
        assert!(
            rounds.iter().all(|r| r.terminated),
            "every turn is a robot terminating"
        );
    }
}

#[test]
fn a_turn_limited_run_is_labelled_incomplete() {
    let mut cfg = config(&[[0.0, 0.0], [100.0, 0.0], [0.0, 100.0]], &[], 1.0);
    cfg.max_turns = Some(5);
    let (rounds, report) = trace(&cfg);
    assert_eq!(rounds.len(), 5);
    assert_eq!(
        (report.status, report.stopped.as_str()),
        (Status::Incomplete, "MaxTurns")
    );
    assert!(report.gathered_at.is_none());
    assert!(
        report.note.starts_with("INCOMPLETE (budget-limited)"),
        "{}",
        report.note
    );
}

#[test]
fn weak_detection_two_or_four_robots_look_the_same() {
    let p = |c: &[[f64; 2]]| c.iter().map(|q| Point::new(q[0], q[1])).collect::<Vec<_>>();
    let two = p(&[[0.0, 0.0], [0.0, 0.0], [5.0, 1.0]]);
    let four = p(&[[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [5.0, 1.0]]);
    let a = observe(&two, 2, 1e-9);
    let b = observe(&four, 4, 1e-9);
    assert_eq!(a, b);
    assert_eq!(decide(&a), decide(&b));
}

#[test]
fn settings_outside_the_model_are_refused() {
    let base = config(&[[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]], &[], 1.0);
    let message = |c: SimConfig| match lcm_algorithms::simulation(&c) {
        Err(ConfigError::Unsupported(m)) => m,
        Err(e) => panic!("expected Unsupported, got {e}"),
        Ok(_) => panic!("expected a refusal"),
    };
    let asynchronous = SimConfig {
        scheduler: SchedulerKind::Async,
        schedule: None,
        max_turns: None,
        ..base.clone()
    };
    assert!(message(asynchronous).contains("sequential scheduler"));
    let limited = SimConfig {
        visibility_radius: Some(50.0),
        ..base.clone()
    };
    assert!(message(limited).contains("unlimited visibility"));
    let faulty = SimConfig {
        num_of_faults: 1,
        ..base.clone()
    };
    assert!(message(faulty).contains("faulty"));
    let other = SimConfig {
        algorithm: "Gathering".to_owned(),
        ..base
    };
    assert!(
        Tracer::new(&other).is_err(),
        "the trace is for SqGathering only"
    );
}

#[test]
fn a_sequential_run_from_json_like_the_page_sends() {
    let text = r#"{
        "algorithm": "SqGathering", "scheduler": "sequential", "num_of_robots": 4,
        "initial_positions": [[0,0],[0,0],[3,4],[-6,0]],
        "rigid_movement": false, "delta": 2, "stop_policy": "delta",
        "schedule": [2, 0, {"robot": 3, "stop": 0.5}], "schedule_end": "round_robin",
        "turn_gap": "none"
    }"#;
    let cfg: SimConfig = serde_json::from_str(text).unwrap();
    let (rounds, report) = trace(&cfg);
    expect(
        &rounds[2],
        3,
        1,
        false,
        Rule::JoinMultiplicity,
        [0.0, 0.0],
        [-3.0, 0.0],
        false,
    );
    assert_eq!(report.status, Status::Gathered);
}

// ── properties over random runs ─────────────────────────────────────────────

/// A random start with integer coordinates and some stacked robots.
fn random_positions(rng: &mut Rng) -> Vec<[f64; 2]> {
    let n = 2 + rng.index(8);
    let mut positions: Vec<[f64; 2]> = Vec::with_capacity(n);
    for _ in 0..n {
        if !positions.is_empty() && rng.random() < 0.35 {
            let copy = positions[rng.index(positions.len())];
            positions.push(copy);
        } else {
            let x = rng.uniform(-40.0, 40.0).round();
            let y = rng.uniform(-40.0, 40.0).round();
            positions.push([x, y]);
        }
    }
    positions
}

fn random_config(seed: u64) -> SimConfig {
    let mut rng = Rng::new(seed);
    let positions = random_positions(&mut rng);
    let mut cfg = config(&positions, &[], 1.0);
    cfg.random_seed = seed;
    cfg.activation_order = if rng.random() < 0.5 {
        ActivationOrder::Random
    } else {
        ActivationOrder::RoundRobin
    };
    match rng.index(3) {
        0 => {
            cfg.rigid_movement = true;
            cfg.delta = None;
        }
        1 => cfg.delta = Some(rng.uniform(0.5, 3.0)),
        _ => {
            cfg.delta = Some(rng.uniform(0.5, 3.0));
            cfg.stop_policy = StopPolicy::Random;
        }
    }
    cfg.max_turns = Some(200_000);
    cfg
}

#[test]
fn every_random_run_gathers_and_the_core_moves_where_the_rule_says() {
    for seed in 0..300 {
        let cfg = random_config(seed);
        let mut tracer = Tracer::new(&cfg).unwrap();
        while let Some(round) = tracer.next_round() {
            let r = tracer.simulation().robots();
            if !round.terminated {
                let target = r.target[round.robot].expect("a computed target");
                assert!(
                    close([target.x, target.y], round.destination),
                    "seed {seed} turn {}",
                    round.turn
                );
            }
        }
        let report = tracer.report();
        assert_eq!(report.status, Status::Gathered, "seed {seed}: {cfg:?}");
        assert_eq!(report.distinct, 1);
    }
}

#[test]
fn moves_never_land_on_or_cross_another_robot_except_joins() {
    // κ = 0: nothing lies strictly between q and its closest point.
    // κ > 1: the leaving robot's whole path to the midpoint is unoccupied.
    for seed in 0..150 {
        let cfg = random_config(seed);
        let mut positions: Vec<Point> = cfg
            .initial_positions
            .as_ref()
            .unwrap()
            .iter()
            .map(|p| Point::new(p[0], p[1]))
            .collect();
        let mut tracer = Tracer::new(&cfg).unwrap();
        while let Some(round) = tracer.next_round() {
            let from = Point::new(round.position[0], round.position[1]);
            let stop = Point::new(round.stop[0], round.stop[1]);
            let rule = round.rule.as_str();
            if rule == Rule::LeaveMultiplicity.key() || rule == Rule::MoveToClosest.key() {
                let destination = Point::new(round.destination[0], round.destination[1]);
                let end = if rule == Rule::LeaveMultiplicity.key() {
                    destination
                } else {
                    stop
                };
                let path = dist(from, end);
                for (j, &p) in positions.iter().enumerate() {
                    if j == round.robot || dist(p, from) < 1e-9 || dist(p, destination) < 1e-9 {
                        continue;
                    }
                    let off_path = dist(from, p) + dist(p, end) - path;
                    assert!(
                        off_path > 1e-9,
                        "seed {seed} turn {}: path crosses robot {j}",
                        round.turn
                    );
                }
            }
            positions[round.robot] = stop;
        }
    }
}

#[test]
fn rigid_moves_keep_the_unique_multiplicity_once_kappa_is_one() {
    for seed in 0..150 {
        let mut cfg = random_config(seed);
        cfg.rigid_movement = true;
        cfg.delta = None;
        let mut positions: Vec<Point> = cfg
            .initial_positions
            .as_ref()
            .unwrap()
            .iter()
            .map(|p| Point::new(p[0], p[1]))
            .collect();
        let mut tracer = Tracer::new(&cfg).unwrap();
        let mut anchor: Option<Point> = None;
        while let Some(round) = tracer.next_round() {
            positions[round.robot] = Point::new(round.stop[0], round.stop[1]);
            let mut multis: Vec<Point> = Vec::new();
            for (i, p) in positions.iter().enumerate() {
                let shared = positions
                    .iter()
                    .enumerate()
                    .any(|(j, q)| j != i && dist(*p, *q) < 1e-9);
                if shared && !multis.iter().any(|o| dist(*o, *p) < 1e-9) {
                    multis.push(*p);
                }
            }
            if let Some(a) = anchor {
                assert!(
                    multis.len() == 1 && dist(multis[0], a) < 1e-12,
                    "seed {seed} turn {}",
                    round.turn
                );
            } else if round.multiplicities_after == 1 {
                anchor = Some(multis[0]);
            }
        }
        assert_eq!(tracer.status(), Status::Gathered);
    }
}

#[test]
fn decisions_do_not_depend_on_the_local_frame_without_ties() {
    let mut rng = Rng::new(77);
    for _ in 0..500 {
        let mut positions: Vec<Point> = random_positions(&mut rng)
            .into_iter()
            .map(|p| Point::new(p[0] + rng.uniform(-0.3, 0.3), p[1] + rng.uniform(-0.3, 0.3)))
            .collect();
        if positions.len() > 3 && rng.random() < 0.6 {
            positions[1] = positions[0];
        }
        for i in 0..positions.len() {
            let obs = observe(&positions, i, 1e-9);
            let global = decide(&obs);
            let frame = LocalFrame {
                origin: positions[i],
                angle: rng.uniform(0.0, std::f64::consts::TAU),
                scale: rng.uniform(0.1, 10.0),
                reflect: rng.random() < 0.5,
            };
            assert_eq!(global, decide_in_frame(&obs, &frame));
        }
    }
}

#[test]
fn all_three_cases_and_every_rule_occur_in_random_runs() {
    let mut seen = std::collections::HashSet::new();
    for seed in 0..100 {
        Tracer::new(&random_config(seed)).unwrap().run_with(|r| {
            seen.insert(r.rule.clone());
        });
    }
    for rule in Rule::ALL.iter().filter(|r| **r != Rule::Alone) {
        assert!(seen.contains(rule.key()), "{} never fired", rule.key());
    }
}

#[test]
fn stacked_starts_from_the_core_gather() {
    for (k, size) in [(1, 0), (2, 0), (3, 4), (6, 5), (10, 2)] {
        let cfg = SimConfig {
            algorithm: "SqGathering".to_owned(),
            scheduler: SchedulerKind::Sequential,
            num_of_robots: 60,
            multiplicities: k,
            multiplicity_size: size,
            activation_order: ActivationOrder::Random,
            max_turns: Some(1_000_000),
            ..SimConfig::default()
        };
        let mut tracer = Tracer::new(&cfg).unwrap();
        assert_eq!(tracer.census().1, k as usize, "κ = {k} at the start");
        let report = tracer.run();
        assert_eq!(report.status, Status::Gathered, "κ = {k}, size {size}");
    }
}

#[test]
fn large_multiplicities_under_kappa_many_stall_at_the_precision_limit() {
    // Two multiplicities of 60: robots leaving one of them halve their
    // distance each time and get closer than ε, so nothing can move.
    let mut positions = vec![[0.0, 0.0]; 60];
    positions.extend(vec![[100.0, 0.0]; 60]);
    let mut cfg = config(&positions, &[], 1.0);
    cfg.rigid_movement = true;
    cfg.delta = None;
    let report = Tracer::new(&cfg).unwrap().run();
    assert_eq!(
        (report.status, report.stopped.as_str()),
        (Status::Stalled, "Stalled")
    );
    assert!(
        report.note.starts_with("STALLED (precision limit)"),
        "{}",
        report.note
    );
    assert!(
        report.turns < 2_000,
        "detected after an epoch, not the budget: {}",
        report.turns
    );
}
