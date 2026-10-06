//! Algorithm 8 (`SqGathering`) under the sequential scheduler.
//!
//! The first four tests are the hand traces of `docs/sqgathering.md`, round
//! by round: κ = 0, κ = 1, κ > 1, and a non-rigid stop that creates a second
//! multiplicity. The rest are properties over many seeded random runs.

use lcm_algorithms::seq::{
    Activation, Fallback, FrameSpec, MovementSpec, Round, ScheduleSpec, SeqConfig, SqGatheringRun,
    Status, StopPolicy,
};
use lcm_algorithms::sq_gathering::{decide, decide_in_frame, observe, LocalFrame, Rule};
use lcm_core::geom::dist;
use lcm_core::{Point, Rng};

fn close(a: [f64; 2], b: [f64; 2]) -> bool {
    (a[0] - b[0]).abs() <= 1e-12 && (a[1] - b[1]).abs() <= 1e-12
}

fn nonrigid(delta: f64) -> MovementSpec {
    MovementSpec::NonRigid {
        delta,
        stop: StopPolicy::Delta,
    }
}

fn explicit(order: &[Activation]) -> ScheduleSpec {
    ScheduleSpec::Explicit {
        order: order.to_vec(),
        then: Fallback::RoundRobin,
    }
}

const fn at(robot: usize, stop: f64) -> Activation {
    Activation::Stopped { robot, stop }
}

const fn r(robot: usize) -> Activation {
    Activation::Robot(robot)
}

fn config(positions: &[[f64; 2]], schedule: ScheduleSpec, movement: MovementSpec) -> SeqConfig {
    SeqConfig {
        positions: positions.to_vec(),
        schedule,
        movement,
        ..SeqConfig::default()
    }
}

fn trace(config: SeqConfig) -> (Vec<Round>, lcm_algorithms::seq::Report) {
    let mut run = SqGatheringRun::new(config).expect("valid config");
    let mut rounds = Vec::new();
    let report = run.run_with(|r| rounds.push(r.clone()));
    (rounds, report)
}

/// Checks one traced round: robot, κ, b, rule, destination, stop, reached.
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
    let n = round.round;
    assert_eq!(round.robot, robot, "round {n}: active robot");
    assert_eq!(round.kappa, kappa, "round {n}: κ");
    assert_eq!(round.on_multiplicity, b, "round {n}: b");
    assert_eq!(round.rule, rule.key(), "round {n}: rule");
    assert!(
        close(round.destination, destination),
        "round {n}: destination {:?}",
        round.destination
    );
    assert!(close(round.stop, stop), "round {n}: stop {:?}", round.stop);
    assert_eq!(round.reached, reached, "round {n}: reached");
}

#[test]
fn hand_trace_a_no_multiplicity() {
    let (rounds, report) = trace(config(
        &[[0.0, 0.0], [4.0, 0.0], [4.0, 3.0], [10.0, 0.0]],
        explicit(&[at(0, 0.5), at(1, 1.0)]),
        nonrigid(1.0),
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
    assert_eq!(report.status, Status::Gathered);
    assert!(close(report.gathered_at.unwrap(), [2.0, 0.0]));
}

#[test]
fn hand_trace_b_one_multiplicity() {
    let (rounds, report) = trace(config(
        &[[0.0, 0.0], [0.0, 0.0], [3.0, 4.0], [-6.0, 0.0]],
        explicit(&[r(2), r(0), at(3, 0.5), r(2), r(2), at(3, 1.0)]),
        nonrigid(2.0),
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
    assert_eq!((report.status, report.rounds), (Status::Gathered, 6));
}

#[test]
fn hand_trace_c_two_multiplicities() {
    let (rounds, report) = trace(config(
        &[[0.0, 0.0], [0.0, 0.0], [6.0, 0.0], [6.0, 0.0], [0.0, 8.0]],
        explicit(&[r(4), r(0), at(1, 1.0), at(0, 1.0), at(4, 1.0)]),
        nonrigid(1.0),
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
    assert_eq!((report.status, report.rounds), (Status::Gathered, 5));
    assert!(close(report.gathered_at.unwrap(), [6.0, 0.0]));
}

#[test]
fn hand_trace_d_nonrigid_stop_creates_a_second_multiplicity() {
    let (rounds, report) = trace(config(
        &[[0.0, 0.0], [0.0, 0.0], [2.0, 0.0], [4.0, 0.0]],
        explicit(&[at(3, 0.5), r(0), at(1, 1.0)]),
        nonrigid(1.0),
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
    assert_eq!((report.status, report.rounds), (Status::Gathered, 4));
    assert!(close(report.gathered_at.unwrap(), [2.0, 0.0]));
}

#[test]
fn minimal_progress_can_collide_twice_and_still_gathers() {
    // Same start, but the adversary always stops after exactly δ: robot 1
    // also lands on a singleton, making κ = 2 a second time.
    let (rounds, report) = trace(config(
        &[[0.0, 0.0], [0.0, 0.0], [2.0, 0.0], [4.0, 0.0]],
        explicit(&[at(3, 0.5)]),
        nonrigid(1.0),
    ));
    let kappas: Vec<usize> = rounds.iter().map(|r| r.kappa).collect();
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
    assert_eq!((report.status, report.rounds), (Status::Gathered, 8));
    assert!(close(report.gathered_at.unwrap(), [1.0, 0.0]));
}

#[test]
fn rendezvous_of_two_robots_under_minimal_progress() {
    let (rounds, report) = trace(config(
        &[[0.0, 0.0], [10.5, 0.0]],
        ScheduleSpec::RoundRobin,
        nonrigid(1.0),
    ));
    assert!(rounds.iter().all(|r| r.rule == Rule::MoveToClosest.key()));
    assert_eq!(report.status, Status::Gathered);
    // Each round closes the gap by exactly δ until it is at most δ: 10 + 1.
    assert_eq!(report.rounds, 11);
}

#[test]
fn a_single_robot_and_a_gathered_start_are_done_at_round_zero() {
    for positions in [vec![[3.0, 3.0]], vec![[1.0, 1.0]; 4]] {
        let mut run =
            SqGatheringRun::new(config(&positions, ScheduleSpec::RoundRobin, nonrigid(1.0)))
                .unwrap();
        assert_eq!(run.status(), Status::Gathered);
        assert!(run.step().is_none());
        assert_eq!(run.report().rounds, 0);
    }
}

#[test]
fn a_budget_limited_run_is_labelled_incomplete() {
    let mut cfg = config(
        &[[0.0, 0.0], [100.0, 0.0], [0.0, 100.0]],
        ScheduleSpec::RoundRobin,
        nonrigid(1.0),
    );
    cfg.max_rounds = 5;
    let (rounds, report) = trace(cfg);
    assert_eq!(rounds.len(), 5);
    assert_eq!(report.status, Status::Incomplete);
    assert!(report.gathered_at.is_none());
    assert!(report.note.starts_with("INCOMPLETE"), "{}", report.note);
}

#[test]
fn weak_detection_two_or_four_robots_look_the_same() {
    let two = [[0.0, 0.0], [0.0, 0.0], [5.0, 1.0]];
    let four = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [5.0, 1.0]];
    let p = |c: &[[f64; 2]]| c.iter().map(|q| Point::new(q[0], q[1])).collect::<Vec<_>>();
    let a = observe(&p(&two), 2, 1e-9);
    let b = observe(&p(&four), 4, 1e-9);
    assert_eq!(a, b);
    assert_eq!(decide(&a), decide(&b));
}

#[test]
fn invalid_schedules_are_rejected() {
    let base = |schedule| {
        config(
            &[[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]],
            schedule,
            nonrigid(1.0),
        )
    };
    let bad_robot = base(explicit(&[r(7)]));
    assert!(SqGatheringRun::new(bad_robot).is_err());
    let unfair = base(ScheduleSpec::Explicit {
        order: vec![r(0), r(1)],
        then: Fallback::Repeat,
    });
    let message = SqGatheringRun::new(unfair).err().unwrap().0;
    assert!(message.contains("robot 2 is never activated"), "{message}");
    let mut wrong = base(ScheduleSpec::RoundRobin);
    wrong.algorithm = "Gathering".to_owned();
    assert!(
        SqGatheringRun::new(wrong).is_err(),
        "the generic Gathering is not this algorithm"
    );
    let bad_delta = config(
        &[[0.0, 0.0], [1.0, 0.0]],
        ScheduleSpec::RoundRobin,
        nonrigid(0.0),
    );
    assert!(SqGatheringRun::new(bad_delta).is_err());
}

#[test]
fn config_json_round_trips() {
    let text = r#"{
        "algorithm": "SqGathering",
        "positions": [[0,0],[0,0],[3,4],[-6,0]],
        "schedule": {"kind": "explicit", "order": [2, 0, {"robot": 3, "stop": 0.5}], "then": "repeat"},
        "movement": {"kind": "non-rigid", "delta": 2, "stop": {"kind": "random", "seed": 9}},
        "frames": {"kind": "random", "seed": 4},
        "max_rounds": 50
    }"#;
    let cfg: SeqConfig = serde_json::from_str(text).unwrap();
    assert_eq!(cfg.positions.len(), 4);
    assert!(matches!(
        cfg.schedule,
        ScheduleSpec::Explicit {
            then: Fallback::Repeat,
            ..
        }
    ));
    let again: SeqConfig = serde_json::from_str(&serde_json::to_string(&cfg).unwrap()).unwrap();
    assert_eq!(cfg, again);
    // Robot 1 is never named, so this repeating schedule is unfair.
    assert!(SqGatheringRun::new(cfg).is_err());
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

fn random_config(seed: u64) -> SeqConfig {
    let mut rng = Rng::new(seed);
    let positions = random_positions(&mut rng);
    let schedule = if rng.random() < 0.5 {
        ScheduleSpec::Shuffled { seed: seed ^ 0xA5 }
    } else {
        ScheduleSpec::RoundRobin
    };
    let movement = match rng.index(3) {
        0 => MovementSpec::Rigid,
        1 => nonrigid(rng.uniform(0.5, 3.0)),
        _ => MovementSpec::NonRigid {
            delta: rng.uniform(0.5, 3.0),
            stop: StopPolicy::Random { seed: seed ^ 0x5A },
        },
    };
    SeqConfig {
        positions,
        schedule,
        movement,
        frames: FrameSpec::Random { seed: seed ^ 0x33 },
        max_rounds: 200_000,
        ..SeqConfig::default()
    }
}

#[test]
fn every_random_run_gathers() {
    for seed in 0..300 {
        let cfg = random_config(seed);
        let report = SqGatheringRun::new(cfg.clone()).unwrap().run();
        assert_eq!(report.status, Status::Gathered, "seed {seed}: {cfg:?}");
        assert_eq!(report.distinct, 1);
    }
}

#[test]
fn moves_never_land_on_or_cross_another_robot_except_joins() {
    // κ = 0: nothing lies strictly between q and its closest point.
    // κ > 1: the leaving robot's whole path to the midpoint is unoccupied.
    for seed in 0..150 {
        let mut run = SqGatheringRun::new(random_config(seed)).unwrap();
        loop {
            let before: Vec<Point> = run.positions().to_vec();
            let Some(round) = run.step() else { break };
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
                for (j, &p) in before.iter().enumerate() {
                    if j == round.robot || dist(p, from) < 1e-9 || dist(p, destination) < 1e-9 {
                        continue;
                    }
                    let off_path = dist(from, p) + dist(p, end) - path;
                    assert!(
                        off_path > 1e-9,
                        "seed {seed} round {}: path crosses robot {j}",
                        round.round
                    );
                }
            }
        }
    }
}

#[test]
fn rigid_moves_keep_the_unique_multiplicity_once_kappa_is_one() {
    for seed in 0..150 {
        let mut cfg = random_config(seed);
        cfg.movement = MovementSpec::Rigid;
        let mut run = SqGatheringRun::new(cfg).unwrap();
        let mut anchor: Option<[f64; 2]> = None;
        while let Some(round) = run.step() {
            let multis: Vec<[f64; 2]> = {
                let pos = run.positions();
                let mut out: Vec<[f64; 2]> = Vec::new();
                for (i, p) in pos.iter().enumerate() {
                    let shared = pos
                        .iter()
                        .enumerate()
                        .any(|(j, q)| j != i && dist(*p, *q) < 1e-9);
                    if shared && !out.iter().any(|o| dist(Point::new(o[0], o[1]), *p) < 1e-9) {
                        out.push([p.x, p.y]);
                    }
                }
                out
            };
            if let Some(a) = anchor {
                assert_eq!(multis, vec![a], "seed {seed} round {}", round.round);
            } else if round.multiplicities_after == 1 {
                anchor = Some(multis[0]);
            }
        }
        assert_eq!(run.status(), Status::Gathered);
    }
}

#[test]
fn decisions_do_not_depend_on_the_local_frame_without_ties() {
    let mut rng = Rng::new(77);
    for _ in 0..500 {
        let positions: Vec<Point> = random_positions(&mut rng)
            .into_iter()
            .map(|p| Point::new(p[0] + rng.uniform(-0.3, 0.3), p[1] + rng.uniform(-0.3, 0.3)))
            .collect();
        // Restack a couple so all three κ cases occur.
        let mut positions = positions;
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
            let local = decide_in_frame(&obs, &frame);
            assert_eq!(global, local);
        }
    }
}

#[test]
fn all_three_cases_and_every_rule_occur_in_random_runs() {
    let mut seen = std::collections::HashSet::new();
    for seed in 0..100 {
        SqGatheringRun::new(random_config(seed))
            .unwrap()
            .run_with(|r| {
                seen.insert(r.rule.clone());
            });
    }
    for rule in Rule::ALL.iter().filter(|r| **r != Rule::Alone) {
        assert!(seen.contains(rule.key()), "{} never fired", rule.key());
    }
}

// ── the main simulator's "Sequential ›" menu ────────────────────────────────

#[test]
fn hashed_grouping_matches_a_linear_scan() {
    let linear = |positions: &[Point], tol: f64| {
        let mut groups: Vec<Point> = Vec::new();
        let mut owner = Vec::new();
        for &p in positions {
            if let Some(g) = groups.iter().position(|&q| dist(p, q) <= tol) {
                owner.push(g);
            } else {
                owner.push(groups.len());
                groups.push(p);
            }
        }
        (groups, owner)
    };
    let mut rng = Rng::new(5);
    for _ in 0..200 {
        let n = 1 + rng.index(60);
        let tol = [1e-9, 0.5, 2.0][rng.index(3)];
        let positions: Vec<Point> = (0..n)
            .map(|_| Point::new(rng.uniform(-5.0, 5.0).round() * 0.5, rng.uniform(-5.0, 5.0)))
            .collect();
        let (groups, _, owner) = lcm_algorithms::sq_gathering::occupancy(&positions, tol);
        assert_eq!((groups, owner), linear(&positions, tol));
    }
}

#[test]
fn the_simulator_settings_map_to_a_sequential_run() {
    use lcm_algorithms::seq::SeqOptions;
    use lcm_core::SimConfig;
    let sim = SimConfig {
        algorithm: "SqGathering".to_owned(),
        num_of_robots: 40,
        rigid_movement: false,
        robot_speeds: 3.0,
        ..SimConfig::default()
    };
    let cfg = SeqConfig::from_sim(&sim, &SeqOptions::default()).unwrap();
    assert_eq!(cfg.positions.len(), 40);
    assert_eq!(
        cfg.movement,
        MovementSpec::NonRigid {
            delta: 3.0,
            stop: StopPolicy::Random {
                seed: 12345 ^ 0x5157_4741
            }
        }
    );
    let report = SqGatheringRun::new(cfg).unwrap().run();
    assert_eq!(report.status, Status::Gathered);

    let options = SeqOptions {
        schedule: "round-robin".into(),
        stop: "delta".into(),
        frames: "global".into(),
        multiplicities: 0,
        multiplicity_size: 0,
    };
    let cfg = SeqConfig::from_sim(&sim, &options).unwrap();
    assert_eq!(
        (cfg.schedule, cfg.frames),
        (ScheduleSpec::RoundRobin, FrameSpec::Global)
    );

    let limited = SimConfig {
        visibility_radius: Some(100.0),
        ..sim.clone()
    };
    assert!(SeqConfig::from_sim(&limited, &SeqOptions::default()).is_err());
    let faulty = SimConfig {
        num_of_faults: 2,
        ..sim.clone()
    };
    assert!(SeqConfig::from_sim(&faulty, &SeqOptions::default()).is_err());
    let bad = SeqOptions {
        stop: "teleport".into(),
        ..SeqOptions::default()
    };
    assert!(SeqConfig::from_sim(&sim, &bad).is_err());
}

#[test]
fn starting_multiplicities_make_exactly_kappa_points() {
    use lcm_algorithms::seq::SeqOptions;
    use lcm_core::SimConfig;
    for (n, k) in [(500, 1), (500, 3), (12, 2), (4, 2), (2, 1)] {
        let sim = SimConfig {
            algorithm: "SqGathering".to_owned(),
            num_of_robots: n,
            max_events: Some(1_000_000),
            ..SimConfig::default()
        };
        let options = SeqOptions {
            multiplicities: k,
            ..SeqOptions::default()
        };
        let cfg = SeqConfig::from_sim(&sim, &options).unwrap();
        let run = SqGatheringRun::new(cfg.clone()).unwrap();
        let (_, multis) = run.census();
        assert_eq!(multis, k as usize, "n = {n}, k = {k}");
        let report = SqGatheringRun::new(cfg).unwrap().run();
        assert_eq!(report.status, Status::Gathered, "n = {n}, k = {k}");
    }
    let sim = SimConfig {
        algorithm: "SqGathering".to_owned(),
        num_of_robots: 5,
        ..SimConfig::default()
    };
    let too_many = SeqOptions {
        multiplicities: 3,
        ..SeqOptions::default()
    };
    assert!(SeqConfig::from_sim(&sim, &too_many).is_err());
}

#[test]
fn large_multiplicities_under_kappa_many_stall_at_the_precision_limit() {
    // Two multiplicities of 60: robots leaving one of them halve their
    // distance each time and run below the 1e-9 coincidence tolerance.
    let mut positions = vec![[0.0, 0.0]; 60];
    positions.extend(vec![[100.0, 0.0]; 60]);
    let mut cfg = config(&positions, ScheduleSpec::RoundRobin, MovementSpec::Rigid);
    cfg.max_rounds = 1_000_000;
    let report = SqGatheringRun::new(cfg).unwrap().run();
    assert_eq!(report.status, Status::Stalled);
    assert!(
        report.note.starts_with("STALLED (floating-point limit)"),
        "{}",
        report.note
    );
    assert!(
        report.rounds < 200,
        "detected promptly, not after the budget: {}",
        report.rounds
    );
}

#[test]
fn kappa_and_multiplicity_size_are_manual() {
    use lcm_algorithms::seq::{stack_multiplicities, SeqOptions};
    use lcm_core::SimConfig;
    for (n, k, size) in [(100, 5, 3), (100, 1, 60), (40, 10, 4), (10, 5, 2)] {
        let sim = SimConfig {
            algorithm: "SqGathering".to_owned(),
            num_of_robots: n,
            max_events: Some(1_000_000),
            ..SimConfig::default()
        };
        let options = SeqOptions {
            multiplicities: k,
            multiplicity_size: size,
            ..SeqOptions::default()
        };
        let cfg = SeqConfig::from_sim(&sim, &options).unwrap();
        let pos: Vec<Point> = cfg
            .positions
            .iter()
            .map(|p| Point::new(p[0], p[1]))
            .collect();
        let (_, counts, _) = lcm_algorithms::sq_gathering::occupancy(&pos, 1e-9);
        let multis: Vec<usize> = counts.into_iter().filter(|c| *c >= 2).collect();
        assert_eq!(
            multis,
            vec![size as usize; k as usize],
            "n {n} k {k} size {size}"
        );
        let report = SqGatheringRun::new(cfg).unwrap().run();
        assert_eq!(report.status, Status::Gathered, "n {n} k {k} size {size}");
    }
    let mut p = vec![[0.0, 0.0]; 6];
    assert!(
        stack_multiplicities(&mut p, 2, 4, 1).is_err(),
        "8 robots asked of 6"
    );
    assert!(
        stack_multiplicities(&mut p, 1, 1, 1).is_err(),
        "size 1 is not a multiplicity"
    );
}
