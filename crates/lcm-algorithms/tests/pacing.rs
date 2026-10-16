//! What the page needs to pace a sequential turn: when the robot that just started moving will
//! arrive. `arrival_time` must be the very time the scheduler queued the arrival for, and
//! reading it must change nothing about the run.

use lcm_core::{
    ActivationOrder, EventKind, SchedulerKind, SimConfig, Simulation, StepOutcome, TurnGap,
};

fn config(positions: &[[f64; 2]], rigid: bool) -> SimConfig {
    SimConfig {
        algorithm: "Gathering".to_owned(),
        num_of_robots: positions.len() as u32,
        initial_positions: Some(positions.to_vec()),
        scheduler: SchedulerKind::Sequential,
        activation_order: ActivationOrder::RoundRobin,
        turn_gap: TurnGap::None,
        rigid_movement: rigid,
        delta: if rigid { None } else { Some(5.0) },
        robot_speeds: 3.0,
        sampling_rate: 0.5,
        max_events: Some(5_000),
        ..SimConfig::default()
    }
}

fn run(config: &SimConfig, peek: bool) -> (f64, u64, Vec<[f64; 2]>, usize) {
    let mut sim: Simulation = lcm_algorithms::simulation(config).unwrap();
    let mut checked = 0;
    while let Ok(info) = sim.step_checked() {
        if peek && info.outcome == StepOutcome::Moved {
            let i = usize::try_from(info.robot).unwrap();
            let arrival = sim
                .arrival_time(i)
                .expect("a robot that just started a move has an arrival");
            // The next event of that robot is its arrival, at exactly that time.
            loop {
                let next = sim.step();
                if next.robot == info.robot && next.kind == Some(EventKind::Wait) {
                    assert_eq!(next.time.to_bits(), arrival.to_bits(), "robot {i}");
                    checked += 1;
                    break;
                }
                assert!(
                    next.kind != Some(EventKind::Look),
                    "the turn ended without an arrival"
                );
            }
        }
    }
    let t = sim.time();
    let pos = (0..sim.robots().len())
        .map(|i| {
            let p = sim.position_at(i, t);
            [p.x, p.y]
        })
        .collect();
    (t, sim.event_count(), pos, checked)
}

#[test]
fn arrival_time_is_the_time_the_arrival_is_queued_for() {
    for rigid in [true, false] {
        let start = [[-300.0, 0.0], [300.0, 0.0], [0.0, 250.0]];
        let (_, _, _, checked) = run(&config(&start, rigid), true);
        assert!(checked >= 5, "only {checked} moves checked (rigid {rigid})");
    }
}

#[test]
fn reading_it_changes_nothing() {
    for rigid in [true, false] {
        let c = config(
            &[[-300.0, 0.0], [300.0, 0.0], [0.0, 250.0], [40.0, 40.0]],
            rigid,
        );
        let (t1, e1, p1, _) = run(&c, false);
        let (t2, e2, p2, _) = run(&c, true);
        assert_eq!((t1.to_bits(), e1), (t2.to_bits(), e2), "rigid {rigid}");
        assert_eq!(p1, p2);
    }
}

#[test]
fn a_robot_that_is_not_moving_has_no_arrival() {
    let sim = lcm_algorithms::simulation(&config(&[[0.0, 0.0], [10.0, 0.0]], true)).unwrap();
    assert_eq!(sim.arrival_time(0), None);
    assert_eq!(sim.arrival_time(1), None);
}
