//! The sequential scheduler: one robot at a time does a whole
//! Look-Compute-Move, and every epoch gives each live robot exactly one turn.

use lcm_core::{
    ActivationOrder, EventKind, FaultSelection, RobotState, SchedulerKind, SimConfig, Simulation,
    StepInfo, StepOutcome, TurnGap,
};
use std::collections::BTreeSet;

fn config(order: ActivationOrder, gap: TurnGap) -> SimConfig {
    SimConfig {
        num_of_robots: 12,
        scheduler: SchedulerKind::Sequential,
        activation_order: order,
        turn_gap: gap,
        // Fast robots and sparse Visualize ticks, so the event budget goes on turns.
        robot_speeds: 500.0,
        sampling_rate: 10.0,
        max_events: Some(20_000),
        ..SimConfig::default()
    }
}

fn all_configs() -> Vec<SimConfig> {
    let mut out = Vec::new();
    for order in [ActivationOrder::RoundRobin, ActivationOrder::Random] {
        for gap in [TurnGap::Random, TurnGap::None] {
            out.push(config(order, gap));
        }
    }
    out
}

/// One handled robot event, with what the scheduler looked like just before it.
struct Turn {
    step: StepInfo,
    epoch: u64,
    live: BTreeSet<i32>,
    previous_time: f64,
}

fn live(sim: &Simulation) -> BTreeSet<i32> {
    let r = sim.robots();
    (0..r.len())
        .filter(|&i| r.state[i] != RobotState::Crash && !r.terminated[i])
        .map(|i| i32::try_from(i).unwrap())
        .collect()
}

fn run(config: &SimConfig) -> (Vec<Turn>, Simulation) {
    let mut sim = lcm_algorithms::simulation(config).unwrap();
    let mut turns = Vec::new();
    let mut previous_time = 0.0;
    let mut first = true;
    while !sim.ended() && sim.event_count() < 20_000 {
        let epoch = sim.epochs_completed();
        let live = live(&sim);
        let step = sim.step();
        let moving = (0..sim.robots().len())
            .filter(|&i| sim.robots().state[i] == RobotState::Move)
            .count();
        assert!(moving <= 1, "{moving} robots moving at t={}", step.time);
        if step.robot >= 0 && step.outcome != StepOutcome::Ignored {
            turns.push(Turn {
                step,
                epoch,
                live,
                previous_time: if first { f64::NAN } else { previous_time },
            });
            previous_time = step.time;
            first = false;
        }
    }
    (turns, sim)
}

fn looks(turns: &[Turn]) -> impl Iterator<Item = &Turn> {
    turns
        .iter()
        .filter(|t| t.step.kind == Some(EventKind::Look))
}

#[test]
fn a_turn_is_a_whole_cycle_and_nobody_else_acts_during_it() {
    for config in all_configs() {
        let (turns, sim) = run(&config);
        assert!(sim.epochs_completed() > 2, "{config:?}");
        for pair in turns.windows(2) {
            let (a, b) = (&pair[0].step, &pair[1].step);
            if a.kind == Some(EventKind::Look) && a.outcome == StepOutcome::Moved {
                assert_eq!(b.robot, a.robot, "another robot acted mid-move");
                assert_eq!(b.kind, Some(EventKind::Wait));
            }
        }
    }
}

#[test]
fn every_live_robot_gets_exactly_one_turn_per_epoch() {
    for config in all_configs() {
        let (turns, sim) = run(&config);
        let last = sim.epochs_completed();
        for epoch in 0..last {
            let in_epoch: Vec<&Turn> = looks(&turns).filter(|t| t.epoch == epoch).collect();
            let robots: Vec<i32> = in_epoch.iter().map(|t| t.step.robot).collect();
            let distinct: BTreeSet<i32> = robots.iter().copied().collect();
            assert_eq!(
                distinct.len(),
                robots.len(),
                "a robot went twice in epoch {epoch}"
            );
            assert_eq!(
                distinct, in_epoch[0].live,
                "epoch {epoch} skipped a live robot"
            );
            if config.activation_order == ActivationOrder::RoundRobin {
                assert!(robots.windows(2).all(|w| w[0] < w[1]), "{robots:?}");
            }
        }
    }
}

#[test]
fn random_order_differs_between_epochs_and_repeats_with_the_seed() {
    let config = config(ActivationOrder::Random, TurnGap::Random);
    let order = |config: &SimConfig| -> Vec<(u64, i32)> {
        looks(&run(config).0)
            .map(|t| (t.epoch, t.step.robot))
            .collect()
    };
    let a = order(&config);
    assert_eq!(a, order(&config));
    let epoch = |k| {
        a.iter()
            .filter(|(e, _)| *e == k)
            .map(|(_, r)| *r)
            .collect::<Vec<_>>()
    };
    assert_ne!(epoch(0), epoch(1));
    let reseeded = SimConfig {
        random_seed: config.random_seed + 1,
        ..config
    };
    assert_ne!(a, order(&reseeded));
}

#[test]
fn crashed_robots_never_take_a_turn() {
    let config = SimConfig {
        num_of_faults: 3,
        fault_type: FaultSelection::Crash,
        ..config(ActivationOrder::RoundRobin, TurnGap::Random)
    };
    let (turns, sim) = run(&config);
    let crashed: BTreeSet<i32> = (0..sim.robots().len())
        .filter(|&i| sim.robots().state[i] == RobotState::Crash)
        .map(|i| i32::try_from(i).unwrap())
        .collect();
    assert_eq!(crashed.len(), 3);
    assert!(turns.iter().all(|t| !crashed.contains(&t.step.robot)));
}

#[test]
fn the_turn_gap_is_random_or_none() {
    for order in [ActivationOrder::RoundRobin, ActivationOrder::Random] {
        let (turns, _) = run(&config(order, TurnGap::None));
        for t in looks(&turns).skip(1) {
            assert_eq!(
                t.step.time, t.previous_time,
                "no gap: look at the instant the last turn ended"
            );
        }
        let (turns, _) = run(&config(order, TurnGap::Random));
        for t in looks(&turns).skip(1) {
            assert!(t.step.time > t.previous_time);
        }
    }
}

#[test]
fn async_stays_the_default() {
    assert_eq!(SimConfig::default().scheduler, SchedulerKind::Async);
    let sim = lcm_algorithms::simulation(&SimConfig::default()).unwrap();
    assert_eq!(sim.epochs_completed(), 0);
}

#[test]
fn ticks_are_counted_apart_from_robot_events() {
    let mut sim =
        lcm_algorithms::simulation(&config(ActivationOrder::RoundRobin, TurnGap::Random)).unwrap();
    let mut robot_events = 0;
    while !sim.ended() {
        if sim.step().robot >= 0 {
            robot_events += 1;
        }
    }
    assert!(sim.tick_count() > 0);
    assert_eq!(sim.event_count() - sim.tick_count(), robot_events);
}
