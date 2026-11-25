//! Mutual visibility (Di Luna et al. 2017, Section 6.7): from starts where
//! robots hide each other, every robot ends up seeing every other.

use lcm_core::geom::dist;
use lcm_core::{Activation, Budget, Point, Rng, ScheduleEnd, SchedulerKind, SimConfig, StopReason};

/// Whether some third robot stands on the segment between a and b (to
/// within `tolerance`), the core's obstruction rule for point robots.
fn blocked(points: &[Point], i: usize, j: usize, tolerance: f64) -> bool {
    let (a, b) = (points[i], points[j]);
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let len2 = dx * dx + dy * dy;
    points.iter().enumerate().any(|(k, q)| {
        if k == i || k == j {
            return false;
        }
        let s = ((q.x - a.x) * dx + (q.y - a.y) * dy) / len2;
        if s <= 0.0 || s >= 1.0 {
            return false;
        }
        let (fx, fy) = (a.x + s * dx, a.y + s * dy);
        (q.x - fx).hypot(q.y - fy) <= tolerance
    })
}

/// Runs `epochs` epochs (every robot moves once per epoch).
fn run(positions: Vec<[f64; 2]>, epochs: u64) -> (StopReason, Vec<Point>) {
    let n = positions.len() as u64;
    let config = SimConfig {
        algorithm: "MutualVisibility".to_owned(),
        num_of_robots: u32::try_from(positions.len()).unwrap(),
        initial_positions: Some(positions),
        scheduler: SchedulerKind::Sequential,
        obstructed_visibility: true,
        robot_speeds: 5.0,
        max_turns: Some(epochs * n),
        ..SimConfig::default()
    };
    let mut sim = lcm_algorithms::simulation(&config).unwrap();
    let reason = sim.advance(Budget {
        max_events: 1_000_000,
        until_time: f64::INFINITY,
    });
    (reason, sim.robots().pos.clone())
}

fn all_see_all(points: &[Point]) -> bool {
    (0..points.len()).all(|i| (i + 1..points.len()).all(|j| !blocked(points, i, j, 1e-5)))
}

#[test]
fn robots_on_a_line_end_up_seeing_each_other() {
    for n in [3, 5, 8, 12] {
        let line: Vec<[f64; 2]> = (0..n).map(|k| [f64::from(k) * 20.0, 0.0]).collect();
        // One epoch is enough (the paper's claim), and later moves keep it so.
        for epochs in [1, 2, 6] {
            let (reason, end) = run(line.clone(), epochs);
            assert_eq!(reason, StopReason::MaxTurns, "n = {n}");
            assert!(
                all_see_all(&end),
                "n = {n}, {epochs} epochs: someone is hidden"
            );
        }
    }
}

#[test]
fn a_grid_ends_up_seeing_itself() {
    let grid: Vec<[f64; 2]> = (0..4)
        .flat_map(|i| (0..4).map(move |j| [f64::from(i) * 30.0, f64::from(j) * 30.0]))
        .collect();
    let (reason, end) = run(grid, 2);
    assert_eq!(reason, StopReason::MaxTurns);
    assert!(all_see_all(&end));
    // And it stays so: the moves never hide anyone again.
    let grid: Vec<[f64; 2]> = end.iter().map(|p| [p.x, p.y]).collect();
    let (_, later) = run(grid, 10);
    assert!(all_see_all(&later));
}

#[test]
fn the_core_counts_who_sees_everyone() {
    let config = SimConfig {
        algorithm: "MutualVisibility".to_owned(),
        num_of_robots: 3,
        initial_positions: Some(vec![[0.0, 0.0], [10.0, 0.0], [20.0, 0.0]]),
        scheduler: SchedulerKind::Sequential,
        obstructed_visibility: true,
        ..SimConfig::default()
    };
    let mut sim = lcm_algorithms::simulation(&config).unwrap();
    // The ends hide each other; only the middle robot sees both.
    assert_eq!(sim.seeing_everyone(), 1);
    let (_, points) = run(vec![[0.0, 0.0], [10.0, 0.0], [20.0, 0.0]], 4);
    let all = (0..3).all(|i| (0..3).all(|j| i == j || !blocked(&points, i, j, 1e-9)));
    assert!(all, "after four epochs every robot sees the others");
    let config = SimConfig {
        initial_positions: Some(points.iter().map(|p| [p.x, p.y]).collect()),
        ..config
    };
    let mut sim = lcm_algorithms::simulation(&config).unwrap();
    assert_eq!(sim.seeing_everyone(), 3);
}

// ---- Review of #17, fix B5: fat robots block within their radius.

fn fat(positions: Vec<[f64; 2]>, radius: f64) -> SimConfig {
    SimConfig {
        algorithm: "MutualVisibility".to_owned(),
        num_of_robots: u32::try_from(positions.len()).unwrap(),
        initial_positions: Some(positions),
        scheduler: SchedulerKind::Sequential,
        obstructed_visibility: true,
        robot_radius: radius,
        robot_speeds: 50.0,
        ..SimConfig::default()
    }
}

/// Worked by hand: robot 0 at the origin, 3 below the line through robots 1
/// and 2, robot 3 far below. The widest gap between the directions points up
/// at 53.5°, towards the line; the line limits the step to half its distance
/// to the band. With a band of ε the step is 1.5 and ends 1.79 from the line,
/// inside a radius of 2, so robot 0 would hide 1 and 2 from each other. With
/// the core's width (2) it is 0.5, ending 2.6 from the line.
#[test]
fn a_fat_robot_stops_short_of_the_line_by_its_radius() {
    let start = vec![[0.0, 0.0], [-10.0, 3.0], [10.0, 3.0], [0.0, -50.0]];
    let config = SimConfig {
        schedule: Some(vec![Activation::Robot(0)]),
        max_turns: Some(1),
        ..fat(start, 2.0)
    };
    let mut sim = lcm_algorithms::simulation(&config).unwrap();
    sim.advance(Budget {
        max_events: 1_000,
        until_time: f64::INFINITY,
    });
    let p = sim.robots().pos[0];
    assert!(p.y > 0.0, "moved towards the line: {p:?}");
    assert!(3.0 - p.y > 2.0, "{p:?} is within the radius of the line");
    assert!(!blocked(&sim.robots().pos, 1, 2, 2.0));
}

/// Whether robot `k` stands within `w` of the segment between robots `i`
/// and `j`, away from both ends: the core's obstruction test.
fn in_between(points: &[Point], k: usize, i: usize, j: usize, w: f64) -> bool {
    let (a, b, q) = (points[i], points[j], points[k]);
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let len2 = dx * dx + dy * dy;
    if dist(q, a) <= w || dist(q, b) <= w {
        return false;
    }
    let s = ((q.x - a.x) * dx + (q.y - a.y) * dy) / len2;
    s > 0.0 && s < 1.0 && dist(q, Point::new(a.x + s * dx, a.y + s * dy)) <= w
}

/// A fat robot's move never puts it within its radius of the segment
/// between two robots it saw and did not already stand between: the band
/// the step is limited by. (Its move can still bring another robot within the radius
/// of its own new line of sight: see the module notes of mutual_visibility.)
#[test]
fn a_fat_robot_never_steps_between_two_others() {
    let mut rng = Rng::new(1503);
    let radius = 1.5;
    for run in 0..60 {
        let n = 4 + rng.index(5);
        let start: Vec<[f64; 2]> = (0..n)
            .map(|_| [rng.uniform(-30.0, 30.0), rng.uniform(-30.0, 30.0)])
            .collect();
        let config = SimConfig {
            schedule_end: ScheduleEnd::RoundRobin,
            max_turns: Some(10 * n as u64),
            ..fat(start.clone(), radius)
        };
        let mut sim = lcm_algorithms::simulation(&config).unwrap();
        let mut before = sim.robots().pos.clone();
        while sim.step_checked().is_ok() {
            let after = sim.robots().pos.clone();
            for m in (0..n).filter(|&m| after[m] != before[m]) {
                // The pairs it could see when it looked (the paper's rule).
                let sees = |i: usize| {
                    (0..n).all(|k| k == m || k == i || !in_between(&before, k, m, i, radius))
                };
                for i in (0..n).filter(|&i| i != m && sees(i)) {
                    for j in (i + 1..n).filter(|&j| j != m && sees(j)) {
                        assert!(
                            !in_between(&after, m, i, j, radius)
                                || in_between(&before, m, i, j, radius),
                            "run {run} from {start:?}: robot {m} stepped between {i} and {j}"
                        );
                    }
                }
            }
            before = after;
        }
    }
}
