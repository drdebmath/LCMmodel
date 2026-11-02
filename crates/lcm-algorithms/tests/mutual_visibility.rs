//! Mutual visibility (Di Luna et al. 2017, Section 6.7): from starts where
//! robots hide each other, every robot ends up seeing every other.

use lcm_core::{Budget, Point, SchedulerKind, SimConfig, StopReason};

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
