//! Go-To-Center (Ando, Oasa, Suzuki, Yamashita 1999) in the Rust core.
//!
//! The recorded Python runs (`fixtures/gtc_*.json`, replayed by `parity.rs`) prove the port
//! matches the original event by event. These tests check the algorithm against its definition,
//! independently of the Python: hand-worked destinations, a brute-force reimplementation, and
//! the guarantee the paper is about, that robots which start within sight of each other stay
//! within sight, under the sequential scheduler.

use lcm_algorithms::GoToCenter;
use lcm_core::pyfloat::pow10_neg;
use lcm_core::{
    Algorithm, Decision, Look, Model, OverlayUpdate, Point, Rng, RobotState, SchedulerKind,
    Scratch, SimConfig, StopPolicy, StopReason, TurnGap, View,
};

fn model(visibility: f64, precision: u8) -> Model {
    Model {
        precision,
        eps: pow10_neg(precision),
        visibility,
        width_bound: None,
        height_bound: None,
        multiplicity: lcm_core::MultiplicityModel::Weak,
    }
}

fn view_of(points: &[(f64, f64)], crashed: &[usize]) -> View {
    let n = points.len();
    View {
        id: (0..n as u32).collect(),
        pos: points.iter().map(|&(x, y)| Point::new(x, y)).collect(),
        state: (0..n)
            .map(|i| {
                if crashed.contains(&i) {
                    RobotState::Crash
                } else {
                    RobotState::Wait
                }
            })
            .collect(),
        terminated: vec![false; n],
        frozen: vec![false; n],
    }
}

/// What robot `0` computes when it sees `points` (itself first) and the listed robots are crashed.
fn go(points: &[(f64, f64)], crashed: &[usize], visibility: f64, precision: u8) -> Decision {
    let view = view_of(points, crashed);
    let model = model(visibility, precision);
    let look = Look {
        me: 0,
        me_index: 0,
        position: view.pos[0],
        view: &view,
        model: &model,
    };
    GoToCenter.compute(&look, &mut Rng::new(1), &mut Scratch::default())
}

fn close(a: Point, x: f64, y: f64, tolerance: f64) -> bool {
    (a.x - x).abs() <= tolerance && (a.y - y).abs() <= tolerance
}

fn circle_of(d: &Decision) -> (Point, f64) {
    match d.overlay {
        OverlayUpdate::Set(c) => (c.center, c.radius),
        other => panic!("expected a circle, got {other:?}"),
    }
}

// ---- hand-worked destinations ----

#[test]
fn a_robot_that_sees_nobody_stays_and_is_done() {
    let d = go(&[(3.0, 4.0)], &[], 100.0, 5);
    assert_eq!(d.target, Point::new(3.0, 4.0));
    assert!(d.terminate);
    assert_eq!(
        d.overlay,
        OverlayUpdate::Keep,
        "the displayed circle is left alone"
    );
    // Everyone else crashed counts as seeing nobody.
    let d = go(&[(3.0, 4.0), (10.0, 0.0)], &[1], 100.0, 5);
    assert_eq!(d.target, Point::new(3.0, 4.0));
    assert!(d.terminate);
}

#[test]
fn the_end_of_a_chain_goes_to_the_midpoint_with_its_only_neighbour() {
    // A(-80,10) sees B(0,0) (80.6 away) but not C (175 away): the smallest circle has the
    // diameter AB, centred (-40,5). The limit from B is 90.3, more than the 40.3 to go.
    let d = go(&[(-80.0, 10.0), (0.0, 0.0)], &[], 100.0, 10);
    assert!(close(d.target, -40.0, 5.0, 1e-9), "{:?}", d.target);
    let (center, radius) = circle_of(&d);
    assert!(close(center, -40.0, 5.0, 1e-9));
    assert!((radius - 40.311_288_741_492_75).abs() < 1e-9);
    assert!(!d.terminate);
}

#[test]
fn the_middle_of_a_chain_goes_to_the_centre_of_the_far_pair() {
    // B(0,0) sees A(-80,10) and C(95,-5). The smallest circle has the diameter AC, centred
    // (7.5, 2.5); B is inside it. No neighbour limits the 7.9 step.
    let d = go(&[(0.0, 0.0), (-80.0, 10.0), (95.0, -5.0)], &[], 100.0, 10);
    assert!(close(d.target, 7.5, 2.5, 1e-9), "{:?}", d.target);
    let (center, radius) = circle_of(&d);
    assert!(close(center, 7.5, 2.5, 1e-9));
    assert!(
        (radius - 175.0_f64.hypot(15.0) / 2.0).abs() < 1e-9,
        "radius {radius}"
    );
}

#[test]
fn the_visibility_limit_can_cut_a_step_short() {
    // Worked out from the definition with a brute-force circle and a bisection (not from this
    // code): the centre is (40.7971, 37.0290), 55.0958 away, but the disc around the midpoint
    // with the neighbour at (-5,-10) only lets the robot go 44.7475 of the way.
    let d = go(
        &[(0.0, 0.0), (-5.0, -10.0), (10.0, 95.0), (95.0, 0.0)],
        &[],
        100.0,
        10,
    );
    let (center, _) = circle_of(&d);
    assert!(close(center, 40.7971, 37.0290, 1e-4), "{center:?}");
    assert!(
        close(d.target, 33.134_414_9, 30.074_042_7, 1e-6),
        "{:?}",
        d.target
    );
    let step = d.target.x.hypot(d.target.y);
    assert!((step - 44.7475).abs() < 1e-3, "step {step}");
}

#[test]
fn two_robots_exactly_at_the_visibility_distance_still_meet_in_the_middle() {
    // d = V = 100: the neighbour is visible. Straight at each other, the limit is
    // r0 + V/2 = 100, so the whole 50 to the midpoint is allowed.
    let d = go(&[(0.0, 0.0), (100.0, 0.0)], &[], 100.0, 10);
    assert!(close(d.target, 50.0, 0.0, 1e-9), "{:?}", d.target);
}

#[test]
fn unlimited_visibility_goes_straight_to_the_centre() {
    let d = go(
        &[(0.0, 0.0), (-300.0, 0.0), (300.0, 40.0), (10.0, 500.0)],
        &[],
        f64::INFINITY,
        10,
    );
    let (center, _) = circle_of(&d);
    assert!(
        close(d.target, center.x, center.y, 1e-9),
        "{:?} vs {center:?}",
        d.target
    );
}

#[test]
fn a_robot_already_at_the_centre_stays_and_is_done_only_when_the_circle_is_a_point() {
    let d = go(&[(0.0, 0.0), (-10.0, 0.0), (10.0, 0.0)], &[], 100.0, 5);
    assert!(close(d.target, 0.0, 0.0, 1e-9));
    assert!(
        !d.terminate,
        "the visible robots are still spread over a circle of radius 10"
    );
    let d = go(&[(0.0, 0.0), (1e-7, 0.0), (-1e-7, 0.0)], &[], 100.0, 5);
    assert!(
        d.terminate,
        "the visible robots have collapsed to within epsilon of a point"
    );
}

#[test]
fn coincident_neighbours_impose_no_limit_and_crashed_robots_are_ignored() {
    // A neighbour on the very same spot is skipped by the limit (distance <= 1e-12).
    let d = go(&[(0.0, 0.0), (0.0, 0.0), (40.0, 0.0)], &[], 100.0, 10);
    assert!(close(d.target, 20.0, 0.0, 1e-9), "{:?}", d.target);
    // A crashed robot is neither in the circle nor a limit.
    let with_crashed = go(&[(0.0, 0.0), (40.0, 0.0), (-90.0, 0.0)], &[2], 100.0, 10);
    let without = go(&[(0.0, 0.0), (40.0, 0.0)], &[], 100.0, 10);
    assert!(close(
        with_crashed.target,
        without.target.x,
        without.target.y,
        1e-12
    ));
}

#[test]
fn the_circle_is_a_randomised_routine_but_its_answer_does_not_depend_on_the_seed() {
    let points = [
        (0.0, 0.0),
        (-5.0, -10.0),
        (10.0, 95.0),
        (95.0, 0.0),
        (40.0, 40.0),
        (-20.0, 60.0),
    ];
    let first = go(&points, &[], 100.0, 10).target;
    for seed in 2..30 {
        let view = view_of(&points, &[]);
        let m = model(100.0, 10);
        let look = Look {
            me: 0,
            me_index: 0,
            position: view.pos[0],
            view: &view,
            model: &m,
        };
        let t = GoToCenter
            .compute(&look, &mut Rng::new(seed), &mut Scratch::default())
            .target;
        assert!(
            close(t, first.x, first.y, 1e-7),
            "seed {seed}: {t:?} vs {first:?}"
        );
    }
}

// ---- a brute-force reimplementation of the definition ----

fn brute_sec(points: &[Point]) -> (Point, f64) {
    let inside = |c: Point, r: f64| {
        points
            .iter()
            .all(|p| lcm_core::geom::dist(c, *p) <= r + 1e-9)
    };
    let mut best: Option<(Point, f64)> = None;
    let consider = |c: Point, r: f64, best: &mut Option<(Point, f64)>| {
        if inside(c, r) && best.is_none_or(|(_, b)| r < b) {
            *best = Some((c, r));
        }
    };
    for (i, a) in points.iter().enumerate() {
        consider(*a, 0.0, &mut best);
        for (j, b) in points.iter().enumerate().skip(i + 1) {
            consider(
                Point::new((a.x + b.x) / 2.0, (a.y + b.y) / 2.0),
                lcm_core::geom::dist(*a, *b) / 2.0,
                &mut best,
            );
            for c in points.iter().skip(j + 1) {
                let d = 2.0 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
                if d.abs() < 1e-12 {
                    continue;
                }
                let (a2, b2, c2) = (
                    a.x * a.x + a.y * a.y,
                    b.x * b.x + b.y * b.y,
                    c.x * c.x + c.y * c.y,
                );
                let ux = (a2 * (b.y - c.y) + b2 * (c.y - a.y) + c2 * (a.y - b.y)) / d;
                let uy = (a2 * (c.x - b.x) + b2 * (a.x - c.x) + c2 * (b.x - a.x)) / d;
                let center = Point::new(ux, uy);
                consider(center, lcm_core::geom::dist(center, *a), &mut best);
            }
        }
    }
    best.expect("a smallest circle exists")
}

/// The destination from the definition: the farthest point on the way to the centre of the
/// smallest circle that is inside the disc (radius V/2) around the midpoint with every neighbour.
fn brute_destination(me: Point, others: &[Point], visibility: f64) -> Point {
    let mut all = vec![me];
    all.extend_from_slice(others);
    let (center, _) = brute_sec(&all);
    let goal = lcm_core::geom::dist(me, center);
    if goal < 1e-9 {
        return me;
    }
    let (ux, uy) = ((center.x - me.x) / goal, (center.y - me.y) / goal);
    let ok = |t: f64| {
        let p = Point::new(me.x + ux * t, me.y + uy * t);
        others.iter().all(|q| {
            lcm_core::geom::dist(me, *q) <= 1e-12
                || lcm_core::geom::dist(p, Point::new((me.x + q.x) / 2.0, (me.y + q.y) / 2.0))
                    <= visibility / 2.0 + 1e-12
        })
    };
    let t = if visibility.is_infinite() || ok(goal) {
        goal
    } else {
        let (mut lo, mut hi) = (0.0, goal);
        for _ in 0..200 {
            let mid = (lo + hi) / 2.0;
            if ok(mid) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo
    };
    Point::new(me.x + ux * t, me.y + uy * t)
}

#[test]
fn the_port_agrees_with_a_brute_force_version_of_the_definition() {
    let mut rng = Rng::new(2024);
    let mut limited = 0;
    for case in 0..400 {
        let visibility = if case % 5 == 0 {
            f64::INFINITY
        } else {
            rng.uniform(40.0, 160.0)
        };
        let n = 2 + rng.index(8);
        let me = Point::new(rng.uniform(-50.0, 50.0), rng.uniform(-50.0, 50.0));
        let mut points = vec![(me.x, me.y)];
        while points.len() < n {
            let p = Point::new(rng.uniform(-110.0, 110.0), rng.uniform(-110.0, 110.0));
            if lcm_core::geom::dist(me, p) <= visibility {
                points.push((p.x, p.y));
            }
        }
        let others: Vec<Point> = points[1..].iter().map(|&(x, y)| Point::new(x, y)).collect();
        let expected = brute_destination(me, &others, visibility);
        let got = go(&points, &[], visibility, 11).target;
        let gap = lcm_core::geom::dist(expected, got);
        assert!(
            gap < 1e-6,
            "case {case}: {got:?} vs {expected:?} (gap {gap})"
        );
        let (center, _) = brute_sec(&[vec![me], others].concat());
        if lcm_core::geom::dist(me, got) < lcm_core::geom::dist(me, center) - 1e-6 {
            limited += 1;
        }
    }
    assert!(
        limited > 20,
        "only {limited} of the cases exercised the visibility limit"
    );
}

// ---- the guarantee: robots that start within sight stay within sight ----

fn connected(points: &[[f64; 2]], v: f64) -> bool {
    let n = points.len();
    let mut seen = vec![false; n];
    let mut stack = vec![0];
    seen[0] = true;
    while let Some(i) = stack.pop() {
        for j in 0..n {
            if !seen[j] && (points[i][0] - points[j][0]).hypot(points[i][1] - points[j][1]) <= v {
                seen[j] = true;
                stack.push(j);
            }
        }
    }
    seen.iter().all(|s| *s)
}

fn random_connected_start(seed: u64, n: usize, side: f64, v: f64) -> Option<Vec<[f64; 2]>> {
    let mut rng = Rng::new(seed);
    let points: Vec<[f64; 2]> = (0..n)
        .map(|_| [rng.uniform(0.0, side), rng.uniform(0.0, side)])
        .collect();
    connected(&points, v).then_some(points)
}

fn run_and_watch_links(
    config: &SimConfig,
    start: &[[f64; 2]],
    v: f64,
) -> (StopReason, f64, Vec<Point>) {
    let n = start.len();
    let pairs: Vec<(usize, usize)> = (0..n)
        .flat_map(|i| (i + 1..n).map(move |j| (i, j)))
        .filter(|&(i, j)| (start[i][0] - start[j][0]).hypot(start[i][1] - start[j][1]) <= v)
        .collect();
    let mut sim = lcm_algorithms::simulation(config).unwrap();
    let mut farthest: f64 = 0.0;
    let reason = loop {
        if sim.ended() {
            break StopReason::Ended;
        }
        if sim.stalled() {
            break StopReason::Stalled;
        }
        if sim.turn_limit_reached() {
            break StopReason::MaxTurns;
        }
        sim.step();
        let t = sim.time();
        for &(i, j) in &pairs {
            let d = lcm_core::geom::dist(sim.position_at(i, t), sim.position_at(j, t));
            farthest = farthest.max(d - v);
        }
    };
    let end: Vec<Point> = (0..n).map(|i| sim.position_at(i, sim.time())).collect();
    (reason, farthest, end)
}

fn config(start: &[[f64; 2]], v: f64) -> SimConfig {
    SimConfig {
        algorithm: "GoToCenter".to_owned(),
        num_of_robots: start.len() as u32,
        initial_positions: Some(start.to_vec()),
        visibility_radius: Some(v),
        scheduler: SchedulerKind::Sequential,
        turn_gap: TurnGap::None,
        robot_speeds: 1000.0,
        sampling_rate: 1000.0,
        max_turns: Some(20_000),
        ..SimConfig::default()
    }
}

#[test]
fn under_the_sequential_scheduler_robots_that_start_within_sight_stay_within_sight() {
    let (mut runs, mut worst): (usize, f64) = (0, f64::MIN);
    for seed in 1..=120 {
        let Some(start) = random_connected_start(seed, 8, 160.0, 100.0) else {
            continue;
        };
        let (reason, farthest, end) = run_and_watch_links(&config(&start, 100.0), &start, 100.0);
        // Near the end the swarm can sit just above the termination threshold with nobody able
        // to move; the core reports that as stalled. The swarm is still gathered (checked below).
        assert!(
            matches!(reason, StopReason::Ended | StopReason::Stalled),
            "seed {seed}: {reason:?}"
        );
        assert!(
            farthest <= 1e-9,
            "seed {seed}: two robots drifted {farthest} beyond sight"
        );
        // Connected and never losing a link, they all end up together.
        let spread = end
            .iter()
            .flat_map(|a| end.iter().map(|b| lcm_core::geom::dist(*a, *b)))
            .fold(0.0, f64::max);
        assert!(spread < 1e-3, "seed {seed}: ended {spread} apart");
        runs += 1;
        worst = worst.max(farthest);
    }
    assert!(runs >= 40, "only {runs} connected starts");
    println!("{runs} runs, worst excess over V: {worst:e}");
}

#[test]
fn the_guarantee_also_holds_when_robots_are_stopped_short_on_the_way() {
    // A move stopped part-way is still on the segment inside every disc, so no link is lost.
    let mut runs = 0;
    for seed in 1..=60 {
        let Some(start) = random_connected_start(seed + 500, 8, 160.0, 100.0) else {
            continue;
        };
        let config = SimConfig {
            rigid_movement: false,
            delta: Some(3.0),
            stop_policy: StopPolicy::Random,
            random_seed: seed,
            ..config(&start, 100.0)
        };
        let (reason, farthest, _) = run_and_watch_links(&config, &start, 100.0);
        assert!(
            matches!(
                reason,
                StopReason::Ended | StopReason::Stalled | StopReason::MaxTurns
            ),
            "seed {seed}: {reason:?}"
        );
        assert!(
            farthest <= 1e-9,
            "seed {seed}: two robots drifted {farthest} beyond sight"
        );
        runs += 1;
    }
    assert!(runs >= 20);
}

#[test]
fn with_unlimited_visibility_everyone_gathers_on_one_point() {
    let mut rng = Rng::new(5);
    let start: Vec<[f64; 2]> = (0..10)
        .map(|_| [rng.uniform(0.0, 600.0), rng.uniform(0.0, 600.0)])
        .collect();
    let config = SimConfig {
        visibility_radius: None,
        ..config(&start, 100.0)
    };
    let mut sim = lcm_algorithms::simulation(&config).unwrap();
    while !sim.ended() && sim.event_count() < 2_000_000 && !sim.turn_limit_reached() {
        sim.step();
    }
    assert!(sim.ended());
    let end: Vec<Point> = (0..10).map(|i| sim.position_at(i, sim.time())).collect();
    let spread = end
        .iter()
        .flat_map(|a| end.iter().map(|b| lcm_core::geom::dist(*a, *b)))
        .fold(0.0, f64::max);
    assert!(spread < 1e-3, "ended {spread} apart");
}

#[test]
fn a_swarm_that_starts_in_two_groups_out_of_sight_gathers_each_group_separately() {
    let mut start = Vec::new();
    let mut rng = Rng::new(9);
    for _ in 0..5 {
        start.push([rng.uniform(0.0, 40.0), rng.uniform(0.0, 40.0)]);
    }
    for _ in 0..5 {
        start.push([rng.uniform(400.0, 440.0), rng.uniform(400.0, 440.0)]);
    }
    let (reason, _, end) = run_and_watch_links(&config(&start, 100.0), &start, 100.0);
    assert_eq!(reason, StopReason::Ended);
    let near: Vec<&Point> = end.iter().filter(|p| p.x < 200.0).collect();
    let far: Vec<&Point> = end.iter().filter(|p| p.x >= 200.0).collect();
    assert_eq!((near.len(), far.len()), (5, 5));
    for group in [near, far] {
        let spread = group
            .iter()
            .flat_map(|a| group.iter().map(|b| lcm_core::geom::dist(**a, **b)))
            .fold(0.0, f64::max);
        assert!(spread < 1e-3, "a group ended {spread} apart");
    }
}
