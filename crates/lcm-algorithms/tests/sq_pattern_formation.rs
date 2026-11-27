//! Sequential Pattern Formation (SqPF / SqPF′, arXiv:2412.10733 Sections 2–5)
//! on the core's sequential scheduler.
//!
//! Three kinds of test: the robot model of Section 2.1 (private coordinate
//! systems, no identities, no multiplicity detection), single decisions of
//! the paper's procedures, and whole runs that must end with the pattern
//! formed and no robot moving. Last, the settings the registry must refuse.

use core::f64::consts::TAU;

use lcm_algorithms::sq_gathering::LocalFrame;
use lcm_algorithms::sq_pattern_formation::{
    compasses, destination, formed_placement, is_formed, pattern_of, shape_points, snapshot, Pattern, Step, KEY,
};
use lcm_core::geom::dist;
use lcm_core::{
    ActivationOrder, Budget, PatternShape, Point, Rng, SchedulerKind, SimConfig, StopPolicy,
    StopReason, TurnGap,
};

fn p(x: f64, y: f64) -> Point {
    Point::new(x, y)
}

/// A run in the paper's model: sequential, non-rigid with δ, unlimited
/// visibility, no faults.
fn config(shape: PatternShape, robots: u32, seed: u64, delta: f64) -> SimConfig {
    SimConfig {
        algorithm: KEY.to_owned(),
        target_pattern: shape,
        num_of_robots: robots,
        random_seed: seed,
        scheduler: SchedulerKind::Sequential,
        activation_order: if seed.is_multiple_of(2) { ActivationOrder::Random } else { ActivationOrder::RoundRobin },
        turn_gap: TurnGap::None,
        rigid_movement: false,
        delta: Some(delta),
        stop_policy: StopPolicy::Random,
        max_turns: Some(400_000),
        ..SimConfig::default()
    }
}

/// The robot's decision, in world coordinates, seen through `frame`.
fn decide_world(pat: &Pattern, world: &[Point], me: usize, frame: &LocalFrame) -> (Point, Step) {
    let frame = LocalFrame { origin: world[me], ..*frame };
    let (d, step) = destination(pat, &snapshot(world, &frame, 0.0), 0.0);
    (frame.to_global(d), step)
}

fn random_cloud(n: usize, seed: u64) -> Vec<Point> {
    let mut rng = Rng::new(seed);
    (0..n).map(|_| p(rng.uniform(-100.0, 100.0), rng.uniform(-100.0, 100.0))).collect()
}

// ── the robot model (Section 2.1) ────────────────────────────────────────────

#[test]
fn a_decision_does_not_depend_on_the_private_coordinate_system() {
    // Disoriented robots: the same configuration seen with any direction of the
    // x-axis, any unit and either handedness gives the same destination. (Only
    // Separate, which by the paper moves "along its own x-axis at one unit
    // distance", is meant to depend on the frame; |Q| ≥ |P̂| here.)
    let patterns = [
        Pattern::new(&shape_points(PatternShape::Triangle)).unwrap(),
        Pattern::new(&shape_points(PatternShape::Star)).unwrap(),
        Pattern::new(&random_cloud(7, 99)).unwrap(),
    ];
    let frames = [
        LocalFrame { origin: p(0.0, 0.0), angle: 0.0, scale: 1.0, reflect: false },
        LocalFrame { origin: p(0.0, 0.0), angle: 1.1, scale: 0.3, reflect: false },
        LocalFrame { origin: p(0.0, 0.0), angle: 4.0, scale: 2.7, reflect: true },
        LocalFrame { origin: p(0.0, 0.0), angle: 2.5, scale: 1.0, reflect: true },
    ];
    let mut checked = 0;
    for (pi, pat) in patterns.iter().enumerate() {
        for seed in 0..12 {
            let world = random_cloud(pat.len() + (seed as usize % 3), 1000 + seed + 17 * pi as u64);
            for me in 0..world.len() {
                let (base, step) = decide_world(pat, &world, me, &frames[0]);
                for f in &frames[1..] {
                    let (d, s) = decide_world(pat, &world, me, f);
                    assert_eq!(s, step, "pattern {pi} seed {seed} robot {me}: step depends on the frame");
                    assert!(dist(d, base) < 1e-6, "pattern {pi} seed {seed} robot {me}: {d:?} vs {base:?}");
                    checked += 1;
                }
            }
        }
    }
    assert!(checked > 300);
}

#[test]
fn robots_on_one_point_are_one_point_and_have_no_identity() {
    // No multiplicity detection: two or five robots on a point look like one.
    let f = LocalFrame::global(p(3.0, 4.0));
    let two = [p(3.0, 4.0), p(0.0, 0.0), p(0.0, 0.0), p(9.0, 1.0)];
    let five = [p(0.0, 0.0), p(9.0, 1.0), p(0.0, 0.0), p(3.0, 4.0), p(0.0, 0.0), p(0.0, 0.0), p(0.0, 0.0)];
    assert_eq!(snapshot(&two, &f, 0.0), snapshot(&five, &f, 0.0));
    // The robot is the origin of its own snapshot.
    assert!(snapshot(&two, &f, 0.0).contains(&p(0.0, 0.0)));
}

#[test]
fn every_robot_has_its_own_compass() {
    let c = compasses(50, 7);
    assert_eq!(c.len(), 50);
    assert!(c.iter().any(|x| x.mirrored) && c.iter().any(|x| !x.mirrored), "both handednesses");
    assert!(c.windows(2).all(|w| w[0].angle != w[1].angle), "directions differ");
    assert!(c.iter().all(|x| (0.5..2.0).contains(&x.scale) && (0.0..TAU).contains(&x.angle)));
    assert_eq!(c, compasses(50, 7), "a seed fixes them");
}

// ── single decisions ─────────────────────────────────────────────────────────

#[test]
fn separate_moves_robots_off_a_multiplicity_along_their_own_x_axis() {
    // Fewer occupied points than pattern points: Procedure Separate.
    let pat = Pattern::new(&shape_points(PatternShape::Pentagon)).unwrap();
    let q = [p(0.0, 0.0), p(5.0, 5.0), p(-7.0, 2.0)];
    let (d, step) = destination(&pat, &q, 0.0);
    assert_eq!(step, Step::Separate);
    assert_eq!(d, p(1.0, 0.0));
}

#[test]
fn a_formed_pattern_never_moves() {
    // Q ∈ 𝓑(P̂): rotated, reflected, scaled and moved copies of every shape.
    // Includes the square and the triangle, which have no unique maximum pair.
    for shape in [
        PatternShape::Line, PatternShape::Triangle, PatternShape::Square, PatternShape::Pentagon,
        PatternShape::Hexagon, PatternShape::Star, PatternShape::Circle, PatternShape::Cross,
        PatternShape::Heart,
    ] {
        let pts = shape_points(shape);
        let pat = Pattern::new(&pts).unwrap();
        for (angle, s, mirror) in [(0.3, 0.7, false), (2.0, 3.0, true)] {
            let world: Vec<Point> = pts
                .iter()
                .map(|q| {
                    let y = if mirror { -q.y } else { q.y };
                    p(40.0 + s * (q.x * f64::cos(angle) - y * f64::sin(angle)),
                      -9.0 + s * (q.x * f64::sin(angle) + y * f64::cos(angle)))
                })
                .collect();
            assert!(is_formed(&pat, &world, 0.0), "{shape:?}");
            for (me, c) in compasses(world.len(), 3).iter().enumerate() {
                let (d, step) = decide_world(&pat, &world, me, &c.frame(world[me]));
                assert!(dist(d, world[me]) < 1e-9, "{shape:?}: robot {me} moves ({step:?})");
                assert!(matches!(step, Step::Formed | Step::PrimeFormed), "{shape:?}: {step:?}");
            }
        }
    }
}

#[test]
fn the_formed_shape_is_reported_in_the_patterns_own_order() {
    // What the page joins into the outline: the pattern's points where the robots stand, in
    // the order the pattern was given, i.e. a scaled, rotated or mirrored copy of P̂ point by
    // point (for a symmetric pattern the copy may start at another corner: the same outline).
    // Robots: P̂ reflected and scaled, one corner stacked (n > k).
    for pts in [shape_points(PatternShape::Pentagon), random_cloud(7, 5)] {
        let pat = Pattern::new(&pts).unwrap();
        let place = |q: &Point| p(7.0 + 2.0 * q.y, 3.0 + 2.0 * q.x); // swap x and y: a reflection
        let mut world: Vec<Point> = pts.iter().map(place).collect();
        world.push(world[2]);
        let got = formed_placement(&pat, &world, 0.0).expect("formed");
        assert_eq!(got.len(), pts.len());
        for g in &got {
            assert!(world.iter().any(|w| dist(*g, *w) < 1e-6), "every drawn point holds a robot");
        }
        let ratio = dist(got[0], got[1]) / dist(pts[0], pts[1]);
        for i in 0..pts.len() {
            for j in 0..pts.len() {
                let (a, b) = (dist(got[i], got[j]), ratio * dist(pts[i], pts[j]));
                assert!((a - b).abs() < 1e-6 * b.max(1.0), "points {i}, {j} not in the pattern's order");
            }
        }
        world[0].x += 5.0;
        assert!(formed_placement(&pat, &world, 0.0).is_none(), "not formed: nothing to draw");
    }
}

// ── whole runs ───────────────────────────────────────────────────────────────

fn run(cfg: &SimConfig) -> (StopReason, bool, u64) {
    let mut sim = lcm_algorithms::simulation(cfg).expect("a valid SqPF config");
    let stop = sim.advance(Budget { max_events: u64::MAX, until_time: f64::INFINITY });
    let pat = pattern_of(cfg).unwrap();
    (stop, is_formed(&pat, &sim.robots().pos, sim.model().eps), sim.epochs_completed())
}

#[test]
fn every_shape_is_formed_and_then_nothing_moves() {
    // The run ends because a whole epoch passed with no robot moving (robots
    // never terminate), and at that point the occupied points are the pattern.
    for shape in [
        PatternShape::Line, PatternShape::Triangle, PatternShape::Square, PatternShape::Pentagon,
        PatternShape::Hexagon, PatternShape::Star,
    ] {
        let k = u32::try_from(shape_points(shape).len()).unwrap();
        for (extra, seed, delta) in [(0, 1, 5.0), (2, 2, 20.0)] {
            let cfg = config(shape, k + extra, seed, delta);
            let (stop, formed, epochs) = run(&cfg);
            assert_eq!(stop, StopReason::Stalled, "{shape:?} n={} seed={seed}", k + extra);
            assert!(formed, "{shape:?} n={} seed={seed}: stopped after {epochs} epochs, not formed", k + extra);
        }
    }
}

#[test]
fn starting_multiplicities_are_separated_and_the_pattern_formed() {
    // Several robots per point at the start (C(0) may contain multiplicities).
    for (shape, seed) in [(PatternShape::Square, 3), (PatternShape::Hexagon, 4)] {
        let k = u32::try_from(shape_points(shape).len()).unwrap();
        let cfg = SimConfig { multiplicities: 2, multiplicity_size: 2, ..config(shape, k + 2, seed, 8.0) };
        let (stop, formed, _) = run(&cfg);
        assert_eq!(stop, StopReason::Stalled);
        assert!(formed, "{shape:?}");
    }
}

#[test]
fn a_custom_pattern_with_a_point_at_the_centre_is_formed() {
    let mut cfg = config(PatternShape::Star, 8, 5, 6.0);
    cfg.target_points = Some(vec![
        [0.0, 0.0], [10.0, 0.0], [3.0, 9.5], [-8.0, 6.0], [-8.0, -6.0], [3.0, -9.5], [5.0, 0.0],
    ]);
    let (stop, formed, _) = run(&cfg);
    assert_eq!(stop, StopReason::Stalled);
    assert!(formed);
}

#[test]
fn a_walker_closer_than_the_minimum_move_is_on_its_pattern_point() {
    // Regression: the walker ended 3.6·10⁻⁶ from its pattern point. That is above
    // 10⁻⁹·ρ, so it counted as "not there yet", but below the simulator's minimum
    // move (`eps` = 10⁻⁵), so its move was dropped: it never moved, the other
    // robots waited for it, and the run stalled unformed (turn 138, 23 epochs).
    let cfg = SimConfig {
        algorithm: KEY.to_owned(),
        target_pattern: PatternShape::Pentagon,
        num_of_robots: 6,
        random_seed: 7,
        scheduler: SchedulerKind::Sequential,
        rigid_movement: false,
        delta: Some(2.0),
        ..SimConfig::default()
    };
    let (stop, formed, epochs) = run(&cfg);
    assert_eq!(stop, StopReason::Stalled);
    assert!(formed, "stopped after {epochs} epochs, not formed");
}

// ── settings the paper's model does not have ─────────────────────────────────

#[test]
fn the_registry_refuses_settings_outside_the_papers_model() {
    let ok = config(PatternShape::Pentagon, 5, 1, 5.0);
    assert!(lcm_algorithms::simulation(&ok).is_ok());
    let refused = [
        ("async", SimConfig { scheduler: SchedulerKind::Async, ..ok.clone() }),
        ("rigid", SimConfig { rigid_movement: true, ..ok.clone() }),
        ("no delta", SimConfig { delta: None, ..ok.clone() }),
        ("limited visibility", SimConfig { visibility_radius: Some(50.0), ..ok.clone() }),
        ("faults", SimConfig { num_of_faults: 1, ..ok.clone() }),
        ("fewer robots than points", SimConfig { num_of_robots: 4, ..ok.clone() }),
        ("one point", SimConfig { target_points: Some(vec![[1.0, 1.0]]), ..ok.clone() }),
        ("repeated point", SimConfig { target_points: Some(vec![[0.0, 0.0], [1.0, 0.0], [0.0, 0.0]]), ..ok.clone() }),
    ];
    for (why, cfg) in refused {
        assert!(lcm_algorithms::simulation(&cfg).is_err(), "{why} must be refused");
    }
    let message = lcm_algorithms::simulation(&SimConfig { scheduler: SchedulerKind::Async, ..ok })
        .err()
        .unwrap()
        .to_string();
    assert!(message.contains("sequential scheduler"), "{message}");
}

#[test]
fn the_page_is_told_to_fix_the_scheduler_and_non_rigid_moves() {
    assert_eq!(lcm_algorithms::requirements(KEY), (true, true));
    assert!(lcm_algorithms::needs_non_rigid(KEY));
    assert!(!lcm_algorithms::needs_non_rigid("SqGathering"));
}
