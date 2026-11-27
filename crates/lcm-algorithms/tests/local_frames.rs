//! Which algorithms work for disoriented robots? For every registered
//! algorithm, one Compute is taken in the global frame and again in a random
//! private frame (rotation and unit length, then also a mirror), and the two
//! destinations are compared in global coordinates. An algorithm that only
//! uses distances and angles between robots gives the same point; one that
//! uses the axes themselves (a grid, a sort by x) does not.
//!
//! `cargo test -p lcm-algorithms --test local_frames -- --nocapture` prints the table.

use lcm_core::{
    LocalFrame, Look, Model, MultiplicityModel, Point, Rng, RobotState, SchedulerKind, Scratch,
    SimConfig, View,
};

const TRIALS: usize = 300;

fn model(eps: f64) -> Model {
    Model {
        precision: 5,
        eps,
        visibility: f64::INFINITY,
        width_bound: None,
        height_bound: None,
        multiplicity: MultiplicityModel::Weak,
    }
}

fn view_of(points: &[Point]) -> View {
    let mut v = View::default();
    for (k, p) in points.iter().enumerate() {
        v.id.push(u32::try_from(k).unwrap());
        v.pos.push(*p);
        v.state.push(RobotState::Wait);
        v.terminated.push(false);
        v.frozen.push(false);
    }
    v
}

/// Fraction of trials whose destination is the same in a random frame
/// (with or without a mirror) as in the global frame.
fn agreement(key: &str, chirality: bool) -> f64 {
    let config = SimConfig {
        algorithm: key.to_owned(),
        scheduler: SchedulerKind::Sequential,
        ..SimConfig::default()
    };
    // Two-robot algorithms refuse the default swarm: build them for two.
    // Sequential Pattern Formation refuses rigid moves (its paper's model):
    // build it with non-rigid moves of at least δ.
    let plan = lcm_algorithms::plan(key, &config)
        .or_else(|_| {
            lcm_algorithms::plan(
                key,
                &SimConfig {
                    num_of_robots: 2,
                    ..config.clone()
                },
            )
        })
        .or_else(|_| {
            lcm_algorithms::plan(
                key,
                &SimConfig {
                    rigid_movement: false,
                    delta: Some(5.0),
                    num_of_robots: 10,
                    ..config.clone()
                },
            )
        })
        .expect("builds");
    let algorithm = &plan.algorithms[0];
    let mut rng = Rng::new(42);
    let mut frames = Rng::new(7);
    let mut scratch = Scratch::default();
    let mut agree = 0;
    for _ in 0..TRIALS {
        let n = 3 + rng.index(6);
        let points: Vec<Point> = (0..n)
            .map(|_| Point::new(rng.uniform(-100.0, 100.0), rng.uniform(-100.0, 100.0)))
            .collect();
        let me = rng.index(n);
        let global_view = view_of(&points);
        let global_model = model(1e-5);
        let global = algorithm.compute(
            &Look {
                me: u32::try_from(me).unwrap(),
                me_index: me,
                position: points[me],
                view: &global_view,
                model: &global_model,
            },
            &mut Rng::new(1),
            &mut scratch,
        );
        let frame = LocalFrame::random(points[me], &mut frames, chirality);
        let local_points: Vec<Point> = points.iter().map(|p| frame.to_local(*p)).collect();
        let local_view = view_of(&local_points);
        let local_model = model(1e-5 * frame.scale);
        let local = algorithm.compute(
            &Look {
                me: u32::try_from(me).unwrap(),
                me_index: me,
                position: Point::new(0.0, 0.0),
                view: &local_view,
                model: &local_model,
            },
            &mut Rng::new(1),
            &mut scratch,
        );
        let back = frame.to_global(local.target);
        if (back.x - global.target.x).abs() < 1e-6 && (back.y - global.target.y).abs() < 1e-6 {
            agree += 1;
        }
    }
    #[allow(clippy::cast_precision_loss)]
    let share = f64::from(agree) / TRIALS as f64;
    share
}

#[test]
fn frame_invariance_of_every_algorithm() {
    println!("{:<18} {:>10} {:>10}", "algorithm", "rotated", "mirrored");
    let mut table = Vec::new();
    for entry in lcm_algorithms::REGISTRY {
        let rotated = agreement(entry.key, true);
        let mirrored = agreement(entry.key, false);
        println!(
            "{:<18} {:>9.0}% {:>9.0}%",
            entry.key,
            rotated * 100.0,
            mirrored * 100.0
        );
        table.push((entry.key, rotated, mirrored));
    }
    // Every registered algorithm uses only distances and angles between
    // robots, so it gives the same point in any frame. An algorithm that
    // reads the axes on purpose must be listed here, with the reason.
    // - PatternFormation: the port of robot.py, which gives every robot the
    //   world's axes and breaks ties with robot IDs (docs/sqpf.md).
    // - SqPatternFormation: with fewer robot positions than pattern points it
    //   runs Procedure Separate, which moves "along its own x-axis" (arXiv:2412.10733,
    //   Procedure 2). Every other step is checked frame-invariant in
    //   tests/sq_pattern_formation.rs.
    const FRAME_DEPENDENT: &[&str] = &["PatternFormation", "SqPatternFormation"];
    for (key, rotated, mirrored) in table {
        if FRAME_DEPENDENT.contains(&key) {
            assert!(rotated < 1.0, "{key} is listed as frame-dependent");
        } else {
            assert!(
                rotated == 1.0 && mirrored == 1.0,
                "{key} should not depend on the frame"
            );
        }
    }
}
