//! What leaves the core for the screen: positions are offsets from the swarm's
//! own centre, so a tiny swarm far from (0, 0) is not rounded into one point.

use lcm_core::{Frame, SchedulerKind, SimConfig, TurnGap};

fn config(start: &[[f64; 2]]) -> SimConfig {
    SimConfig {
        algorithm: "Gathering".to_owned(),
        num_of_robots: start.len() as u32,
        initial_positions: Some(start.to_vec()),
        scheduler: SchedulerKind::Sequential,
        turn_gap: TurnGap::None,
        robot_speeds: 1000.0,
        sampling_rate: 1000.0,
        max_events: Some(50),
        ..SimConfig::default()
    }
}

#[test]
fn a_tiny_swarm_far_from_the_origin_keeps_its_shape() {
    // 1e-4 apart at 1e6: an absolute f32 (about 0.06 resolution there) cannot tell them apart.
    let start = [[1.0e6, 1.0e6], [1.0e6 + 1.0e-4, 1.0e6]];
    assert_eq!(start[0][0] as f32, start[1][0] as f32);
    let sim = lcm_algorithms::simulation(&config(&start)).unwrap();
    let mut frame = Frame::default();
    sim.write_frame(sim.time(), &mut frame);
    let gap = f64::from(frame.xy[2] - frame.xy[0]);
    assert!((gap - 1.0e-4).abs() < 1e-9, "gap {gap}");
    for i in 0..2 {
        let x = frame.origin[0] + f64::from(frame.xy[2 * i]);
        let y = frame.origin[1] + f64::from(frame.xy[2 * i + 1]);
        let p = sim.position_at(i, sim.time());
        assert!((x - p.x).abs() < 1e-9 && (y - p.y).abs() < 1e-9);
    }
}

#[test]
fn the_origin_is_the_centroid_and_targets_use_it() {
    let start = [[10.0, 20.0], [30.0, 20.0], [20.0, 50.0]];
    let mut sim = lcm_algorithms::simulation(&config(&start)).unwrap();
    let mut frame = Frame::default();
    sim.write_frame(sim.time(), &mut frame);
    assert!((frame.origin[0] - 20.0).abs() < 1e-12 && (frame.origin[1] - 30.0).abs() < 1e-12);
    // Run until some robot has a target, then check it against the core's own value.
    for _ in 0..20 {
        sim.step();
        sim.write_frame(sim.time(), &mut frame);
        if let Some(i) = (0..3).find(|&i| !frame.target[2 * i].is_nan()) {
            let t = sim.robots().target[i].unwrap();
            assert!((frame.origin[0] + f64::from(frame.target[2 * i]) - t.x).abs() < 1e-4);
            return;
        }
    }
    panic!("no robot ever had a target");
}
