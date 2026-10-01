//! Replays the Python fixtures (`reference/gen_fixtures.py`) and requires the
//! Rust core to take the same steps: same event order, robots, kinds and
//! outcomes, with times and coordinates equal to within 1e-9 (relative).

use lcm_core::{EventKind, SimConfig, Simulation, StepInfo};
use serde::Deserialize;
use std::path::PathBuf;

#[derive(Deserialize)]
struct Fixture {
    name: String,
    config: SimConfig,
    events: Vec<Vec<Option<f64>>>,
    #[serde(rename = "final")]
    end: Final,
}

#[derive(Deserialize)]
struct Final {
    event_count: u64,
    time: f64,
    ended: bool,
    robots: Vec<RobotRecord>,
}

#[derive(Deserialize)]
struct RobotRecord {
    pos: [f64; 2],
    target: Option<[f64; 2]>,
    state: u8,
    frozen: bool,
    terminated: bool,
    light: Option<String>,
    travelled: f64,
    speed: f64,
    fault: String,
    color: Option<String>,
}

fn close(a: f64, b: f64) -> bool {
    a == b || (a - b).abs() <= 1e-9 * a.abs().max(b.abs()).max(1.0)
}

fn kind_code(kind: Option<EventKind>) -> f64 {
    match kind {
        Some(EventKind::Crash) => 0.0,
        Some(EventKind::Look) => 1.0,
        Some(EventKind::Wait) => 2.0,
        Some(EventKind::Visualize) => 3.0,
        None => -1.0,
    }
}

fn fixtures() -> Vec<PathBuf> {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../fixtures");
    let mut paths: Vec<_> = std::fs::read_dir(&dir)
        .expect("fixtures directory")
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "json"))
        .collect();
    paths.sort();
    assert!(!paths.is_empty(), "no fixtures in {}", dir.display());
    paths
}

/// Compares one recorded event; returns a description of the first mismatch.
fn compare(sim: &Simulation, step: &StepInfo, rec: &[Option<f64>]) -> Result<(), String> {
    let v = |k: usize| rec.get(k).copied().flatten();
    let expected_time = v(0).unwrap_or(f64::NAN);
    if !close(step.time, expected_time) {
        return Err(format!("time {} vs python {}", step.time, expected_time));
    }
    if f64::from(step.robot) != v(1).unwrap_or(f64::NAN)
        || kind_code(step.kind) != v(2).unwrap_or(f64::NAN)
    {
        return Err(format!(
            "event (robot {}, {:?}) vs python (robot {:?}, kind {:?})",
            step.robot,
            step.kind,
            v(1),
            v(2)
        ));
    }
    if f64::from(step.outcome.code()) != v(3).unwrap_or(f64::NAN) {
        return Err(format!(
            "outcome {:?} vs python code {:?}",
            step.outcome,
            v(3)
        ));
    }
    if step.robot < 0 || rec.len() < 7 {
        return Ok(());
    }
    let i = usize::try_from(step.robot).unwrap_or_default();
    let r = sim.robots();
    let shown = if step.kind == Some(EventKind::Look) {
        r.target[i]
    } else {
        Some(r.pos[i])
    };
    match (shown, v(4), v(5)) {
        (None, None, None) => {}
        (Some(p), Some(x), Some(y)) if close(p.x, x) && close(p.y, y) => {}
        (p, x, y) => return Err(format!("robot {i} point {p:?} vs python ({x:?}, {y:?})")),
    }
    let flags =
        r.state[i] as u8 | if r.frozen[i] { 4 } else { 0 } | if r.terminated[i] { 8 } else { 0 };
    if f64::from(flags) != v(6).unwrap_or(f64::NAN) {
        return Err(format!("robot {i} flags {flags} vs python {:?}", v(6)));
    }
    Ok(())
}

#[test]
fn rust_core_matches_python_fixtures() {
    let mut failures = Vec::new();
    for path in fixtures() {
        let text = std::fs::read_to_string(&path).unwrap();
        let fx: Fixture =
            serde_json::from_str(&text).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
        let mut sim = lcm_algorithms::simulation(&fx.config).unwrap();

        let initial_ok = fx.end.robots.len() == sim.robots().len()
            && fx.end.robots.iter().enumerate().all(|(i, rec)| {
                let r = sim.robots();
                let fault = format!("{:?}", r.fault[i]).to_lowercase();
                let color = r.task[i].map(|c| format!("{c:?}").to_lowercase());
                close(rec.speed, r.speed[i]) && rec.fault == fault && rec.color == color
            });
        if !initial_ok {
            failures.push(format!(
                "{}: robot setup (speed / fault / colour) differs",
                fx.name
            ));
            continue;
        }

        let mut diverged = None;
        for (n, rec) in fx.events.iter().enumerate() {
            let step = sim.step();
            if let Err(why) = compare(&sim, &step, rec) {
                diverged = Some(format!("{}: event #{n}: {why}", fx.name));
                break;
            }
        }
        if let Some(why) = diverged {
            failures.push(why);
            continue;
        }

        // Past the recorded prefix, run to where Python stopped and compare the end state.
        while sim.event_count() < fx.end.event_count && !sim.ended() {
            sim.step();
        }
        let r = sim.robots();
        let end_ok = sim.event_count() == fx.end.event_count
            && sim.ended() == fx.end.ended
            && close(sim.time(), fx.end.time)
            && fx.end.robots.iter().enumerate().all(|(i, rec)| {
                let light = r.light[i].map(|l| format!("{l:?}").to_lowercase());
                let target_ok = match (r.target[i], rec.target) {
                    (None, None) => true,
                    (Some(p), Some(q)) => close(p.x, q[0]) && close(p.y, q[1]),
                    _ => false,
                };
                close(r.pos[i].x, rec.pos[0])
                    && close(r.pos[i].y, rec.pos[1])
                    && target_ok
                    && r.state[i] as u8 == rec.state
                    && r.frozen[i] == rec.frozen
                    && r.terminated[i] == rec.terminated
                    && light == rec.light
                    && close(r.travelled[i], rec.travelled)
            });
        if !end_ok {
            failures.push(format!(
                "{}: final state differs (events {} vs {}, ended {} vs {}, time {} vs {})",
                fx.name,
                sim.event_count(),
                fx.end.event_count,
                sim.ended(),
                fx.end.ended,
                sim.time(),
                fx.end.time
            ));
        }
    }
    assert!(
        failures.is_empty(),
        "parity failures:\n  {}",
        failures.join("\n  ")
    );
}

#[test]
fn every_registered_algorithm_builds() {
    for entry in lcm_algorithms::REGISTRY {
        let config = SimConfig {
            algorithm: entry.key.to_owned(),
            num_of_robots: 20,
            ..SimConfig::default()
        };
        let mut sim = lcm_algorithms::simulation(&config).unwrap();
        for _ in 0..500 {
            sim.step();
        }
    }
}
