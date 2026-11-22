//! `lcm run <config.json> [--trace <file>] [--ticks]`
//!                          — run a configuration to the end and print a summary. With `--trace`,
//!                            also write what every robot event did: JSON lines, or CSV if the file
//!                            name ends in `.csv`. `--ticks` adds the clock ticks to the trace.
//! `lcm bench [--algorithm A] [--robots N] [--seconds S] [--visibility V]`
//!                          — throughput of the core on this machine.
//! `lcm seq [config.json] [flags]` — SqGathering on the sequential scheduler,
//!                          turn by turn (see `seq.rs`).

mod seq;

use lcm_core::{
    Budget, EventKind, RobotState, SimConfig, Simulation, StepInfo, StepOutcome, StopReason,
};
use std::io::Write;
use std::time::Instant;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let result = match args.first().map(String::as_str) {
        Some("run") => run(&args[1..]),
        Some("bench") => bench(&args[1..]),
        Some("seq") => seq::run(&args[1..]).map(|code| if code != 0 { std::process::exit(code) }),
        Some("ring") => ring(&args[1..]),
        _ => Err("usage: lcm run <config.json> [--trace <file>] [--ticks] | lcm seq [config.json] [flags] | lcm bench [--algorithm A] [--robots N] [--seconds S] [--visibility V]".to_owned()),
    };
    if let Err(message) = result {
        eprintln!("{message}");
        std::process::exit(2);
    }
}

fn build(config: &SimConfig) -> Result<Simulation, String> {
    lcm_algorithms::simulation(config).map_err(|e| e.to_string())
}

fn run(args: &[String]) -> Result<(), String> {
    let path = args.first().ok_or("run needs a config file")?;
    let trace_path = match args.iter().position(|a| a == "--trace") {
        None => None,
        Some(i) => Some(args.get(i + 1).ok_or("--trace needs a file name")?),
    };
    let ticks = args.iter().any(|a| a == "--ticks");
    let text = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
    let config: SimConfig = serde_json::from_str(&text).map_err(|e| format!("{path}: {e}"))?;
    let mut sim = build(&config)?;
    let start = Instant::now();
    let mut traced = 0u64;
    let reason = if let Some(trace_path) = trace_path {
        let file = std::fs::File::create(trace_path).map_err(|e| format!("{trace_path}: {e}"))?;
        let mut out = std::io::BufWriter::new(file);
        let csv = trace_path.to_ascii_lowercase().ends_with(".csv");
        let visibility = config.visibility_radius.unwrap_or(f64::INFINITY);
        if csv {
            writeln!(out, "{CSV_HEADER}").map_err(|e| format!("{trace_path}: {e}"))?;
        }
        let reason = loop {
            let info = match sim.step_checked() {
                Ok(info) => info,
                Err(reason) => break reason,
            };
            if info.robot < 0 && !ticks {
                continue;
            }
            let record = Record::new(&sim, &info, visibility);
            let line = if csv { record.csv() } else { record.json() };
            writeln!(out, "{line}").map_err(|e| format!("{trace_path}: {e}"))?;
            traced += 1;
        };
        out.flush().map_err(|e| format!("{trace_path}: {e}"))?;
        reason
    } else {
        sim.advance(Budget {
            max_events: u64::MAX,
            until_time: f64::INFINITY,
        })
    };
    let elapsed = start.elapsed().as_secs_f64();
    let mut summary = serde_json::json!({
        "stopped": format!("{reason:?}"),
        "events": sim.event_count(),
        "time": sim.time(),
        "terminated": sim.robots().terminated.iter().filter(|t| **t).count(),
        "turns": sim.turn_count(),
        "epochs": sim.epochs_completed(),
        "robots": sim.robots().len(),
        "wall_seconds": elapsed,
    });
    if trace_path.is_some() {
        summary["traced"] = traced.into();
    }
    println!("{summary}");
    Ok(())
}

const CSV_HEADER: &str = "event,time,robot,kind,outcome,x,y,target_x,target_y,circle_x,circle_y,circle_r,visible,state,frozen,terminated,turn,epoch";

/// What one handled event did, for the trace.
struct Record {
    event: u64,
    time: f64,
    robot: i32,
    kind: &'static str,
    outcome: &'static str,
    position: Option<[f64; 2]>,
    target: Option<[f64; 2]>,
    circle: Option<[f64; 3]>,
    /// Robots in sight at a Look (not itself, not crashed); `None` for other events.
    visible: Option<Vec<usize>>,
    state: &'static str,
    frozen: bool,
    terminated: bool,
    turn: u64,
    epoch: u64,
}

impl Record {
    fn new(sim: &Simulation, info: &StepInfo, visibility: f64) -> Self {
        let kind = match info.kind {
            Some(EventKind::Look) => "look",
            Some(EventKind::Wait) => "arrive",
            Some(EventKind::Crash) => "crash",
            Some(EventKind::Visualize) => "tick",
            None => "none",
        };
        let outcome = match info.outcome {
            StepOutcome::Ignored => "ignored",
            StepOutcome::Moved => "moved",
            // After an arrival the robot just waits for its next turn; "frozen" is for a Look
            // that decided not to move.
            StepOutcome::Frozen if info.kind == Some(EventKind::Wait) => "arrived",
            StepOutcome::Frozen => "frozen",
            StepOutcome::Terminated => "terminated",
            StepOutcome::Crashed => "crashed",
            StepOutcome::Visualize => "tick",
            StepOutcome::Ended => "ended",
        };
        let mut record = Self {
            event: sim.event_count(),
            time: info.time,
            robot: info.robot,
            kind,
            outcome,
            position: None,
            target: None,
            circle: None,
            visible: None,
            state: "",
            frozen: false,
            terminated: false,
            turn: sim.turn_count(),
            epoch: sim.epochs_completed(),
        };
        let robots = sim.robots();
        let Ok(i) = usize::try_from(info.robot) else {
            return record;
        };
        if i >= robots.len() {
            return record;
        }
        record.position = Some([robots.pos[i].x, robots.pos[i].y]);
        record.target = robots.target[i].map(|p| [p.x, p.y]);
        record.circle = robots.overlay[i].map(|c| [c.center.x, c.center.y, c.radius]);
        record.state = match robots.state[i] {
            RobotState::Wait => "wait",
            RobotState::Look => "look",
            RobotState::Move => "move",
            RobotState::Crash => "crash",
        };
        record.frozen = robots.frozen[i];
        record.terminated = robots.terminated[i];
        if info.kind == Some(EventKind::Look) {
            let me = robots.pos[i];
            record.visible = Some(
                (0..robots.len())
                    .filter(|&j| {
                        j != i
                            && robots.state[j] != RobotState::Crash
                            && lcm_core::geom::dist(me, sim.position_at(j, info.time)) <= visibility
                    })
                    .collect(),
            );
        }
        record
    }

    /// One JSON object per line, its fields in reading order.
    fn json(&self) -> String {
        let j = |v: &dyn erased::ToJson| v.to_json();
        let mut fields = vec![
            ("event", j(&self.event)),
            ("time", j(&self.time)),
            ("robot", j(&self.robot)),
            ("kind", j(&self.kind)),
            ("outcome", j(&self.outcome)),
        ];
        if let Some([x, y]) = self.position {
            fields.push(("x", j(&x)));
            fields.push(("y", j(&y)));
            fields.push(("target", j(&self.target)));
            fields.push(("circle", j(&self.circle)));
            if let Some(visible) = &self.visible {
                fields.push(("visible", j(visible)));
            }
            fields.push(("state", j(&self.state)));
            fields.push(("frozen", j(&self.frozen)));
            fields.push(("terminated", j(&self.terminated)));
        }
        if self.turn > 0 || self.epoch > 0 {
            fields.push(("turn", j(&self.turn)));
            fields.push(("epoch", j(&self.epoch)));
        }
        let body: Vec<String> = fields.iter().map(|(k, v)| format!("\"{k}\":{v}")).collect();
        format!("{{{}}}", body.join(","))
    }

    fn csv(&self) -> String {
        let cell = |v: Option<f64>| v.map_or_else(String::new, |v| v.to_string());
        let [x, y] = self
            .position
            .map_or([None, None], |p| [Some(p[0]), Some(p[1])]);
        let [tx, ty] = self
            .target
            .map_or([None, None], |p| [Some(p[0]), Some(p[1])]);
        let [cx, cy, cr] = self
            .circle
            .map_or([None, None, None], |c| [Some(c[0]), Some(c[1]), Some(c[2])]);
        let visible = self.visible.as_ref().map_or_else(String::new, |v| {
            v.iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join(" ")
        });
        format!(
            "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
            self.event,
            self.time,
            self.robot,
            self.kind,
            self.outcome,
            cell(x),
            cell(y),
            cell(tx),
            cell(ty),
            cell(cx),
            cell(cy),
            cell(cr),
            visible,
            self.state,
            self.frozen,
            self.terminated,
            self.turn,
            self.epoch
        )
    }
}

fn flag<T: std::str::FromStr>(args: &[String], name: &str, default: T) -> Result<T, String> {
    match args.iter().position(|a| a == name) {
        None => Ok(default),
        Some(i) => args
            .get(i + 1)
            .and_then(|v| v.parse().ok())
            .ok_or_else(|| format!("{name} needs a value")),
    }
}

#[allow(clippy::cast_precision_loss)]
fn bench(args: &[String]) -> Result<(), String> {
    let algorithm: String = flag(args, "--algorithm", "Gathering".to_owned())?;
    let robots: u32 = flag(args, "--robots", 10_000)?;
    let seconds: f64 = flag(args, "--seconds", 5.0)?;
    let visibility: f64 = flag(args, "--visibility", 0.0)?;
    let config = SimConfig {
        algorithm,
        num_of_robots: robots,
        visibility_radius: (visibility > 0.0).then_some(visibility),
        random_seed: 1,
        ..SimConfig::default()
    };
    let mut sim = build(&config)?;
    let start = Instant::now();
    let mut reason = StopReason::Budget;
    while start.elapsed().as_secs_f64() < seconds {
        reason = sim.advance(Budget {
            max_events: 256,
            until_time: f64::INFINITY,
        });
        if reason != StopReason::Budget {
            break;
        }
    }
    let elapsed = start.elapsed().as_secs_f64();
    let events = sim.event_count();
    println!(
        "{} robots={} events={} ({:.0}/s) sim_time={:.2} wall={:.2}s stopped={:?}",
        config.algorithm,
        robots,
        events,
        events as f64 / elapsed,
        sim.time(),
        elapsed,
        reason
    );
    Ok(())
}

/// Lets `Record::json` turn values of different types into JSON text the same way.
mod erased {
    pub trait ToJson {
        fn to_json(&self) -> String;
    }
    impl<T: serde::Serialize + ?Sized> ToJson for T {
        fn to_json(&self) -> String {
            serde_json::to_string(self).expect("plain data")
        }
    }
}

/// `lcm ring <config.json> [--trace]`: GatheRRing on a ring (`lcm_ring`).
/// Prints a JSON summary; `--trace` prints every activation first.
fn ring(args: &[String]) -> Result<(), String> {
    let path = args
        .first()
        .filter(|a| !a.starts_with("--"))
        .ok_or("usage: lcm ring <config.json> [--trace]")?;
    let text = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
    let config: lcm_ring::RingConfig =
        serde_json::from_str(&text).map_err(|e| format!("{path}: {e}"))?;
    let unsolvable = lcm_ring::unsolvable(&config);
    let mut sim = lcm_ring::RingSim::new(&config)?;
    let turns = sim.run();
    if args.iter().any(|a| a == "--trace") {
        for t in &turns {
            println!(
                "turn {:>4} epoch {:>3} robot {:>2}: {:>3} -> {:<3} {:?} {}",
                t.turn, t.epoch, t.robot, t.from, t.to, t.task, t.rule
            );
        }
    }
    println!(
        "{}",
        serde_json::json!({
            "status": sim.status,
            "epochs": sim.epochs,
            "turns": sim.turns,
            "positions": sim.positions,
            "bound_epochs": config.nodes.saturating_sub(3),
            "unsolvable": unsolvable,
        })
    );
    Ok(())
}
