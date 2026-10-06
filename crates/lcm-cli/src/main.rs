//! `lcm run <config.json>`  — run a configuration to the end and print a summary.
//! `lcm bench [--algorithm A] [--robots N] [--seconds S] [--visibility V]`
//!                          — throughput of the core on this machine.
//! `lcm seq [config.json] [flags]` — a sequential-scheduler algorithm
//!                          (SqGathering) with its round-by-round trace; see `seq.rs`.

mod seq;

use lcm_core::{Budget, SimConfig, Simulation, StopReason};
use std::time::Instant;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let result = match args.first().map(String::as_str) {
        Some("run") => run(&args[1..]),
        Some("bench") => bench(&args[1..]),
        Some("seq") => match seq::run(&args[1..]) {
            Ok(0) => Ok(()),
            Ok(code) => std::process::exit(code),
            Err(e) => Err(e),
        },
        _ => Err(format!("usage: lcm run <config.json> | lcm bench [--algorithm A] [--robots N] [--seconds S] [--visibility V]
       {}", seq::USAGE)),
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
    let text = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
    let config: SimConfig = serde_json::from_str(&text).map_err(|e| format!("{path}: {e}"))?;
    let mut sim = build(&config)?;
    let start = Instant::now();
    let reason = sim.advance(Budget {
        max_events: u64::MAX,
        until_time: f64::INFINITY,
    });
    let elapsed = start.elapsed().as_secs_f64();
    let summary = serde_json::json!({
        "stopped": format!("{reason:?}"),
        "events": sim.event_count(),
        "time": sim.time(),
        "terminated": sim.robots().terminated.iter().filter(|t| **t).count(),
        "robots": sim.robots().len(),
        "wall_seconds": elapsed,
    });
    println!("{summary}");
    Ok(())
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
