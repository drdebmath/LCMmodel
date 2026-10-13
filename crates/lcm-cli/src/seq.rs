//! `lcm seq`: run SqGathering (Algorithm 8) on the core's sequential
//! scheduler and print what every turn did.
//!
//!   lcm seq [config.json] [--positions "x,y;x,y;…"] [--robots N] [--seed S]
//!           [--schedule round-robin | random | "2,0,3@0.5,…"[+repeat]]
//!           [--rigid | --delta D [--stop delta | random | FRACTION]]
//!           [--multiplicities K [--size S]] [--max-turns N]
//!           [--trace - | FILE.jsonl] [--format table | json] [--print-config]
//!
//! `config.json` is an ordinary `SimConfig` (as for `lcm run`); flags
//! override it. The algorithm is always SqGathering and the scheduler always
//! sequential. The summary goes to stdout; `--trace -` prints every turn
//! first, `--trace FILE` writes one JSON turn per line.
//! Exit status: 0 gathered, 3 incomplete (a limit was hit), 4 stalled
//! (precision limit), 2 bad input.

use lcm_algorithms::sq_trace::{Report, Round, Status, Tracer};
use lcm_core::{
    Activation, ActivationOrder, ScheduleEnd, SchedulerKind, SimConfig, StopPolicy, TurnGap,
};
use std::io::Write as _;

pub const USAGE: &str = "usage: lcm seq [config.json] [--positions \"x,y;x,y;...\"] [--robots N] [--seed S] \
[--schedule round-robin|random|\"2,0,3@0.5,...\"[+repeat]] [--rigid | --delta D [--stop delta|random|FRACTION]] \
[--multiplicities K [--size S]] [--max-turns N] [--trace -|FILE.jsonl] [--format table|json] [--print-config]";

const VALUED: &[&str] = &[
    "--positions",
    "--robots",
    "--seed",
    "--schedule",
    "--delta",
    "--stop",
    "--multiplicities",
    "--size",
    "--max-turns",
    "--trace",
    "--format",
];

fn value<'a>(args: &'a [String], name: &str) -> Result<Option<&'a str>, String> {
    match args.iter().position(|a| a == name) {
        None => Ok(None),
        Some(i) => args
            .get(i + 1)
            .map(|v| Some(v.as_str()))
            .ok_or_else(|| format!("{name} needs a value")),
    }
}

fn number<T: std::str::FromStr>(text: &str, what: &str) -> Result<T, String> {
    text.trim()
        .parse()
        .map_err(|_| format!("{what}: cannot read {text:?}"))
}

fn parse_positions(text: &str) -> Result<Vec<[f64; 2]>, String> {
    text.split(';')
        .filter(|p| !p.trim().is_empty())
        .map(|p| {
            let xy: Vec<&str> = p.split(',').collect();
            if xy.len() != 2 {
                return Err(format!("--positions: {p:?} is not x,y"));
            }
            Ok([number(xy[0], "--positions")?, number(xy[1], "--positions")?])
        })
        .collect()
}

/// `round-robin`, `random`, or an explicit list `r,r@stop,…` with an
/// optional `+repeat`.
fn apply_schedule(config: &mut SimConfig, text: &str) -> Result<(), String> {
    match text {
        "round-robin" => {
            config.schedule = None;
            config.activation_order = ActivationOrder::RoundRobin;
        }
        "random" => {
            config.schedule = None;
            config.activation_order = ActivationOrder::Random;
        }
        _ => {
            let (list, end) = match text.strip_suffix("+repeat") {
                Some(list) => (list, ScheduleEnd::Repeat),
                None => (text, ScheduleEnd::RoundRobin),
            };
            let order = list
                .split(',')
                .filter(|a| !a.trim().is_empty())
                .map(|a| match a.split_once('@') {
                    Some((robot, stop)) => Ok(Activation::Stopped {
                        robot: number(robot, "--schedule")?,
                        stop: number(stop, "--schedule stop")?,
                    }),
                    None => Ok(Activation::Robot(number(a, "--schedule")?)),
                })
                .collect::<Result<Vec<_>, String>>()?;
            config.schedule = Some(order);
            config.schedule_end = end;
        }
    }
    Ok(())
}

/// The configuration from an optional file plus flags.
///
/// # Errors
/// An unreadable file, a bad flag, or settings the core or SqGathering refuse.
pub fn configure(args: &[String]) -> Result<SimConfig, String> {
    let file = args.first().filter(|a| !a.starts_with("--"));
    let mut config = match file {
        Some(path) => {
            let text = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
            serde_json::from_str(&text).map_err(|e| format!("{path}: {e}"))?
        }
        None => SimConfig {
            turn_gap: TurnGap::None,
            ..SimConfig::default()
        },
    };
    for (i, a) in args.iter().enumerate() {
        let flag_value = i > 0 && VALUED.contains(&args[i - 1].as_str());
        if a.starts_with("--") {
            if !(VALUED.contains(&a.as_str()) || a == "--rigid" || a == "--print-config") {
                return Err(format!("unknown option {a}\n{USAGE}"));
            }
        } else if i > 0 && !flag_value {
            return Err(format!("unexpected argument {a:?}\n{USAGE}"));
        }
    }
    "SqGathering".clone_into(&mut config.algorithm);
    config.scheduler = SchedulerKind::Sequential;
    if let Some(text) = value(args, "--robots")? {
        config.num_of_robots = number(text, "--robots")?;
    }
    if let Some(text) = value(args, "--seed")? {
        config.random_seed = number(text, "--seed")?;
    }
    if let Some(text) = value(args, "--positions")? {
        let positions = parse_positions(text)?;
        config.num_of_robots = u32::try_from(positions.len()).map_err(|e| e.to_string())?;
        config.initial_positions = Some(positions);
    }
    if let Some(text) = value(args, "--schedule")? {
        apply_schedule(&mut config, text)?;
    }
    let delta = value(args, "--delta")?
        .map(|d| number::<f64>(d, "--delta"))
        .transpose()?;
    let stop = value(args, "--stop")?;
    if args.iter().any(|a| a == "--rigid") {
        if delta.is_some() || stop.is_some() {
            return Err("--rigid cannot be combined with --delta or --stop".to_owned());
        }
        config.rigid_movement = true;
        config.delta = None;
    } else if delta.is_some() || stop.is_some() {
        config.rigid_movement = false;
        if let Some(d) = delta {
            config.delta = Some(d);
        }
        if config.delta.is_none() {
            return Err("--stop needs a minimum move: add --delta D".to_owned());
        }
        match stop {
            None => {}
            Some("delta") => config.stop_policy = StopPolicy::Delta,
            Some("random") => config.stop_policy = StopPolicy::Random,
            Some(fraction) => {
                config.stop_policy = StopPolicy::Fraction;
                config.stop_fraction = number(fraction, "--stop")?;
            }
        }
    }
    if let Some(text) = value(args, "--multiplicities")? {
        config.multiplicities = number(text, "--multiplicities")?;
    }
    if let Some(text) = value(args, "--size")? {
        config.multiplicity_size = number(text, "--size")?;
    }
    if let Some(text) = value(args, "--max-turns")? {
        config.max_turns = Some(number(text, "--max-turns")?);
    }
    config.validate().map_err(|e| e.to_string())?;
    Ok(config)
}

fn pt(p: [f64; 2]) -> String {
    format!("({}, {})", trim(p[0]), trim(p[1]))
}

fn trim(v: f64) -> String {
    let s = format!("{v:.4}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s == "-0" {
        "0".to_owned()
    } else {
        s.to_owned()
    }
}

fn observation(round: &Round) -> String {
    round
        .observation
        .iter()
        .enumerate()
        .map(|(k, o)| {
            let mark = if o.multiplicity { "M" } else { "s" };
            let me = if k == round.self_index { "*" } else { "" };
            format!("{me}{}{mark}", pt(o.at))
        })
        .collect::<Vec<_>>()
        .join(" ")
}

#[must_use]
pub fn table_row(round: &Round) -> String {
    let outcome = if round.terminated {
        "terminates".to_owned()
    } else if round.destination == round.position {
        "stays".to_owned()
    } else if round.reached {
        "reached".to_owned()
    } else {
        "stopped by the adversary".to_owned()
    };
    format!(
        "{:>5} {:>3} {:>5}  κ={} b={}  {:<24} lines {:<7} {} -> {}  end {}  [{}]  sees: {}",
        round.turn,
        round.epoch,
        round.robot,
        round.kappa,
        u8::from(round.on_multiplicity),
        round.rule,
        round.lines,
        pt(round.position),
        pt(round.destination),
        pt(round.stop),
        outcome,
        observation(round),
    )
}

#[must_use]
pub fn summary(report: &Report) -> String {
    match report.status {
        Status::Gathered => format!(
            "GATHERED at {} after {} turns ({} turns in all, {} complete epochs)",
            pt(report.gathered_at.unwrap_or_default()),
            report.gathered_after.unwrap_or(report.turns),
            report.turns,
            report.epochs_completed
        ),
        Status::Incomplete | Status::Stalled => report.note.clone(),
        Status::Running => format!("RUNNING after {} turns", report.turns),
    }
}

/// # Errors
/// Bad input, or a trace file that cannot be written.
pub fn run(args: &[String]) -> Result<i32, String> {
    let config = configure(args)?;
    if args.iter().any(|a| a == "--print-config") {
        println!(
            "{}",
            serde_json::to_string_pretty(&config).map_err(|e| e.to_string())?
        );
        return Ok(0);
    }
    let format = value(args, "--format")?.unwrap_or("table");
    if format != "table" && format != "json" {
        return Err(format!("--format: expected table or json, got {format:?}"));
    }
    let trace = value(args, "--trace")?;
    let mut sink: Option<Box<dyn std::io::Write>> = match trace {
        None => None,
        Some("-") => Some(Box::new(std::io::stdout().lock())),
        Some(path) => Some(Box::new(std::io::BufWriter::new(
            std::fs::File::create(path).map_err(|e| format!("{path}: {e}"))?,
        ))),
    };
    let to_stdout = trace == Some("-");
    let mut tracer = Tracer::new(&config).map_err(|e| e.to_string())?;
    if to_stdout && format == "table" {
        println!(
            "{:>5} {:>3} {:>5}  {:<9}  {:<24} {:<13} from -> destination  end  [outcome]  sees: (* = self, M = multiplicity, s = single)",
            "turn", "ep", "robot", "observes", "rule", "Alg. 8"
        );
    }
    let mut failure = None;
    let report = tracer.run_with(|round| {
        if let Some(out) = sink.as_mut() {
            let line = if to_stdout && format == "table" {
                table_row(round)
            } else {
                serde_json::to_string(round).unwrap_or_default()
            };
            if let Err(e) = writeln!(out, "{line}") {
                failure.get_or_insert(e.to_string());
            }
        }
    });
    if let Some(out) = sink.as_mut() {
        out.flush().map_err(|e| e.to_string())?;
    }
    drop(sink);
    if let Some(e) = failure {
        return Err(e);
    }
    if format == "json" {
        println!(
            "{}",
            serde_json::to_string(&report).map_err(|e| e.to_string())?
        );
    } else {
        println!("{}", summary(&report));
    }
    Ok(match report.status {
        Status::Gathered => 0,
        Status::Stalled => 4,
        Status::Incomplete | Status::Running => 3,
    })
}
