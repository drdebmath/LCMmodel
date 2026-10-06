//! `lcm seq`: run a sequential-scheduler algorithm (Algorithm 8, SqGathering)
//! and print its trace.
//!
//!   lcm seq [config.json] [--algorithm SqGathering] [--positions "x,y;x,y;…"]
//!           [--schedule round-robin | shuffled:SEED | "2,0,3@0.5,…"[+repeat]]
//!           [--rigid | --delta D] [--stop delta | random:SEED | FRACTION]
//!           [--frames global | random:SEED] [--max-rounds N]
//!           [--trace - | FILE.jsonl] [--format table | json] [--print-config]
//!
//! Flags override the config file. The summary goes to stdout; `--trace -`
//! prints every round first, `--trace FILE` writes one JSON round per line.
//! Exit status: 0 gathered, 3 incomplete (round budget hit), 2 bad input.

use lcm_algorithms::seq::{
    Activation, Fallback, FrameSpec, MovementSpec, Report, Round, ScheduleSpec, SeqConfig,
    SqGatheringRun, Status, StopPolicy,
};
use std::io::Write as _;

pub const USAGE: &str =
    "usage: lcm seq [config.json] [--algorithm SqGathering] [--positions \"x,y;x,y;...\"] \
[--schedule round-robin|shuffled:SEED|\"2,0,3@0.5,...\"[+repeat]] [--rigid | --delta D] \
[--stop delta|random:SEED|FRACTION] [--frames global|random:SEED] [--max-rounds N] \
[--trace -|FILE.jsonl] [--format table|json] [--print-config]";

const VALUED: &[&str] = &[
    "--algorithm",
    "--positions",
    "--schedule",
    "--delta",
    "--stop",
    "--frames",
    "--max-rounds",
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

fn parse_schedule(text: &str) -> Result<ScheduleSpec, String> {
    if text == "round-robin" {
        return Ok(ScheduleSpec::RoundRobin);
    }
    if let Some(seed) = text.strip_prefix("shuffled:") {
        return Ok(ScheduleSpec::Shuffled {
            seed: number(seed, "--schedule shuffled")?,
        });
    }
    let (list, then) = match text.strip_suffix("+repeat") {
        Some(list) => (list, Fallback::Repeat),
        None => (text, Fallback::RoundRobin),
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
    Ok(ScheduleSpec::Explicit { order, then })
}

fn parse_stop(text: &str) -> Result<StopPolicy, String> {
    if text == "delta" {
        return Ok(StopPolicy::Delta);
    }
    if let Some(seed) = text.strip_prefix("random:") {
        return Ok(StopPolicy::Random {
            seed: number(seed, "--stop random")?,
        });
    }
    Ok(StopPolicy::Fraction {
        value: number(text, "--stop")?,
    })
}

fn parse_frames(text: &str) -> Result<FrameSpec, String> {
    if text == "global" {
        return Ok(FrameSpec::Global);
    }
    match text.strip_prefix("random:") {
        Some(seed) => Ok(FrameSpec::Random {
            seed: number(seed, "--frames random")?,
        }),
        None => Err(format!(
            "--frames: expected global or random:SEED, got {text:?}"
        )),
    }
}

/// The configuration from an optional file plus flags.
pub fn configure(args: &[String]) -> Result<SeqConfig, String> {
    let file = args.first().filter(|a| !a.starts_with("--"));
    let mut config = match file {
        Some(path) => {
            let text = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
            serde_json::from_str(&text).map_err(|e| format!("{path}: {e}"))?
        }
        None => SeqConfig::default(),
    };
    for (i, a) in args.iter().enumerate() {
        let flag_value = i > 0 && VALUED.contains(&args[i - 1].as_str());
        if a.starts_with("--") {
            let known = VALUED.contains(&a.as_str()) || a == "--rigid" || a == "--print-config";
            if !known {
                return Err(format!("unknown option {a}\n{USAGE}"));
            }
        } else if i > 0 && !flag_value {
            return Err(format!("unexpected argument {a:?}\n{USAGE}"));
        }
    }
    if let Some(name) = value(args, "--algorithm")? {
        name.clone_into(&mut config.algorithm);
    }
    if let Some(text) = value(args, "--positions")? {
        config.positions = parse_positions(text)?;
    }
    if let Some(text) = value(args, "--schedule")? {
        config.schedule = parse_schedule(text)?;
    }
    let delta = value(args, "--delta")?
        .map(|d| number::<f64>(d, "--delta"))
        .transpose()?;
    let stop = value(args, "--stop")?.map(parse_stop).transpose()?;
    if args.iter().any(|a| a == "--rigid") {
        if delta.is_some() || stop.is_some() {
            return Err("--rigid cannot be combined with --delta or --stop".to_owned());
        }
        config.movement = MovementSpec::Rigid;
    } else if delta.is_some() || stop.is_some() {
        let (old_delta, old_stop) = match config.movement {
            MovementSpec::NonRigid { delta, stop } => (delta, stop),
            MovementSpec::Rigid => (1.0, StopPolicy::Delta),
        };
        config.movement = MovementSpec::NonRigid {
            delta: delta.unwrap_or(old_delta),
            stop: stop.unwrap_or(old_stop),
        };
    }
    if let Some(text) = value(args, "--frames")? {
        config.frames = parse_frames(text)?;
    }
    if let Some(text) = value(args, "--max-rounds")? {
        config.max_rounds = number(text, "--max-rounds")?;
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

pub fn table_row(round: &Round) -> String {
    let outcome = if round.destination == round.position {
        "stays".to_owned()
    } else if round.reached {
        "reached".to_owned()
    } else {
        format!("stopped after {}", trim(round.travelled))
    };
    format!(
        "{:>5} {:>3} {:>4}  κ={} b={}  {:<24} lines {:<7} {} -> {}  end {}  [{}]  sees: {}",
        round.round,
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

pub fn summary(report: &Report) -> String {
    match report.status {
        Status::Gathered => format!(
            "GATHERED at {} after {} rounds ({} complete epochs)",
            pt(report.gathered_at.unwrap_or_default()),
            report.rounds,
            report.epochs_completed
        ),
        Status::Incomplete | Status::Stalled => report.note.clone(),
        Status::Running => format!("RUNNING after {} rounds", report.rounds),
    }
}

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
    let mut run = SqGatheringRun::new(config).map_err(|e| e.to_string())?;
    if to_stdout && format == "table" {
        println!(
            "{:>5} {:>3} {:>4}  {:<9}  {:<24} {:<13} from -> destination  end  [outcome]  sees: (* = self, M = multiplicity, s = single)",
            "round", "ep", "robot", "observes", "rule", "Alg. 8"
        );
    }
    let mut failure = None;
    let report = run.run_with(|round| {
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
    Ok(if report.status == Status::Gathered {
        0
    } else {
        3
    })
}
