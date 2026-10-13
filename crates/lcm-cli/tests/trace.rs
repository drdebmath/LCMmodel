//! `lcm run --trace`: what every robot event did, as JSON lines or CSV.

use std::path::PathBuf;
use std::process::{Command, Output};

/// The doc's three-robot chain: A and C each see only B, B sees both.
const CHAIN: &str = r#"{"algorithm":"GoToCenter","num_of_robots":3,
  "initial_positions":[[-80,10],[0,0],[95,-5]],"visibility_radius":100,
  "scheduler":"sequential","turn_gap":"none","robot_speeds":20,"sampling_rate":1000}"#;

fn scratch(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("lcm-cli-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir.join(name)
}

fn lcm(args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_lcm"))
        .args(args)
        .output()
        .expect("runs")
}

fn config(name: &str, json: &str) -> PathBuf {
    let path = scratch(name);
    std::fs::write(&path, json).unwrap();
    path
}

fn run_traced(json: &str, trace_name: &str, extra: &[&str]) -> (serde_json::Value, String) {
    let cfg = config(&format!("{trace_name}.config.json"), json);
    let trace = scratch(trace_name);
    let mut args = vec![
        "run",
        cfg.to_str().unwrap(),
        "--trace",
        trace.to_str().unwrap(),
    ];
    args.extend_from_slice(extra);
    let out = lcm(&args);
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let summary = serde_json::from_slice(&out.stdout).expect("a JSON summary");
    (summary, std::fs::read_to_string(trace).unwrap())
}

fn records(text: &str) -> Vec<serde_json::Value> {
    text.lines()
        .map(|l| serde_json::from_str(l).expect("a JSON object per line"))
        .collect()
}

fn point(v: &serde_json::Value) -> (f64, f64) {
    (v[0].as_f64().unwrap(), v[1].as_f64().unwrap())
}

#[test]
fn the_first_look_is_the_hand_worked_one() {
    let (summary, text) = run_traced(CHAIN, "chain.jsonl", &[]);
    let rows = records(&text);
    assert_eq!(summary["traced"].as_u64().unwrap() as usize, rows.len());
    // Robot A(-80,10) sees only B (80.6 away): the smallest circle has AB as a diameter, centred
    // (-40,5) with radius 40.311, and the limit from B (90.3) does not bind the 40.3 to go.
    let first = &rows[0];
    assert_eq!(
        (first["kind"].as_str(), first["robot"].as_i64()),
        (Some("look"), Some(0))
    );
    assert_eq!(first["visible"], serde_json::json!([1]));
    assert_eq!(
        (first["x"].as_f64(), first["y"].as_f64()),
        (Some(-80.0), Some(10.0))
    );
    let (tx, ty) = point(&first["target"]);
    assert!(
        (tx + 40.0).abs() < 1e-4 && (ty - 5.0).abs() < 1e-4,
        "target ({tx}, {ty})"
    );
    let circle = first["circle"].as_array().unwrap();
    assert!((circle[2].as_f64().unwrap() - 40.311_288_741_5).abs() < 1e-4);
    assert_eq!(first["outcome"], "moved");
    assert_eq!(rows[1]["kind"], "arrive", "the move ends with an arrival");
    assert_eq!(rows[1]["outcome"], "arrived");
}

#[test]
fn robots_are_listed_by_who_they_can_see() {
    let (_, text) = run_traced(CHAIN, "chain-sight.jsonl", &[]);
    let rows = records(&text);
    // Right after A moved to (-40,5), B(0,0) sees A and C(95,-5) (95.1 away); C sees only B.
    let b = rows
        .iter()
        .find(|r| r["kind"] == "look" && r["robot"] == 1)
        .unwrap();
    assert_eq!(b["visible"], serde_json::json!([0, 2]));
    let c = rows
        .iter()
        .find(|r| r["kind"] == "look" && r["robot"] == 2)
        .unwrap();
    assert!(
        c["visible"]
            .as_array()
            .unwrap()
            .iter()
            .all(|v| v == 1 || v == 0),
        "{c}"
    );
    // Only Looks list who is in sight.
    assert!(rows
        .iter()
        .filter(|r| r["kind"] != "look")
        .all(|r| r.get("visible").is_none()));
}

#[test]
fn a_whole_run_is_in_order_and_ends_with_everyone_done() {
    let (summary, text) = run_traced(CHAIN, "chain-run.jsonl", &[]);
    let rows = records(&text);
    assert!(rows
        .windows(2)
        .all(|w| w[0]["event"].as_u64() < w[1]["event"].as_u64()));
    assert!(rows
        .windows(2)
        .all(|w| w[0]["time"].as_f64() <= w[1]["time"].as_f64()));
    assert_eq!(summary["stopped"], "Ended");
    assert_eq!(summary["terminated"], 3);
    let last_look = rows.iter().rev().find(|r| r["kind"] == "look").unwrap();
    assert_eq!(last_look["terminated"], true);
    assert!(
        rows.iter().all(|r| r["kind"] != "tick"),
        "ticks are left out unless asked for"
    );
}

#[test]
fn ticks_are_added_on_request_and_then_every_event_is_there() {
    // A short sampling interval, so that ticks fall inside the run.
    let chain = CHAIN.replace(r#""sampling_rate":1000"#, r#""sampling_rate":1"#);
    let (summary, text) = run_traced(&chain, "chain-ticks.jsonl", &["--ticks"]);
    let rows = records(&text);
    assert!(rows.iter().any(|r| r["kind"] == "tick"));
    assert_eq!(rows.len() as u64, summary["events"].as_u64().unwrap());
    assert_eq!(
        rows.last().unwrap()["event"].as_u64(),
        summary["events"].as_u64()
    );
}

#[test]
fn a_csv_name_gives_a_spreadsheet_with_the_same_rows() {
    let (_, json) = run_traced(CHAIN, "same.jsonl", &[]);
    let (_, csv) = run_traced(CHAIN, "same.csv", &[]);
    let lines: Vec<&str> = csv.lines().collect();
    assert!(lines[0].starts_with("event,time,robot,kind,outcome,x,y,target_x"));
    assert_eq!(lines.len() - 1, records(&json).len());
    let columns = lines[0].split(',').count();
    assert!(
        lines.iter().all(|l| l.split(',').count() == columns),
        "ragged rows"
    );
    let first: Vec<&str> = lines[1].split(',').collect();
    assert_eq!(
        (first[2], first[3], first[4], first[12]),
        ("0", "look", "moved", "1")
    );
}

#[test]
fn a_run_stopped_by_a_limit_says_so_and_the_trace_ends_there() {
    let limited = CHAIN.replace(
        r#""sampling_rate":1000"#,
        r#""sampling_rate":1000,"max_turns":2"#,
    );
    let (summary, text) = run_traced(&limited, "limited.jsonl", &[]);
    assert_eq!(summary["stopped"], "MaxTurns");
    assert_eq!(summary["turns"], 2);
    let looks = records(&text)
        .into_iter()
        .filter(|r| r["kind"] == "look")
        .count();
    assert_eq!(looks, 2);
}

#[test]
fn the_trace_does_not_change_the_run() {
    let cfg = config("plain.config.json", CHAIN);
    let plain = lcm(&["run", cfg.to_str().unwrap()]);
    let mut plain: serde_json::Value = serde_json::from_slice(&plain.stdout).unwrap();
    let (mut traced, _) = run_traced(CHAIN, "same-run.jsonl", &[]);
    for v in [&mut plain, &mut traced] {
        let map = v.as_object_mut().unwrap();
        map.remove("wall_seconds");
        map.remove("traced");
    }
    assert_eq!(plain, traced);
}

#[test]
fn bad_arguments_are_refused_with_a_message() {
    let cfg = config("args.config.json", CHAIN);
    let out = lcm(&["run", cfg.to_str().unwrap(), "--trace"]);
    assert_eq!(out.status.code(), Some(2));
    assert!(String::from_utf8_lossy(&out.stderr).contains("--trace needs a file name"));
}
