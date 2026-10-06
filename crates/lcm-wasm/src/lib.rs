//! Browser bindings for `lcm-core`.
//!
//! A thin adapter: it parses the configuration, drives `Simulation::advance`
//! and copies frames into typed arrays. No simulation logic lives here.
//! Typed arrays returned by the frame getters are fresh copies, so the worker
//! can transfer their buffers to the page without copying again.

use lcm_algorithms::seq::{Round, SeqConfig, SeqOptions, SqGatheringRun, Status};
use lcm_algorithms::sq_gathering::occupancy;
use lcm_core::{
    Budget, EventKind, Frame, RobotState, SimConfig, Simulation, StopReason, FLAG_HAS_TARGET,
    FLAG_TERMINATED,
};
use wasm_bindgen::prelude::*;

/// `advance` result codes, shared with `web/worker.js`.
#[wasm_bindgen]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Stop {
    Budget = 0,
    UntilTime = 1,
    Ended = 2,
    MaxEvents = 3,
    MaxTime = 4,
    /// A sequential run hit the floating-point limit (`Status::Stalled`).
    Stalled = 5,
}

impl From<StopReason> for Stop {
    fn from(reason: StopReason) -> Self {
        match reason {
            StopReason::Budget => Self::Budget,
            StopReason::UntilTime => Self::UntilTime,
            StopReason::Ended => Self::Ended,
            StopReason::MaxEvents => Self::MaxEvents,
            StopReason::MaxTime => Self::MaxTime,
        }
    }
}

/// The algorithms the page can run, as JSON
/// `[{"key": …, "name": …, "group": "async" | "sequential", "detail"?: …}, …]`:
/// the asynchronous registry, then the sequential algorithms (run with
/// `WasmSeqSimulation`).
#[wasm_bindgen]
#[must_use]
pub fn algorithms() -> String {
    let mut list: Vec<_> = lcm_algorithms::REGISTRY
        .iter()
        .map(|e| serde_json::json!({ "key": e.key, "name": e.name, "group": "async" }))
        .collect();
    list.push(serde_json::json!({
        "key": "SqGathering",
        "name": "SqGathering",
        "group": "sequential",
        "detail": "Algorithm 8 · weak multiplicity detection",
    }));
    serde_json::Value::Array(list).to_string()
}

#[wasm_bindgen]
pub struct WasmSimulation {
    sim: Simulation,
    frame: Frame,
}

#[wasm_bindgen]
impl WasmSimulation {
    /// `config_json` is a `SimConfig` (docs/core-schema.md §2).
    ///
    /// # Errors
    /// Malformed JSON, an invalid field or an unknown algorithm.
    #[wasm_bindgen(constructor)]
    pub fn new(config_json: &str) -> Result<WasmSimulation, JsError> {
        let config: SimConfig = serde_json::from_str(config_json)?;
        let sim = lcm_algorithms::simulation(&config)?;
        Ok(Self {
            sim,
            frame: Frame::default(),
        })
    }

    /// Handles up to `max_events` events, stopping early before the first
    /// event after `until_time`, at a configured limit, or at the end.
    pub fn advance(&mut self, max_events: u32, until_time: f64) -> Stop {
        self.sim
            .advance(Budget {
                max_events: u64::from(max_events),
                until_time,
            })
            .into()
    }

    #[must_use]
    pub fn time(&self) -> f64 {
        self.sim.time()
    }

    /// Events handled so far. `f64` because JS numbers are exact up to 2⁵³.
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
    pub fn event_count(&self) -> f64 {
        self.sim.event_count() as f64
    }

    #[must_use]
    pub fn ended(&self) -> bool {
        self.sim.ended()
    }

    #[must_use]
    pub fn robot_count(&self) -> u32 {
        u32::try_from(self.sim.robots().len()).unwrap_or(u32::MAX)
    }

    #[must_use]
    pub fn terminated_count(&self) -> u32 {
        let n = self.sim.robots().terminated.iter().filter(|t| **t).count();
        u32::try_from(n).unwrap_or(u32::MAX)
    }

    /// Handles exactly one event and reports it as
    /// `[time, robot (-1 = none), kind, outcome]`: kind 0 crash, 1 look,
    /// 2 wait, 3 visualize, -1 none; outcome is `StepOutcome::code`.
    pub fn step_one(&mut self) -> Box<[f64]> {
        let info = self.sim.step();
        let kind = match info.kind {
            Some(EventKind::Crash) => 0.0,
            Some(EventKind::Look) => 1.0,
            Some(EventKind::Wait) => 2.0,
            Some(EventKind::Visualize) => 3.0,
            None => -1.0,
        };
        Box::new([
            info.time,
            f64::from(info.robot),
            kind,
            f64::from(info.outcome.code()),
        ])
    }

    /// Everything known about robot `i` at the current time, as JSON; `null`
    /// for an index out of range.
    #[must_use]
    pub fn robot_info(&self, i: u32) -> String {
        let r = self.sim.robots();
        let i = i as usize;
        if i >= r.len() {
            return "null".to_owned();
        }
        let p = self.sim.position_at(i, self.sim.time());
        let point = |p: Option<lcm_core::Point>| p.map(|p| [p.x, p.y]);
        serde_json::json!({
            "id": i,
            "algorithm": self.sim.algorithm_key(i),
            "x": p.x,
            "y": p.y,
            "state": format!("{:?}", r.state[i]),
            "frozen": r.frozen[i],
            "terminated": r.terminated[i],
            "fault": format!("{:?}", r.fault[i]),
            "light": r.light[i].map(|l| format!("{l:?}")),
            "task": r.task[i].map(|t| format!("{t:?}")),
            "target": point(r.target[i]),
            "move_start": point(r.start_time[i].map(|_| r.start_pos[i])),
            "speed": r.speed[i],
            "travelled": r.travelled[i],
            "circle": r.overlay[i].map(|c| [c.center.x, c.center.y, c.radius]),
        })
        .to_string()
    }

    /// Every robot at the current time as CSV, in full precision.
    #[must_use]
    pub fn export_csv(&self) -> String {
        use std::fmt::Write as _;
        let r = self.sim.robots();
        let t = self.sim.time();
        let mut out = String::from(
            "id,x,y,state,frozen,terminated,fault,light,task,target_x,target_y,speed,travelled,algorithm\n",
        );
        for i in 0..r.len() {
            let p = self.sim.position_at(i, t);
            let (tx, ty) = r.target[i].map_or((String::new(), String::new()), |q| {
                (q.x.to_string(), q.y.to_string())
            });
            let _ = writeln!(
                out,
                "{i},{},{},{:?},{},{},{:?},{},{},{tx},{ty},{},{},{}",
                p.x,
                p.y,
                r.state[i],
                r.frozen[i],
                r.terminated[i],
                r.fault[i],
                r.light[i].map_or(String::new(), |l| format!("{l:?}")),
                r.task[i].map_or(String::new(), |c| format!("{c:?}")),
                r.speed[i],
                r.travelled[i],
                self.sim.algorithm_key(i),
            );
        }
        out
    }

    /// Captures the current state straight into caller-owned arrays, so the
    /// page can hand the same buffers back every frame instead of allocating
    /// new ones. Each array must have the length the getters below return.
    #[allow(clippy::too_many_arguments)]
    pub fn fill_frame(
        &mut self,
        xy: &mut [f32],
        flags: &mut [u8],
        light: &mut [u8],
        fault: &mut [u8],
        task: &mut [u8],
        target: &mut [f32],
        circle: &mut [f32],
    ) {
        self.capture_frame();
        let f = &self.frame;
        copy(xy, &f.xy);
        copy(flags, &f.flags);
        copy(light, &f.light);
        copy(fault, &f.fault);
        copy(task, &f.task);
        copy(target, &f.target);
        copy(circle, &f.circle);
    }

    /// Captures the state at the current time; read it with the getters below.
    pub fn capture_frame(&mut self) {
        self.sim.write_frame(self.sim.time(), &mut self.frame);
    }

    #[must_use]
    pub fn frame_time(&self) -> f64 {
        self.frame.time
    }

    /// `[x0, y0, x1, y1, …]`
    #[must_use]
    pub fn xy(&self) -> js_sys::Float32Array {
        js_sys::Float32Array::from(self.frame.xy.as_slice())
    }

    /// Per robot: bits 0–1 state, bit 2 frozen, bit 3 terminated, bit 4 has target.
    #[must_use]
    pub fn flags(&self) -> js_sys::Uint8Array {
        js_sys::Uint8Array::from(self.frame.flags.as_slice())
    }

    /// Per robot: 0 none, 1 blue, 2 red, 3 green.
    #[must_use]
    pub fn light(&self) -> js_sys::Uint8Array {
        js_sys::Uint8Array::from(self.frame.light.as_slice())
    }

    /// Per robot: 0 none, 1 crash, 2 byzantine, 3 omission, 4 delay.
    #[must_use]
    pub fn fault(&self) -> js_sys::Uint8Array {
        js_sys::Uint8Array::from(self.frame.fault.as_slice())
    }

    /// Per robot: 0 none, 1 red, 2 blue.
    #[must_use]
    pub fn task(&self) -> js_sys::Uint8Array {
        js_sys::Uint8Array::from(self.frame.task.as_slice())
    }

    /// `[tx0, ty0, …]`, NaN where a robot has no target.
    #[must_use]
    pub fn target(&self) -> js_sys::Float32Array {
        js_sys::Float32Array::from(self.frame.target.as_slice())
    }

    /// `[cx0, cy0, r0, …]`, NaN where a robot has no circle.
    #[must_use]
    pub fn circle(&self) -> js_sys::Float32Array {
        js_sys::Float32Array::from(self.frame.circle.as_slice())
    }

    /// Robots sharing each robot's point; empty unless multiplicity detection is on.
    #[must_use]
    pub fn multiplicity(&self) -> js_sys::Uint16Array {
        js_sys::Uint16Array::from(self.frame.multiplicity.as_slice())
    }
}

// ── Sequential scheduler: Algorithm 8, SqGathering ─────────────────────────
// Separate from `WasmSimulation`: these run one robot per round under the
// sequential scheduler, not the asynchronous event loop.

/// The sequential-scheduler algorithms, as JSON `["SqGathering"]`.
#[wasm_bindgen]
#[must_use]
pub fn seq_algorithms() -> String {
    serde_json::to_string(lcm_algorithms::seq::SEQ_ALGORITHMS).unwrap_or_default()
}

/// Runs a `SeqConfig` to the end: `{"report": …, "trace": [round, …]}`
/// (`trace` empty unless `with_trace`). Same output as `lcm seq --format json`.
///
/// # Errors
/// Malformed JSON or an invalid configuration.
#[wasm_bindgen]
pub fn sq_gathering_run(config_json: &str, with_trace: bool) -> Result<String, JsError> {
    let config: SeqConfig = serde_json::from_str(config_json)?;
    let mut run = SqGatheringRun::new(config)?;
    let mut trace = Vec::new();
    let report = run.run_with(|r| {
        if with_trace {
            trace.push(r.clone());
        }
    });
    Ok(serde_json::json!({ "report": report, "trace": trace }).to_string())
}

/// A sequential run played one round at a time (the web demo).
#[wasm_bindgen]
pub struct WasmSqGathering {
    run: SqGatheringRun,
}

#[wasm_bindgen]
impl WasmSqGathering {
    /// # Errors
    /// Malformed JSON or an invalid configuration.
    #[wasm_bindgen(constructor)]
    pub fn new(config_json: &str) -> Result<WasmSqGathering, JsError> {
        let config: SeqConfig = serde_json::from_str(config_json)?;
        Ok(Self {
            run: SqGatheringRun::new(config)?,
        })
    }

    /// Plays one round and returns it as JSON, or `null` once finished.
    pub fn step(&mut self) -> String {
        self.run.step().map_or_else(
            || "null".to_owned(),
            |r| serde_json::to_string(&r).unwrap_or_default(),
        )
    }

    /// `"running"`, `"gathered"` or `"incomplete"`.
    #[must_use]
    pub fn status(&self) -> String {
        serde_json::to_value(self.run.status())
            .ok()
            .and_then(|v| v.as_str().map(str::to_owned))
            .unwrap_or_default()
    }

    #[must_use]
    pub fn round(&self) -> f64 {
        #[allow(clippy::cast_precision_loss)]
        let r = self.run.round() as f64;
        r
    }

    /// `[x0, y0, x1, y1, …]` now.
    #[must_use]
    pub fn positions(&self) -> js_sys::Float64Array {
        let flat: Vec<f64> = self
            .run
            .positions()
            .iter()
            .flat_map(|p| [p.x, p.y])
            .collect();
        js_sys::Float64Array::from(flat.as_slice())
    }

    /// What robot `i` would see and decide if activated now, as JSON
    /// `{observation: [{at, multiplicity}], self_index, kappa, on_multiplicity,
    /// rule, lines, description, destination}`; `null` if out of range.
    #[must_use]
    pub fn preview(&self, i: u32) -> String {
        let i = i as usize;
        if i >= self.run.positions().len() {
            return "null".to_owned();
        }
        let (obs, d) = self.run.preview(i);
        let dest = d.destination(&obs);
        serde_json::json!({
            "observation": obs.points.iter().map(|o| serde_json::json!({
                "at": [o.at.x, o.at.y], "multiplicity": o.multiplicity,
            })).collect::<Vec<_>>(),
            "self_index": obs.me,
            "kappa": d.kappa,
            "on_multiplicity": d.on_multiplicity,
            "rule": d.rule.key(),
            "lines": d.rule.lines(),
            "description": d.rule.description(),
            "destination": [dest.x, dest.y],
        })
        .to_string()
    }

    #[must_use]
    pub fn report(&self) -> String {
        serde_json::to_string(&self.run.report()).unwrap_or_default()
    }
}

/// A sequential algorithm behind the same interface as `WasmSimulation`, so
/// the main simulator page can run it when it is picked from the
/// "Sequential ›" menu. Time is the round number: one round = one robot's
/// whole Look-Compute-Move cycle, and an "event" is one round.
#[wasm_bindgen]
pub struct WasmSeqSimulation {
    run: SqGatheringRun,
    last: Option<Round>,
}

#[wasm_bindgen]
impl WasmSeqSimulation {
    /// `config_json` is the page's `SimConfig`; `options_json` its
    /// `SeqOptions` (`{schedule, stop, frames}`, all optional).
    ///
    /// # Errors
    /// Malformed JSON, an invalid field, or settings the algorithm does not model.
    #[wasm_bindgen(constructor)]
    pub fn new(config_json: &str, options_json: &str) -> Result<WasmSeqSimulation, JsError> {
        let config: SimConfig = serde_json::from_str(config_json)?;
        let options: SeqOptions = if options_json.trim().is_empty() {
            SeqOptions::default()
        } else {
            serde_json::from_str(options_json)?
        };
        let run = SqGatheringRun::new(SeqConfig::from_sim(&config, &options)?)?;
        Ok(Self { run, last: None })
    }

    /// Plays up to `max_events` rounds, stopping before round `t` when
    /// `t > until_time`. Codes as for `WasmSimulation::advance`; a run out of
    /// round budget stops with `MaxEvents`.
    pub fn advance(&mut self, max_events: u32, until_time: f64) -> Stop {
        for _ in 0..max_events {
            match self.run.status() {
                Status::Gathered => return Stop::Ended,
                Status::Incomplete => return Stop::MaxEvents,
                Status::Stalled => return Stop::Stalled,
                Status::Running => {}
            }
            if self.time() + 1.0 > until_time {
                return Stop::UntilTime;
            }
            self.last = self.run.step();
        }
        match self.run.status() {
            Status::Gathered => Stop::Ended,
            Status::Incomplete => Stop::MaxEvents,
            Status::Stalled => Stop::Stalled,
            Status::Running => Stop::Budget,
        }
    }

    #[must_use]
    pub fn time(&self) -> f64 {
        #[allow(clippy::cast_precision_loss)]
        let t = self.run.round() as f64;
        t
    }

    #[must_use]
    pub fn event_count(&self) -> f64 {
        self.time()
    }

    /// Gathered. A run out of budget is not ended: `advance` reports it.
    #[must_use]
    pub fn ended(&self) -> bool {
        self.run.status() == Status::Gathered
    }

    #[must_use]
    pub fn robot_count(&self) -> u32 {
        u32::try_from(self.run.positions().len()).unwrap_or(u32::MAX)
    }

    /// Robots already gathered: all of them once gathered; while there is
    /// exactly one multiplicity, the robots on it (they never leave it
    /// again unless a non-rigid stop creates a second one); otherwise 0.
    #[must_use]
    pub fn terminated_count(&self) -> u32 {
        let n = if self.ended() {
            self.run.positions().len()
        } else {
            let (_, counts, _) = occupancy(self.run.positions(), self.run.config().coincidence);
            let mut multis = counts.iter().filter(|c| **c >= 2);
            match (multis.next(), multis.next()) {
                (Some(&c), None) => c,
                _ => 0,
            }
        };
        u32::try_from(n).unwrap_or(u32::MAX)
    }

    /// Plays one round: `[round, robot, 1 (look), outcome]`, outcome 2 if the
    /// robot moved, 3 if it stayed, -1 if the run is over.
    pub fn step_one(&mut self) -> Box<[f64]> {
        match self.run.step() {
            None => Box::new([self.time(), -1.0, -1.0, -1.0]),
            Some(r) => {
                let moved = r.position != r.destination;
                #[allow(clippy::cast_precision_loss)]
                let out = Box::new([
                    r.round as f64,
                    r.robot as f64,
                    1.0,
                    if moved { 2.0 } else { 3.0 },
                ]);
                self.last = Some(r);
                out
            }
        }
    }

    /// Robot `i` now, plus what it would observe and decide if activated.
    #[must_use]
    pub fn robot_info(&self, i: u32) -> String {
        let i = i as usize;
        let positions = self.run.positions();
        if i >= positions.len() {
            return "null".to_owned();
        }
        let p = positions[i];
        let (obs, d) = self.run.preview(i);
        let target = self
            .last
            .as_ref()
            .filter(|r| r.robot == i && r.position != r.destination)
            .map(|r| r.destination);
        serde_json::json!({
            "id": i,
            "algorithm": self.run.config().algorithm,
            "x": p.x,
            "y": p.y,
            "state": if self.last.as_ref().is_some_and(|r| r.robot == i) { "Active last round" } else { "Wait" },
            "frozen": false,
            "terminated": self.ended(),
            "fault": "None",
            "light": null,
            "task": null,
            "target": target,
            "move_start": null,
            "speed": 0.0,
            "travelled": 0.0,
            "circle": null,
            "extra": [
                ["Sees", format!("κ = {}, own point {}", d.kappa, if d.on_multiplicity { "multiplicity (b = 1)" } else { "single (b = 0)" })],
                ["If activated", format!("{} (lines {})", d.rule.key(), d.rule.lines())],
                ["Occupied points", obs.points.len().to_string()],
            ],
        })
        .to_string()
    }

    #[must_use]
    pub fn export_csv(&self) -> String {
        use std::fmt::Write as _;
        let mut out = String::from("id,x,y,on_multiplicity,algorithm,round\n");
        let (_, counts, owner) = occupancy(self.run.positions(), self.run.config().coincidence);
        for (i, p) in self.run.positions().iter().enumerate() {
            let _ = writeln!(
                out,
                "{i},{},{},{},{},{}",
                p.x,
                p.y,
                counts[owner[i]] >= 2,
                self.run.config().algorithm,
                self.run.round()
            );
        }
        out
    }

    /// Same arrays as `WasmSimulation::fill_frame`. The robot active in the
    /// last round is drawn moving towards its destination; once gathered,
    /// every robot is drawn terminated. Each multiplicity point gets a ring:
    /// `circle` = (x, y, -11), a negative radius meaning screen pixels.
    #[allow(clippy::too_many_arguments, clippy::cast_possible_truncation)]
    pub fn fill_frame(
        &mut self,
        xy: &mut [f32],
        flags: &mut [u8],
        light: &mut [u8],
        fault: &mut [u8],
        task: &mut [u8],
        target: &mut [f32],
        circle: &mut [f32],
    ) {
        let done = self.ended();
        let (_, counts, owner) = occupancy(self.run.positions(), self.run.config().coincidence);
        let mut first = vec![usize::MAX; counts.len()];
        for (i, &g) in owner.iter().enumerate().rev() {
            first[g] = i;
        }
        let active = self
            .last
            .as_ref()
            .filter(|r| !done && r.position != r.destination)
            .map(|r| (r.robot, r.destination));
        for (i, p) in self.run.positions().iter().enumerate() {
            if 2 * i + 1 < xy.len() {
                xy[2 * i] = p.x as f32;
                xy[2 * i + 1] = p.y as f32;
            }
            let moving = active.filter(|(r, _)| *r == i).map(|(_, d)| d);
            if let Some(f) = flags.get_mut(i) {
                *f = match moving {
                    Some(_) => RobotState::Move as u8 | FLAG_HAS_TARGET,
                    None if done => RobotState::Wait as u8 | FLAG_TERMINATED,
                    None => RobotState::Wait as u8,
                };
            }
            if 2 * i + 1 < target.len() {
                let d = moving.map_or([f32::NAN; 2], |d| [d[0] as f32, d[1] as f32]);
                target[2 * i] = d[0];
                target[2 * i + 1] = d[1];
            }
            for a in [light.get_mut(i), fault.get_mut(i), task.get_mut(i)]
                .into_iter()
                .flatten()
            {
                *a = 0;
            }
            // One ring per multiplicity point (on its first robot): a
            // negative radius tells the renderer "this many screen pixels".
            let ring = counts[owner[i]] >= 2 && first[owner[i]] == i;
            let c = if ring {
                [p.x as f32, p.y as f32, -11.0]
            } else {
                [f32::NAN; 3]
            };
            for (k, v) in c.into_iter().enumerate() {
                if let Some(slot) = circle.get_mut(3 * i + k) {
                    *slot = v;
                }
            }
        }
    }
}

fn copy<T: Copy>(dst: &mut [T], src: &[T]) {
    let n = dst.len().min(src.len());
    dst[..n].copy_from_slice(&src[..n]);
}
