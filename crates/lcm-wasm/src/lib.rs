//! Browser bindings for `lcm-core`.
//!
//! A thin adapter: it parses the configuration, drives `Simulation::advance`
//! and copies frames into typed arrays. No simulation logic lives here.
//! Typed arrays returned by the frame getters are fresh copies, so the worker
//! can transfer their buffers to the page without copying again.

use lcm_algorithms::sq_trace::Tracer;
use lcm_core::{
    Budget, EventKind, Frame, RobotState, SimConfig, Simulation, StepOutcome, StopReason,
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
    Stalled = 5,
    MaxTurns = 6,
    /// Only `advance_to_move` returns it: a robot has just started a move.
    Moved = 7,
    MaxRounds = 8,
    /// An `await` schedule waits for the next turn or round (adversary mode).
    AwaitingSchedule = 9,
}

impl From<StopReason> for Stop {
    fn from(reason: StopReason) -> Self {
        match reason {
            StopReason::Budget => Self::Budget,
            StopReason::UntilTime => Self::UntilTime,
            StopReason::Ended => Self::Ended,
            StopReason::MaxEvents => Self::MaxEvents,
            StopReason::MaxTime => Self::MaxTime,
            StopReason::Stalled => Self::Stalled,
            StopReason::MaxTurns => Self::MaxTurns,
            StopReason::MaxRounds => Self::MaxRounds,
            StopReason::AwaitingSchedule => Self::AwaitingSchedule,
        }
    }
}

/// The algorithms the core knows, as JSON `[{"key": …, "name": …,
/// "sequential_only": …, "multiplicities": …}, …]`: `sequential_only` when
/// the algorithm refuses the async scheduler, `multiplicities` when it works
/// with points that hold several robots (the page rings them).
#[wasm_bindgen]
#[must_use]
pub fn algorithms() -> String {
    let list: Vec<_> = lcm_algorithms::REGISTRY
        .iter()
        .map(|e| {
            let (sequential_only, multiplicities) = lcm_algorithms::requirements(e.key);
            serde_json::json!({
                "key": e.key,
                "name": e.name,
                "sequential_only": sequential_only,
                "multiplicities": multiplicities,
            })
        })
        .collect();
    serde_json::Value::Array(list).to_string()
}

/// Where `config_json`'s settings leave the model its algorithm was designed
/// or proved for, as a JSON list of sentences (empty inside the model).
///
/// # Errors
/// Malformed JSON.
#[wasm_bindgen]
pub fn notes(config_json: &str) -> Result<String, JsError> {
    let config: SimConfig = serde_json::from_str(config_json)?;
    Ok(serde_json::to_string(&lcm_algorithms::notes(
        &config.algorithm,
        &config,
    ))?)
}

#[wasm_bindgen]
pub struct WasmSimulation {
    sim: Simulation,
    frame: Frame,
    /// Reused by `swarm_size`, so sampling it allocates nothing.
    positions: Vec<lcm_core::Point>,
    /// A time between two events at which to draw the robots that are walking, set by the page
    /// while it paces a turn. Display only: the run's own clock (`time`) is not touched.
    view_time: Option<f64>,
    /// The robot `advance_to_move` last saw start a move.
    moved: i32,
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
            positions: Vec::new(),
            view_time: None,
            moved: -1,
        })
    }

    /// Handles up to `max_events` events, stopping early before the first
    /// event after `until_time`, at a configured limit, or at the end.
    pub fn advance(&mut self, max_events: u32, until_time: f64) -> Stop {
        self.view_time = None;
        self.sim
            .advance(Budget {
                max_events: u64::from(max_events),
                until_time,
            })
            .into()
    }

    /// Handles events one by one, as `advance` does, but stops as soon as a robot starts a move
    /// (`Stop::Moved`; `moved_robot` says which), even if that is at `until_time` itself. The page
    /// uses it to skip turns in which nobody moves, and to catch the start of every walk,
    /// including one that begins at the instant the last one ends.
    pub fn advance_to_move(&mut self, max_events: u32, until_time: f64) -> Stop {
        self.view_time = None;
        for _ in 0..max_events {
            // A limit or the end first, as `advance` checks them (a budget of 0 is only that check).
            let limit = self.sim.advance(Budget {
                max_events: 0,
                until_time,
            });
            if limit != StopReason::Budget {
                return limit.into();
            }
            if self.sim.next_event_time().is_some_and(|t| t > until_time) {
                return Stop::UntilTime;
            }
            let info = self.sim.step();
            if info.outcome == StepOutcome::Moved {
                self.moved = info.robot;
                return Stop::Moved;
            }
        }
        Stop::Budget
    }

    /// The robot `advance_to_move` last saw start a move, or -1.
    #[must_use]
    pub fn moved_robot(&self) -> i32 {
        self.moved
    }

    /// When robot `i` arrives, if it is on a move right now; NaN if it is not.
    #[must_use]
    pub fn arrival_time(&self, i: u32) -> f64 {
        let i = i as usize;
        if i >= self.sim.robots().len() {
            return f64::NAN;
        }
        self.sim.arrival_time(i).unwrap_or(f64::NAN)
    }

    /// Draw the robots as they are at time `t`, a time between the last event handled and the
    /// next one (a walk has no events inside it, so this is what makes it glide). NaN, or any
    /// time before the run's clock, goes back to drawing the run's own time.
    pub fn set_view_time(&mut self, t: f64) {
        self.view_time = (t.is_finite() && t >= self.sim.time()).then_some(t);
    }

    #[must_use]
    pub fn time(&self) -> f64 {
        self.sim.time()
    }

    /// The time the current frame shows: the view time while one is set, else the run's time.
    fn display_time(&self) -> f64 {
        self.view_time.unwrap_or_else(|| self.sim.time())
    }

    /// Events handled so far. `f64` because JS numbers are exact up to 2⁵³.
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
    pub fn event_count(&self) -> f64 {
        self.sim.event_count() as f64
    }

    /// Visualize ticks among the events handled: they belong to no robot.
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
    pub fn tick_count(&self) -> f64 {
        self.sim.tick_count() as f64
    }

    /// How far the farthest robot that has not crashed is from the centre of those robots
    /// right now: a measure of how spread out the swarm is, for the page's convergence plot.
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
    pub fn swarm_size(&mut self) -> f64 {
        let t = self.sim.time();
        self.positions.clear();
        let robots = self.sim.robots();
        for i in 0..robots.len() {
            if robots.state[i] != RobotState::Crash {
                self.positions.push(self.sim.position_at(i, t));
            }
        }
        if self.positions.is_empty() {
            return 0.0;
        }
        let n = self.positions.len() as f64;
        let cx = self.positions.iter().map(|p| p.x).sum::<f64>() / n;
        let cy = self.positions.iter().map(|p| p.y).sum::<f64>() / n;
        self.positions
            .iter()
            .map(|p| (p.x - cx).hypot(p.y - cy))
            .fold(0.0, f64::max)
    }

    /// `true` under the sequential scheduler.
    #[must_use]
    pub fn sequential(&self) -> bool {
        self.sim.sequential()
    }

    /// `frames: "compass"`: each robot's compass deviation from the global
    /// north, in radians (empty otherwise).
    #[must_use]
    pub fn compass(&self) -> js_sys::Float64Array {
        js_sys::Float64Array::from(self.sim.compass_deviations())
    }

    /// How many robots see every other robot right now (mutual visibility).
    #[allow(clippy::cast_precision_loss)]
    pub fn seeing_everyone(&mut self) -> f64 {
        self.sim.seeing_everyone() as f64
    }

    /// k-Async: Looks put off so far because they would have broken the bound.
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
    pub fn deferred_looks(&self) -> f64 {
        self.sim.deferred_looks() as f64
    }

    /// SSYNC or FSYNC: robots act in rounds.
    #[must_use]
    pub fn in_rounds(&self) -> bool {
        self.sim.in_rounds()
    }

    /// SSYNC/FSYNC: rounds begun so far.
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
    pub fn round_count(&self) -> f64 {
        self.sim.round_count() as f64
    }

    /// SSYNC/FSYNC: the robots active in the round under way.
    #[must_use]
    pub fn round_members(&self) -> js_sys::Uint32Array {
        let ids: Vec<u32> = self
            .sim
            .round_members()
            .iter()
            .map(|&i| u32::try_from(i).unwrap_or(u32::MAX))
            .collect();
        js_sys::Uint32Array::from(ids.as_slice())
    }

    /// A schedule ending in `await` has run out: the run waits for the
    /// adversary's next turn (`push_turn`) or round (`push_round`).
    #[must_use]
    pub fn awaiting_schedule(&self) -> bool {
        self.sim.awaiting_schedule()
    }

    /// Adds a turn for `robot`, stopped after `stop` of the way (NaN: let it
    /// arrive, or stop it as the stop policy says).
    ///
    /// # Errors
    /// Not a sequential run with an `await` schedule, no such robot, or a
    /// stopping point the movement settings do not allow.
    pub fn push_turn(&mut self, robot: u32, stop: f64) -> Result<(), JsError> {
        let stop = stop.is_finite().then_some(stop);
        self.sim.push_turn(robot as usize, stop)?;
        Ok(())
    }

    /// Adds a round in which `robots` look and move together.
    ///
    /// # Errors
    /// Not an SSYNC run with `await` rounds, an empty round, or no such robot.
    pub fn push_round(&mut self, robots: &[u32]) -> Result<(), JsError> {
        self.sim
            .push_round(robots.iter().map(|&r| r as usize).collect())?;
        Ok(())
    }

    /// Where robot `i` would go if it looked now, `[x, y]`, without changing
    /// the run; empty if it would not compute (crashed, terminated, …).
    pub fn preview_target(&mut self, i: u32) -> Box<[f64]> {
        match self.sim.preview_target(i as usize) {
            Some(p) => Box::new([p.x, p.y]),
            None => Box::new([]),
        }
    }

    /// Sequential scheduler: turns taken so far (one per Look).
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
    pub fn turn_count(&self) -> f64 {
        self.sim.turn_count() as f64
    }

    /// Sequential and SSYNC/FSYNC: epochs in which every live robot has been active.
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
    pub fn epochs_completed(&self) -> f64 {
        self.sim.epochs_completed() as f64
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
        self.view_time = None;
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

    /// What robot `i` would see and which rule it would apply if it looked
    /// now, in its algorithm's words (`undefined` if it would not compute or
    /// the algorithm gives no explanation). Changes nothing about the run.
    pub fn explain(&mut self, i: u32) -> Option<String> {
        self.sim.explain(i as usize)
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
        let p = self.sim.position_at(i, self.display_time());
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
            "goal": point(r.goal(i)),
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
        self.sim.write_frame(self.display_time(), &mut self.frame);
    }

    #[must_use]
    pub fn frame_time(&self) -> f64 {
        self.frame.time
    }

    /// World coordinates the frame's positions, targets and circle centres are
    /// measured from (a robot's world position is origin + offset, added in f64).
    #[must_use]
    pub fn frame_origin_x(&self) -> f64 {
        self.frame.origin[0]
    }

    #[must_use]
    pub fn frame_origin_y(&self) -> f64 {
        self.frame.origin[1]
    }

    /// `[x0, y0, x1, y1, …]`, offsets from the frame origin.
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

// ── SqGathering, turn by turn ───────────────────────────────────────────────
// The run is an ordinary core simulation on the sequential scheduler; these
// add the per-turn trace (observation, κ, rule) the step-by-step page shows.

/// Runs a SqGathering `SimConfig` to the end: `{"report": …, "trace": [turn,
/// …]}` (`trace` empty unless `with_trace`). Same output as
/// `lcm seq --format json --trace FILE`.
///
/// # Errors
/// Malformed JSON, or a configuration the core or SqGathering refuses.
#[wasm_bindgen]
pub fn sq_gathering_trace(config_json: &str, with_trace: bool) -> Result<String, JsError> {
    let config: SimConfig = serde_json::from_str(config_json)?;
    let mut tracer = Tracer::new(&config)?;
    let mut trace = Vec::new();
    let report = tracer.run_with(|r| {
        if with_trace {
            trace.push(r.clone());
        }
    });
    Ok(serde_json::json!({ "report": report, "trace": trace }).to_string())
}

/// A SqGathering run played one turn at a time (the step-by-step page).
#[wasm_bindgen]
pub struct WasmSqTrace {
    tracer: Tracer,
}

#[wasm_bindgen]
impl WasmSqTrace {
    /// # Errors
    /// Malformed JSON, or a configuration the core or SqGathering refuses.
    #[wasm_bindgen(constructor)]
    pub fn new(config_json: &str) -> Result<WasmSqTrace, JsError> {
        let config: SimConfig = serde_json::from_str(config_json)?;
        Ok(Self {
            tracer: Tracer::new(&config)?,
        })
    }

    /// Plays the next turn and returns it as JSON, or `null` once the run
    /// has stopped.
    pub fn step(&mut self) -> String {
        self.tracer.next_round().map_or_else(
            || "null".to_owned(),
            |r| serde_json::to_string(&r).unwrap_or_default(),
        )
    }

    /// `"running"`, `"gathered"`, `"incomplete"` or `"stalled"`.
    #[must_use]
    pub fn status(&self) -> String {
        serde_json::to_value(self.tracer.status())
            .ok()
            .and_then(|v| v.as_str().map(str::to_owned))
            .unwrap_or_default()
    }

    /// The start positions `[x0, y0, x1, y1, …]` (stacked starts applied).
    #[must_use]
    pub fn start_positions(&self) -> js_sys::Float64Array {
        let flat: Vec<f64> = self
            .tracer
            .simulation()
            .robots()
            .start_pos
            .iter()
            .flat_map(|p| [p.x, p.y])
            .collect();
        js_sys::Float64Array::from(flat.as_slice())
    }

    #[must_use]
    pub fn report(&self) -> String {
        serde_json::to_string(&self.tracer.report()).unwrap_or_default()
    }
}

fn copy<T: Copy>(dst: &mut [T], src: &[T]) {
    let n = dst.len().min(src.len());
    dst[..n].copy_from_slice(&src[..n]);
}

// ----------------------------------------------------------------- ring
// GatheRRing on a ring of nodes (crate lcm-ring), for web/ring.html.

/// A ring run, stepped by the page one activation at a time.
#[wasm_bindgen]
pub struct WasmRing {
    sim: lcm_ring::RingSim,
}

#[wasm_bindgen]
impl WasmRing {
    /// `config_json` is a `RingConfig`.
    ///
    /// # Errors
    /// Malformed JSON or an invalid ring.
    #[wasm_bindgen(constructor)]
    pub fn new(config_json: &str) -> Result<WasmRing, JsError> {
        let config: lcm_ring::RingConfig = serde_json::from_str(config_json)?;
        let sim = lcm_ring::RingSim::new(&config).map_err(|e| JsError::new(&e))?;
        Ok(Self { sim })
    }

    /// One activation as JSON (`null` once the run is over).
    pub fn step(&mut self) -> String {
        self.sim.step().map_or_else(
            || "null".to_owned(),
            |t| serde_json::to_string(&t).unwrap_or_default(),
        )
    }

    /// The run now, as JSON: positions, status, counts, the task every robot
    /// would take, the next robot and what it would do.
    #[must_use]
    pub fn state(&self) -> String {
        let view = self.sim.view();
        let next = self.sim.next_robot();
        let (task, mv, rule) = lcm_ring::decide(&view, self.sim.positions[next]);
        serde_json::json!({
            "nodes": self.sim.config.nodes,
            "positions": self.sim.positions,
            "status": self.sim.status,
            "epochs": self.sim.epochs,
            "turns": self.sim.turns,
            "next": next,
            "task": task,
            "rule": rule,
            "move": match mv {
                lcm_ring::Move::Stay => 0,
                lcm_ring::Move::Step(d) => i32::from(d),
                lcm_ring::Move::Either => 2,
            },
            "holes": view.holes,
            "bound": self.sim.config.nodes.saturating_sub(3),
            "unsolvable": lcm_ring::unsolvable(&self.sim.config),
        })
        .to_string()
    }
}
