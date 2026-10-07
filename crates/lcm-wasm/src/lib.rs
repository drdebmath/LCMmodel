//! Browser bindings for `lcm-core`.
//!
//! A thin adapter: it parses the configuration, drives `Simulation::advance`
//! and copies frames into typed arrays. No simulation logic lives here.
//! Typed arrays returned by the frame getters are fresh copies, so the worker
//! can transfer their buffers to the page without copying again.

use lcm_core::{Budget, EventKind, Frame, SimConfig, Simulation, StopReason};
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
        }
    }
}

/// The algorithms the core knows, as JSON `[{"key": …, "name": …}, …]`.
#[wasm_bindgen]
#[must_use]
pub fn algorithms() -> String {
    let list: Vec<_> = lcm_algorithms::REGISTRY
        .iter()
        .map(|e| serde_json::json!({ "key": e.key, "name": e.name }))
        .collect();
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

    /// Visualize ticks among the events handled: they belong to no robot.
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
    pub fn tick_count(&self) -> f64 {
        self.sim.tick_count() as f64
    }

    /// `true` under the sequential scheduler.
    #[must_use]
    pub fn sequential(&self) -> bool {
        self.sim.sequential()
    }

    /// Sequential scheduler: turns taken so far (one per Look).
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
    pub fn turn_count(&self) -> f64 {
        self.sim.turn_count() as f64
    }

    /// Sequential scheduler: epochs in which every live robot has had its turn.
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

fn copy<T: Copy>(dst: &mut [T], src: &[T]) {
    let n = dst.len().min(src.len());
    dst[..n].copy_from_slice(&src[..n]);
}
