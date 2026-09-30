//! Robot state as a structure of arrays (docs/core-schema.md §3).

use crate::algorithm::TaskColor;
use crate::geom::{dist, interpolate, Circle, Point};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RobotState {
    Wait = 0,
    Look = 1,
    Move = 2,
    Crash = 3,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FaultKind {
    None = 0,
    Crash = 1,
    Byzantine = 2,
    Omission = 3,
    Delay = 4,
}

/// Luminous-robot light, set by the model per phase.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Light {
    Blue = 1,
    Red = 2,
    Green = 3,
}

/// Minimum time between two light changes (`Robot.set_light`).
pub const LIGHT_COOLDOWN: f64 = 0.5;

#[derive(Clone, Debug, Default)]
pub struct Robots {
    pub pos: Vec<Point>,
    pub start_pos: Vec<Point>,
    pub target: Vec<Option<Point>>,
    pub start_time: Vec<Option<f64>>,
    pub speed: Vec<f64>,
    pub state: Vec<RobotState>,
    pub frozen: Vec<bool>,
    pub terminated: Vec<bool>,
    pub fault: Vec<FaultKind>,
    pub light: Vec<Option<Light>>,
    pub last_light_time: Vec<f64>,
    pub algo: Vec<u8>,
    pub task: Vec<Option<TaskColor>>,
    pub overlay: Vec<Option<Circle>>,
    pub travelled: Vec<f64>,
}

impl Robots {
    pub(crate) fn new(positions: &[Point], speed: f64) -> Self {
        let n = positions.len();
        Self {
            pos: positions.to_vec(),
            start_pos: positions.to_vec(),
            target: vec![None; n],
            start_time: vec![None; n],
            speed: vec![speed; n],
            state: vec![RobotState::Wait; n],
            frozen: vec![false; n],
            terminated: vec![false; n],
            fault: vec![FaultKind::None; n],
            light: vec![None; n],
            last_light_time: vec![-1.0; n],
            algo: vec![0; n],
            task: vec![None; n],
            overlay: vec![None; n],
            travelled: vec![0.0; n],
        }
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.pos.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.pos.is_empty()
    }

    /// `Robot.get_position(t)`.
    #[must_use]
    pub fn position_at(&self, i: usize, t: f64, eps: f64) -> Point {
        let (RobotState::Move, Some(start), Some(target)) =
            (self.state[i], self.start_time[i], self.target[i])
        else {
            return self.pos[i];
        };
        let from = self.start_pos[i];
        let total = dist(from, target);
        if total < 1e-9 {
            return target;
        }
        let covered = self.speed[i] * (t - start);
        if covered >= total - eps {
            target
        } else {
            interpolate(from, target, covered / total)
        }
    }

    /// `Robot.set_light`.
    pub(crate) fn set_light(&mut self, i: usize, light: Light, t: f64) {
        if self.light[i] != Some(light) && t - self.last_light_time[i] >= LIGHT_COOLDOWN {
            self.light[i] = Some(light);
            self.last_light_time[i] = t;
        }
    }
}
