//! What an algorithm sees and returns (docs/core-schema.md §7).

use crate::geom::{Circle, Point};
use crate::rng::Rng;
use crate::robots::RobotState;

/// Model constants every algorithm may read.
#[derive(Clone, Copy, Debug)]
pub struct Model {
    /// `threshold_precision`.
    pub precision: u8,
    /// `10 ** -precision`.
    pub eps: f64,
    /// `f64::INFINITY` when unlimited.
    pub visibility: f64,
    pub width_bound: Option<f64>,
    pub height_bound: Option<f64>,
}

/// The robots one robot sees at the instant of its look, in ID order,
/// including itself. Structure of arrays in a buffer reused across looks.
#[derive(Clone, Debug, Default)]
pub struct View {
    pub id: Vec<u32>,
    pub pos: Vec<Point>,
    pub state: Vec<RobotState>,
    pub terminated: Vec<bool>,
    pub frozen: Vec<bool>,
}

impl View {
    #[must_use]
    pub fn len(&self) -> usize {
        self.id.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.id.is_empty()
    }

    pub(crate) fn clear(&mut self) {
        self.id.clear();
        self.pos.clear();
        self.state.clear();
        self.terminated.clear();
        self.frozen.clear();
    }

    pub(crate) fn push(
        &mut self,
        id: u32,
        pos: Point,
        state: RobotState,
        terminated: bool,
        frozen: bool,
    ) {
        self.id.push(id);
        self.pos.push(pos);
        self.state.push(state);
        self.terminated.push(terminated);
        self.frozen.push(frozen);
    }

    /// Positions of robots that have not crashed, in view order.
    pub fn live_positions(&self) -> impl Iterator<Item = Point> + '_ {
        self.pos
            .iter()
            .zip(&self.state)
            .filter(|(_, s)| **s != RobotState::Crash)
            .map(|(p, _)| *p)
    }
}

/// One robot's look: its identity and position, what it sees, and the model.
pub struct Look<'a> {
    pub me: u32,
    pub me_index: usize,
    pub position: Point,
    pub view: &'a View,
    pub model: &'a Model,
}

/// Change to the circle a robot displays (Python `robot.sec`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum OverlayUpdate {
    Keep,
    Set(Circle),
    Clear,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Decision {
    pub target: Point,
    pub terminate: bool,
    pub overlay: OverlayUpdate,
}

/// Reusable memory for `compute`. Robots are oblivious: nothing here may
/// carry meaning from one look to the next.
#[derive(Debug, Default)]
pub struct Scratch {
    pub points: Vec<Point>,
    pub points_b: Vec<Point>,
    pub floats: Vec<f64>,
    pub indices: Vec<usize>,
    pub indices_b: Vec<usize>,
}

pub trait Algorithm {
    /// Registry key, e.g. `"Gathering"`.
    fn key(&self) -> &'static str;

    fn compute(&self, look: &Look<'_>, rng: &mut Rng, scratch: &mut Scratch) -> Decision;
}

/// Colour marking which task a robot was given (TwoTask).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TaskColor {
    Red,
    Blue,
}

/// How robots are assigned algorithms.
pub enum Assignment {
    /// Every robot runs `algorithms[0]`.
    Single,
    /// Per robot, in ID order, draw `u = random()` and take the first entry
    /// with `u < cumulative`: `(cumulative, algorithm index, colour)`.
    Random(Vec<(f64, usize, Option<TaskColor>)>),
}

/// The algorithms of one simulation and how robots get them.
pub struct Plan {
    pub algorithms: Vec<Box<dyn Algorithm>>,
    pub assignment: Assignment,
}

impl Plan {
    #[must_use]
    pub fn single(algorithm: Box<dyn Algorithm>) -> Self {
        Self {
            algorithms: vec![algorithm],
            assignment: Assignment::Single,
        }
    }
}
