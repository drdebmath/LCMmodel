//! Deterministic Look-Compute-Move simulation core.
//!
//! No DOM, file I/O, threads or GPU: the same code runs natively and inside a
//! browser worker. The contract is `docs/core-schema.md`; the behavioural
//! reference is the Python simulator in the repository root.

mod algorithm;
mod config;
mod event;
mod frame;
pub mod geom;
pub mod pyfloat;
mod rng;
mod robots;
mod sim;
pub mod start;

pub use algorithm::{
    Algorithm, Assignment, Decision, Look, Model, OverlayUpdate, Plan, Scratch, TaskColor, View,
};
pub use config::{
    ActivationOrder, ConfigError, FaultSelection, SchedulerKind, SimConfig, TurnGap, MAX_ROBOTS,
};
pub use event::{Event, EventKind, EventQueue};
pub use frame::{flag_state, Frame, FLAG_FROZEN, FLAG_HAS_TARGET, FLAG_TERMINATED};
pub use geom::{Circle, Point};
pub use rng::Rng;
pub use robots::{FaultKind, Light, RobotState, Robots, LIGHT_COOLDOWN};
pub use sim::{Budget, Simulation, StepInfo, StepOutcome, StopReason};
pub use start::{Arrangement, Distribution, Pattern, StartSpec};
