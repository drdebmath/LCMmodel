//! Render data leaving the core (docs/core-schema.md §8). Buffers are owned by
//! the caller and reused, so a browser worker can transfer them each frame.

use crate::algorithm::TaskColor;
use crate::robots::RobotState;
use crate::sim::Simulation;

pub const FLAG_FROZEN: u8 = 1 << 2;
pub const FLAG_TERMINATED: u8 = 1 << 3;
pub const FLAG_HAS_TARGET: u8 = 1 << 4;

#[derive(Clone, Debug, Default)]
pub struct Frame {
    pub time: f64,
    /// World point that `xy`, `target` and the circle centres are measured from:
    /// the centroid of the robots that have not crashed. f32 keeps about seven
    /// digits, so a swarm tiny compared with its distance from (0, 0) would
    /// otherwise round to a single point; offsets from its own centre do not.
    pub origin: [f64; 2],
    /// Offsets from `origin`.
    pub xy: Vec<f32>,
    pub flags: Vec<u8>,
    pub light: Vec<u8>,
    pub fault: Vec<u8>,
    pub task: Vec<u8>,
    /// Offsets from `origin`, NaN where a robot has no target.
    pub target: Vec<f32>,
    /// Centre offset from `origin` and radius, NaN where a robot has no circle.
    pub circle: Vec<f32>,
    /// Robots at the same point (within ε), 1 when alone. Filled only when the
    /// simulation has multiplicity detection on.
    pub multiplicity: Vec<u16>,
}

#[allow(clippy::cast_possible_truncation)]
impl Simulation {
    /// Writes the state at time `t` (normally `self.time()`).
    pub fn write_frame(&self, t: f64, frame: &mut Frame) {
        let r = self.robots();
        let n = r.len();
        frame.time = t;
        frame.xy.resize(2 * n, 0.0);
        frame.flags.resize(n, 0);
        frame.light.resize(n, 0);
        frame.fault.resize(n, 0);
        frame.task.resize(n, 0);
        frame.target.resize(2 * n, f32::NAN);
        frame.circle.resize(3 * n, f32::NAN);
        let [ox, oy] = self.frame_origin(t);
        frame.origin = [ox, oy];
        for i in 0..n {
            let p = self.position_at(i, t);
            frame.xy[2 * i] = (p.x - ox) as f32;
            frame.xy[2 * i + 1] = (p.y - oy) as f32;
            let mut flags = r.state[i] as u8;
            if r.frozen[i] {
                flags |= FLAG_FROZEN;
            }
            if r.terminated[i] {
                flags |= FLAG_TERMINATED;
            }
            if let Some(target) = r.target[i] {
                flags |= FLAG_HAS_TARGET;
                frame.target[2 * i] = (target.x - ox) as f32;
                frame.target[2 * i + 1] = (target.y - oy) as f32;
            } else {
                frame.target[2 * i] = f32::NAN;
                frame.target[2 * i + 1] = f32::NAN;
            }
            frame.flags[i] = flags;
            frame.light[i] = r.light[i].map_or(0, |l| l as u8);
            frame.fault[i] = r.fault[i] as u8;
            frame.task[i] = match r.task[i] {
                None => 0,
                Some(TaskColor::Red) => 1,
                Some(TaskColor::Blue) => 2,
            };
            let c = r.overlay[i];
            frame.circle[3 * i] = c.map_or(f32::NAN, |c| (c.center.x - ox) as f32);
            frame.circle[3 * i + 1] = c.map_or(f32::NAN, |c| (c.center.y - oy) as f32);
            frame.circle[3 * i + 2] = c.map_or(f32::NAN, |c| c.radius as f32);
        }
        if self.multiplicity_detection() {
            self.write_multiplicity(t, frame);
        } else {
            frame.multiplicity.clear();
        }
    }

    /// Centroid of the robots that have not crashed (of all, if every one has).
    fn frame_origin(&self, t: f64) -> [f64; 2] {
        let r = self.robots();
        let (mut sum, mut count) = ([0.0, 0.0], 0usize);
        for pass_all in [false, true] {
            for i in 0..r.len() {
                if pass_all || r.state[i] != RobotState::Crash {
                    let p = self.position_at(i, t);
                    sum[0] += p.x;
                    sum[1] += p.y;
                    count += 1;
                }
            }
            if count > 0 {
                break;
            }
        }
        if count == 0 {
            return [0.0, 0.0];
        }
        [sum[0] / count as f64, sum[1] / count as f64]
    }

    /// `Scheduler._detect_multiplicity`: sort by (x, y); each unvisited robot
    /// starts a group of later robots closer than ε to it. Sorting by x lets
    /// the scan stop once x differs by ε, which gives the same groups as the
    /// Python's full O(n²) scan.
    fn write_multiplicity(&self, t: f64, frame: &mut Frame) {
        let n = self.robots().len();
        let eps = self.model().eps;
        let pts: Vec<_> = (0..n).map(|i| self.position_at(i, t)).collect();
        let mut order: Vec<usize> = (0..n).collect();
        order.sort_by(|&a, &b| {
            pts[a]
                .x
                .total_cmp(&pts[b].x)
                .then(pts[a].y.total_cmp(&pts[b].y))
        });
        let mut visited = vec![false; n];
        frame.multiplicity.clear();
        frame.multiplicity.resize(n, 1);
        let mut group = Vec::new();
        for s in 0..n {
            if visited[s] {
                continue;
            }
            visited[s] = true;
            group.clear();
            group.push(order[s]);
            let anchor = pts[order[s]];
            for (u, &j) in order.iter().enumerate().skip(s + 1) {
                if pts[j].x - anchor.x >= eps {
                    break;
                }
                if !visited[u] && crate::geom::dist(anchor, pts[j]) < eps {
                    visited[u] = true;
                    group.push(j);
                }
            }
            let count = u16::try_from(group.len()).unwrap_or(u16::MAX);
            for &j in &group {
                frame.multiplicity[j] = count;
            }
        }
    }
}

/// Unpacks the state bits of a frame flag byte.
#[must_use]
pub fn flag_state(flags: u8) -> RobotState {
    match flags & 0b11 {
        0 => RobotState::Wait,
        1 => RobotState::Look,
        2 => RobotState::Move,
        _ => RobotState::Crash,
    }
}
