//! Robots on a ring of nodes under round robin, and Algorithm GatheRRing:
//! Navarra and Piselli, "Oblivious robots under round robin: gathering on
//! rings" (arXiv:2502.03939v2, Sections II-V, Tables I-III).
//!
//! **Model.** An anonymous ring of `n` nodes without orientation. Robots are
//! anonymous, oblivious, disoriented and silent, and have **no multiplicity
//! detection at all**: a robot sees only which nodes are occupied. A move goes
//! to a neighbouring node (or nowhere) and is instantaneous. The round-robin
//! scheduler activates the robots one at a time in a fixed order the robots do
//! not know, and repeats it: every robot acts exactly once per epoch.
//!
//! **Algorithm.** Tasks T8 down to T1 are checked in that order and the first
//! whose precondition holds gives the move (Table II, Eq. 1; Table III):
//!
//! | task | precondition | move |
//! |---|---|---|
//! | T8 | o1: one occupied node | none (gathered) |
//! | T7 | o2: two neighbouring nodes | m7: to the occupied neighbour |
//! | T6 | o3: three consecutive nodes | m6: with an empty neighbour, to the occupied one |
//! | T5 | p: two nodes one apart | m5: into the empty node between them |
//! | T4 | h: two holes, of 1 and of more than 1 | m4: next to the biggest hole and a robot, to the robot |
//! | T3 | b5: one hole of at least 5 | m3: to an empty neighbour |
//! | T2 | b4 or f: one hole of at most 4, or none | m2 (see [`decide`]) |
//! | T1 | otherwise | m1 (see [`decide`]) |
//!
//! Theorem V.1: from every start outside the unsolvable set ([`unsolvable`])
//! the robots gather in at most n - 3 epochs.
//!
//! **Choices the paper leaves to the adversary** are made explicit by
//! [`Ties`]: moving "in any direction" (m2 with no hole), a robot exactly
//! between two equal choices, and, in m1, a robot between two biggest holes
//! ("it moves toward the hole", proof of Lemma V.1, H2).

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// A step: towards the next node (+1), the previous one (-1), or none.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Move {
    Stay,
    Step(i8),
    /// Either way: the adversary chooses.
    Either,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Task {
    T1,
    T2,
    T3,
    T4,
    T5,
    T6,
    T7,
    T8,
}

/// How the adversary settles a choice the algorithm leaves open.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum Ties {
    /// Always towards the next node (index + 1).
    #[default]
    Next,
    /// At random, from the run's seed.
    Random,
}

#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(default, deny_unknown_fields))]
pub struct RingConfig {
    /// Nodes on the ring (at least 3).
    pub nodes: usize,
    /// The node each robot starts on (repeats make multiplicities).
    pub robots: Vec<usize>,
    /// The round-robin order (a permutation of the robots). `None`: 0, 1, 2, …
    pub order: Option<Vec<usize>>,
    pub ties: Ties,
    pub seed: u64,
    /// Stop after this many epochs. `None`: 4n.
    pub max_epochs: Option<u64>,
}

impl Default for RingConfig {
    fn default() -> Self {
        Self {
            nodes: 10,
            robots: vec![0, 2, 3, 7],
            order: None,
            ties: Ties::Next,
            seed: 1,
            max_epochs: None,
        }
    }
}

impl RingConfig {
    /// # Errors
    /// Fewer than 3 nodes, no robots, a robot off the ring, or an order that
    /// is not a permutation of the robots.
    pub fn validate(&self) -> Result<(), String> {
        if self.nodes < 3 {
            return Err(format!("a ring needs at least 3 nodes, got {}", self.nodes));
        }
        if self.robots.is_empty() {
            return Err("place at least one robot".to_owned());
        }
        if let Some(&v) = self.robots.iter().find(|&&v| v >= self.nodes) {
            return Err(format!(
                "robot on node {v}, but the ring has nodes 0..{}",
                self.nodes - 1
            ));
        }
        if let Some(order) = &self.order {
            let mut seen = vec![false; self.robots.len()];
            for &r in order {
                if r >= seen.len() || seen[r] {
                    return Err("order must name every robot exactly once".to_owned());
                }
                seen[r] = true;
            }
            if seen.iter().any(|s| !s) {
                return Err("order must name every robot exactly once".to_owned());
            }
        }
        Ok(())
    }
}

/// What every robot sees: which nodes are occupied, and the holes (maximal
/// runs of empty nodes) and islands (maximal runs of occupied nodes).
#[derive(Clone, Debug)]
pub struct View {
    pub occupied: Vec<bool>,
    /// `(first node, length)`, going up from an occupied node.
    pub holes: Vec<(usize, usize)>,
    pub islands: Vec<(usize, usize)>,
    /// For an empty node, the index of its hole.
    pub hole_of: Vec<Option<usize>>,
}

fn runs(occ: &[bool], want: bool) -> Vec<(usize, usize)> {
    let n = occ.len();
    let Some(start) = (0..n).find(|&v| occ[v] != want) else {
        return if want { vec![(0, n)] } else { Vec::new() };
    };
    let mut out = Vec::new();
    let mut k = 0;
    while k < n {
        let v = (start + k) % n;
        if occ[v] == want {
            let first = v;
            let mut len = 0;
            while k < n && occ[(start + k) % n] == want {
                len += 1;
                k += 1;
            }
            out.push((first, len));
        } else {
            k += 1;
        }
    }
    out
}

impl View {
    #[must_use]
    pub fn new(occupied: Vec<bool>) -> Self {
        let holes = runs(&occupied, false);
        let islands = runs(&occupied, true);
        let n = occupied.len();
        let mut hole_of = vec![None; n];
        for (h, &(first, len)) in holes.iter().enumerate() {
            for k in 0..len {
                hole_of[(first + k) % n] = Some(h);
            }
        }
        Self {
            occupied,
            holes,
            islands,
            hole_of,
        }
    }

    fn n(&self) -> usize {
        self.occupied.len()
    }

    fn at(&self, v: usize, d: i8) -> usize {
        let n = self.n();
        (v + n + usize::from(d == 1) + (n - 1) * usize::from(d == -1)) % n
    }

    fn count(&self) -> usize {
        self.occupied.iter().filter(|o| **o).count()
    }

    /// The task whose precondition holds (T8 first, then down to T1).
    #[must_use]
    pub fn task(&self) -> Task {
        let n = self.n();
        let m = self.count();
        let holes = &self.holes;
        let o1 = m == 1;
        let o2 = m == 2 && self.islands.len() == 1;
        let o3 = m == 3 && self.islands.len() == 1;
        let p = m == 2
            && (0..n).any(|x| {
                !self.occupied[x] && self.occupied[self.at(x, 1)] && self.occupied[self.at(x, -1)]
            });
        let h = holes.len() == 2
            && ((holes[0].1 == 1 && holes[1].1 > 1) || (holes[1].1 == 1 && holes[0].1 > 1));
        let b5 = holes.len() == 1 && holes[0].1 >= 5;
        let b4 = holes.len() == 1 && holes[0].1 <= 4;
        let f = holes.is_empty();
        if o1 {
            Task::T8
        } else if o2 {
            Task::T7
        } else if o3 {
            Task::T6
        } else if p {
            Task::T5
        } else if h {
            Task::T4
        } else if b5 {
            Task::T3
        } else if b4 || f {
            Task::T2
        } else {
            Task::T1
        }
    }
}

/// Which rule of Table III gave a move, for traces and the page.
pub type Rule = &'static str;

/// GatheRRing for the robot on node `v`: the task, the move and the rule.
#[must_use]
#[allow(clippy::too_many_lines)]
pub fn decide(view: &View, v: usize) -> (Task, Move, Rule) {
    let n = view.n();
    let occ = &view.occupied;
    let (lo, hi) = (view.at(v, -1), view.at(v, 1));
    let (ol, oh) = (occ[lo], occ[hi]);
    let toward_occupied = || match (ol, oh) {
        (true, false) => Move::Step(-1),
        (false, true) => Move::Step(1),
        (true, true) => Move::Either,
        (false, false) => Move::Stay,
    };
    let toward_empty = || match (ol, oh) {
        (false, true) => Move::Step(-1),
        (true, false) => Move::Step(1),
        (false, false) => Move::Either,
        (true, true) => Move::Stay,
    };
    let task = view.task();
    match task {
        Task::T8 => (task, Move::Stay, "o1: gathered"),
        Task::T7 => (task, toward_occupied(), "m7: to the occupied neighbour"),
        Task::T6 => {
            if !ol || !oh {
                (
                    task,
                    toward_occupied(),
                    "m6: at an end, to the occupied neighbour",
                )
            } else {
                (task, Move::Stay, "m6: in the middle, stay")
            }
        }
        Task::T5 => {
            let gap = |x: usize| !occ[x] && occ[view.at(x, 1)] && occ[view.at(x, -1)];
            match (gap(lo), gap(hi)) {
                (true, false) => (task, Move::Step(-1), "m5: into the gap"),
                (false, true) => (task, Move::Step(1), "m5: into the gap"),
                (true, true) => (task, Move::Either, "m5: into a gap"),
                (false, false) => (task, Move::Stay, "m5: not next to the gap"),
            }
        }
        Task::T4 => {
            let big = if view.holes[0].1 > 1 { 0 } else { 1 };
            let in_big = |x: usize| view.hole_of[x] == Some(big);
            if (in_big(lo) && oh) || (in_big(hi) && ol) {
                (
                    task,
                    toward_occupied(),
                    "m4: next to the biggest hole and a robot, to the robot",
                )
            } else {
                (task, Move::Stay, "m4: stay")
            }
        }
        Task::T3 => {
            if !ol || !oh {
                (task, toward_empty(), "m3: to the empty neighbour")
            } else {
                (task, Move::Stay, "m3: stay")
            }
        }
        Task::T2 => {
            if !(ol && oh) {
                return (task, Move::Stay, "m2: has an empty neighbour, stay");
            }
            if view.holes.is_empty() {
                return (task, Move::Either, "m2: no hole, either way");
            }
            // The (unique) hole's nodes; distance from v to it each way.
            let (first, len) = view.holes[0];
            let dist_up = (1..n).find(|&k| !occ[(v + k) % n]).unwrap_or(n);
            let dist_down = (1..n).find(|&k| !occ[(v + n - k) % n]).unwrap_or(n);
            if n == 6 && len == 1 {
                let _ = first;
                // Away from the hole, unless already on the farthest node.
                if dist_up == dist_down {
                    (
                        task,
                        Move::Stay,
                        "m2 (6-ring): farthest from the hole, stay",
                    )
                } else if dist_up < dist_down {
                    (task, Move::Step(-1), "m2 (6-ring): away from the hole")
                } else {
                    (task, Move::Step(1), "m2 (6-ring): away from the hole")
                }
            } else if dist_up < dist_down {
                (task, Move::Step(1), "m2: toward the hole")
            } else if dist_down < dist_up {
                (task, Move::Step(-1), "m2: toward the hole")
            } else {
                (task, Move::Either, "m2: toward the hole, either way")
            }
        }
        Task::T1 => {
            let biggest = view.holes.iter().map(|h| h.1).max().unwrap_or(0);
            let is_big = |x: usize| view.hole_of[x].is_some_and(|h| view.holes[h].1 == biggest);
            let (big_lo, big_hi) = (!ol && is_big(lo), !oh && is_big(hi));
            if !(big_lo || big_hi) {
                return (task, Move::Stay, "m1: not next to a biggest hole, stay");
            }
            let unique = view.holes.iter().filter(|h| h.1 == biggest).count() == 1;
            let m = view.count();
            if view.islands.iter().all(|i| i.1 == 2) {
                if unique {
                    (
                        task,
                        toward_occupied(),
                        "m1: islands of 2, unique biggest hole: to the occupied neighbour",
                    )
                } else {
                    (
                        task,
                        toward_empty(),
                        "m1: islands of 2, several biggest holes: to the closest empty node",
                    )
                }
            } else if m + 2 == n && view.holes.len() == 2 {
                // n - 2 nodes occupied, not consecutive: two holes of size 1.
                if ol != oh {
                    (
                        task,
                        toward_occupied(),
                        "m1: n-2 occupied, one empty neighbour: to the occupied neighbour",
                    )
                } else {
                    (
                        task,
                        Move::Stay,
                        "m1: n-2 occupied, two empty neighbours: stay",
                    )
                }
            } else if big_lo && big_hi {
                // Between two biggest holes: "it moves toward the hole" (H2).
                (
                    task,
                    Move::Either,
                    "m1: between two biggest holes, into one",
                )
            } else if big_lo {
                (task, Move::Step(1), "m1: away from the biggest hole")
            } else {
                (task, Move::Step(-1), "m1: away from the biggest hole")
            }
        }
    }
}

/// Why a start is in the paper's unsolvable set UC (Theorems III.2, III.3,
/// Corollary III.1; for k >= 3 robots), or `None`.
#[must_use]
pub fn unsolvable(config: &RingConfig) -> Option<&'static str> {
    let n = config.nodes;
    let k = config.robots.len();
    let mut count = vec![0usize; n];
    for &v in &config.robots {
        count[v] += 1;
    }
    let occupied: Vec<bool> = count.iter().map(|&c| c > 0).collect();
    let view = View::new(occupied.clone());
    let m = view.count();
    let distinct = count.iter().all(|&c| c <= 1);
    if k < 3 {
        return None;
    }
    if distinct {
        if k == 3 && m == 3 && view.islands.len() == 1 {
            return Some(
                "three robots on three consecutive nodes (Distinct Gathering is impossible)",
            );
        }
        if (n == 4 || n == 5) && m == n {
            return Some("a fully occupied 4- or 5-ring (Distinct Gathering is impossible)");
        }
        return None;
    }
    if m == 2 && view.islands.len() == 1 {
        return Some("exactly two neighbouring nodes occupied");
    }
    if m == 3 && view.islands.len() == 1 {
        return Some("exactly three consecutive nodes occupied");
    }
    if m == n {
        return Some("every node occupied");
    }
    if n == 5 && k >= 5 && m == 3 {
        let next_to_occupied =
            (0..n).any(|v| count[v] >= 2 && (occupied[(v + 1) % n] || occupied[(v + n - 1) % n]));
        if next_to_occupied {
            return Some(
                "a 5-ring on three nodes with a multiplicity next to another occupied node",
            );
        }
    }
    if n == 5 && k >= 5 && m == 4 {
        return Some("a 5-ring with four nodes occupied");
    }
    None
}

/// One activation, for traces and the page (written, never read back).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize))]
pub struct Turn {
    pub turn: u64,
    pub epoch: u64,
    pub robot: usize,
    pub from: usize,
    pub to: usize,
    pub task: Task,
    pub rule: Rule,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum Status {
    Running,
    Gathered,
    /// A whole epoch without any robot moving: nothing will change.
    Stalled,
    /// The epoch limit was reached first.
    Incomplete,
}

/// A run of GatheRRing under round robin.
pub struct RingSim {
    pub config: RingConfig,
    pub positions: Vec<usize>,
    order: Vec<usize>,
    next: usize,
    pub turns: u64,
    pub epochs: u64,
    moved_this_epoch: bool,
    rng: lcm_core::Rng,
    pub status: Status,
    max_epochs: u64,
}

impl RingSim {
    /// # Errors
    /// An invalid configuration.
    pub fn new(config: &RingConfig) -> Result<Self, String> {
        config.validate()?;
        let order = config
            .order
            .clone()
            .unwrap_or_else(|| (0..config.robots.len()).collect());
        let mut sim = Self {
            config: config.clone(),
            positions: config.robots.clone(),
            order,
            next: 0,
            turns: 0,
            epochs: 0,
            moved_this_epoch: false,
            rng: lcm_core::Rng::new(config.seed),
            status: Status::Running,
            max_epochs: config.max_epochs.unwrap_or(4 * config.nodes as u64),
        };
        if sim.view().task() == Task::T8 {
            sim.status = Status::Gathered;
        }
        Ok(sim)
    }

    #[must_use]
    pub fn view(&self) -> View {
        let mut occ = vec![false; self.config.nodes];
        for &v in &self.positions {
            occ[v] = true;
        }
        View::new(occ)
    }

    /// The robot that acts next.
    #[must_use]
    pub fn next_robot(&self) -> usize {
        self.order[self.next]
    }

    /// One activation; `None` once the run is over.
    pub fn step(&mut self) -> Option<Turn> {
        if self.status != Status::Running {
            return None;
        }
        let robot = self.order[self.next];
        let from = self.positions[robot];
        let view = self.view();
        let (task, mv, rule) = decide(&view, from);
        let d = match mv {
            Move::Stay => 0,
            Move::Step(d) => d,
            Move::Either => match self.config.ties {
                Ties::Next => 1,
                Ties::Random => {
                    if self.rng.random() < 0.5 {
                        1
                    } else {
                        -1
                    }
                }
            },
        };
        let to = view.at(from, d);
        let to = if d == 0 { from } else { to };
        self.positions[robot] = to;
        self.moved_this_epoch |= to != from;
        self.turns += 1;
        let turn = Turn {
            turn: self.turns,
            epoch: self.epochs + 1,
            robot,
            from,
            to,
            task,
            rule,
        };
        self.next += 1;
        if self.next == self.order.len() {
            self.next = 0;
            self.epochs += 1;
            if !self.moved_this_epoch && self.view().task() != Task::T8 {
                self.status = Status::Stalled;
            }
            self.moved_this_epoch = false;
        }
        if self.view().task() == Task::T8 {
            self.status = Status::Gathered;
        } else if self.status == Status::Running && self.epochs >= self.max_epochs {
            self.status = Status::Incomplete;
        }
        Some(turn)
    }

    /// Plays to the end; returns every turn.
    pub fn run(&mut self) -> Vec<Turn> {
        let mut turns = Vec::new();
        while let Some(t) = self.step() {
            turns.push(t);
        }
        turns
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn view(n: usize, occupied: &[usize]) -> View {
        let mut occ = vec![false; n];
        for &v in occupied {
            occ[v] = true;
        }
        View::new(occ)
    }

    #[test]
    fn holes_and_islands_wrap_around() {
        let v = view(10, &[0, 1, 5, 9]);
        let mut holes: Vec<usize> = v.holes.iter().map(|h| h.1).collect();
        holes.sort_unstable();
        assert_eq!(holes, vec![3, 3]);
        let mut islands: Vec<usize> = v.islands.iter().map(|i| i.1).collect();
        islands.sort_unstable();
        assert_eq!(islands, vec![1, 3]);
    }

    #[test]
    fn tasks_follow_the_preconditions() {
        assert_eq!(view(8, &[3]).task(), Task::T8);
        assert_eq!(view(8, &[3, 4]).task(), Task::T7);
        assert_eq!(view(8, &[3, 4, 5]).task(), Task::T6);
        assert_eq!(view(8, &[3, 5]).task(), Task::T5);
        assert_eq!(view(8, &[0, 1, 2, 4]).task(), Task::T4); // holes of 1 and 3
        assert_eq!(view(10, &[0, 1, 2, 3]).task(), Task::T3); // one hole of 6
        assert_eq!(view(8, &[0, 1, 2, 3, 4, 5]).task(), Task::T2); // one hole of 2
        assert_eq!(view(5, &[0, 1, 2, 3, 4]).task(), Task::T2); // full
        assert_eq!(view(12, &[0, 1, 4, 5, 9]).task(), Task::T1);
    }

    #[test]
    fn m1_moves_away_from_the_biggest_hole() {
        // Holes: 2-3 (2), 6-8 (3), 10-11 (2). Robot at 5 borders the biggest.
        let v = view(12, &[0, 1, 4, 5, 9]);
        assert_eq!(decide(&v, 5).1, Move::Step(-1));
        assert_eq!(decide(&v, 9).1, Move::Step(1));
        assert_eq!(decide(&v, 0).1, Move::Stay);
    }

    #[test]
    fn the_unsolvable_set() {
        let c = |nodes: usize, robots: Vec<usize>| RingConfig {
            nodes,
            robots,
            ..RingConfig::default()
        };
        assert!(unsolvable(&c(9, vec![2, 3, 4])).is_some());
        assert!(unsolvable(&c(5, vec![0, 1, 2, 3, 4])).is_some());
        assert!(unsolvable(&c(9, vec![2, 2, 3])).is_some());
        assert!(unsolvable(&c(9, vec![0, 2, 5])).is_none());
    }
}
