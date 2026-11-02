//! Points several robots share (multiplicities), as robots perceive them.

use crate::config::MultiplicityModel;
use crate::geom::{dist, Point};

/// One occupied point as a robot perceives it under a [`MultiplicityModel`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Occupied {
    pub at: Point,
    /// More than one robot is here. Always `false` under `None`; under
    /// `Local`, known only for the robot's own point (`false` elsewhere).
    pub multiple: bool,
    /// How many robots are here: `Some` only under `Strong`.
    pub count: Option<usize>,
}

/// The occupied points of `positions` as seen under `model`, and the index of
/// the point robot `me` stands on. Points closer than `tolerance` are one.
#[must_use]
pub fn perceive(
    positions: &[Point],
    me: usize,
    tolerance: f64,
    model: MultiplicityModel,
) -> (Vec<Occupied>, usize) {
    let (groups, counts, owner) = occupancy(positions, tolerance);
    let mine = owner.get(me).copied().unwrap_or(0);
    let points = groups
        .into_iter()
        .zip(counts)
        .enumerate()
        .map(|(g, (at, c))| Occupied {
            at,
            multiple: c >= 2
                && match model {
                    MultiplicityModel::None => false,
                    MultiplicityModel::Local => g == mine,
                    MultiplicityModel::Weak | MultiplicityModel::Strong => true,
                },
            count: (model == MultiplicityModel::Strong).then_some(c),
        })
        .collect();
    (points, mine)
}

/// Distinct locations of `positions` (first-seen order) and how many robots
/// stand on each. Robot `i` stands on `groups[owner[i]]`.
#[must_use]
#[allow(clippy::cast_possible_truncation)]
pub fn occupancy(positions: &[Point], tolerance: f64) -> (Vec<Point>, Vec<usize>, Vec<usize>) {
    // Groups are bucketed in cells of side `tolerance`, so a point only has to
    // be compared with the groups in its own and the 8 neighbouring cells.
    // Taking the lowest matching index keeps the first-seen grouping of a
    // plain linear scan, in O(n) instead of O(n²).
    let cell = |v: f64| libm::floor(v / tolerance) as i64;
    let mut buckets: std::collections::HashMap<(i64, i64), Vec<usize>> =
        std::collections::HashMap::new();
    let mut groups: Vec<Point> = Vec::new();
    let mut counts: Vec<usize> = Vec::new();
    let mut owner = Vec::with_capacity(positions.len());
    for &p in positions {
        let (cx, cy) = (cell(p.x), cell(p.y));
        let mut found: Option<usize> = None;
        for dx in -1..=1_i64 {
            for dy in -1..=1_i64 {
                let key = (cx.saturating_add(dx), cy.saturating_add(dy));
                for &g in buckets.get(&key).map_or(&[][..], Vec::as_slice) {
                    if dist(p, groups[g]) <= tolerance && found.is_none_or(|f| g < f) {
                        found = Some(g);
                    }
                }
            }
        }
        if let Some(g) = found {
            counts[g] += 1;
            owner.push(g);
        } else {
            buckets.entry((cx, cy)).or_default().push(groups.len());
            owner.push(groups.len());
            groups.push(p);
            counts.push(1);
        }
    }
    (groups, counts, owner)
}
