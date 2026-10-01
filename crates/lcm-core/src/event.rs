//! Event queue ordered like Python's `heapq` of `(time, id, state)` tuples.

use core::cmp::Ordering;
use std::collections::BinaryHeap;

/// Declared in Python string order (`"CRASH" < "LOOK" < "VISUALIZE" < "WAIT"`),
/// which is how ties on `(time, id)` break.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum EventKind {
    Crash = 0,
    Look = 1,
    Visualize = 3,
    Wait = 4,
}

#[derive(Clone, Copy, Debug)]
pub struct Event {
    pub time: f64,
    /// `-1` for visualize events.
    pub robot: i32,
    pub kind: EventKind,
}

impl Event {
    fn key_cmp(&self, other: &Self) -> Ordering {
        self.time
            .total_cmp(&other.time)
            .then(self.robot.cmp(&other.robot))
            .then(self.kind.cmp(&other.kind))
    }
}

impl PartialEq for Event {
    fn eq(&self, other: &Self) -> bool {
        self.key_cmp(other) == Ordering::Equal
    }
}

impl Eq for Event {}

impl PartialOrd for Event {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Reversed so `BinaryHeap` (a max-heap) pops the earliest event.
impl Ord for Event {
    fn cmp(&self, other: &Self) -> Ordering {
        other.key_cmp(self)
    }
}

#[derive(Clone, Debug, Default)]
pub struct EventQueue {
    heap: BinaryHeap<Event>,
}

impl EventQueue {
    pub fn push(&mut self, event: Event) {
        self.heap.push(event);
    }

    pub fn pop(&mut self) -> Option<Event> {
        self.heap.pop()
    }

    #[must_use]
    pub fn peek(&self) -> Option<&Event> {
        self.heap.peek()
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.heap.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.heap.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pops_in_python_tuple_order() {
        let mut q = EventQueue::default();
        let e = |time, robot, kind| Event { time, robot, kind };
        q.push(e(1.0, 2, EventKind::Look));
        q.push(e(0.5, 7, EventKind::Wait));
        q.push(e(1.0, -1, EventKind::Visualize));
        q.push(e(1.0, 2, EventKind::Crash));
        let order: Vec<_> = std::iter::from_fn(|| q.pop())
            .map(|e| (e.time, e.robot, e.kind))
            .collect();
        assert_eq!(
            order,
            vec![
                (0.5, 7, EventKind::Wait),
                (1.0, -1, EventKind::Visualize),
                (1.0, 2, EventKind::Crash),
                (1.0, 2, EventKind::Look),
            ]
        );
    }
}
