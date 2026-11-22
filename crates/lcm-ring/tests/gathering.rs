//! Theorem V.1 of Navarra and Piselli: from every start outside the
//! unsolvable set, GatheRRing gathers the robots in at most n - 3 epochs,
//! whatever the round-robin order.

use lcm_core::Rng;
use lcm_ring::{unsolvable, RingConfig, RingSim, Status, Ties};

/// A random start: `k` robots on a ring of `n`, all on different nodes or not.
fn start(rng: &mut Rng, n: usize, k: usize, distinct: bool) -> Vec<usize> {
    if distinct {
        let mut nodes: Vec<usize> = (0..n).collect();
        rng.shuffle(&mut nodes);
        nodes.truncate(k);
        nodes
    } else {
        (0..k).map(|_| rng.index(n)).collect()
    }
}

fn check(distinct: bool) -> (usize, Vec<String>) {
    let mut rng = Rng::new(2025);
    let mut tried = 0;
    let mut failures = Vec::new();
    for n in 6..=16 {
        for k in 3..=n.min(10) {
            for _ in 0..30 {
                let robots = start(&mut rng, n, k, distinct);
                let mut order: Vec<usize> = (0..k).collect();
                rng.shuffle(&mut order);
                let ties = if rng.random() < 0.5 {
                    Ties::Next
                } else {
                    Ties::Random
                };
                let config = RingConfig {
                    nodes: n,
                    robots,
                    order: Some(order),
                    ties,
                    seed: rng.next_u64(),
                    max_epochs: Some(10 * n as u64),
                };
                if unsolvable(&config).is_some() {
                    continue;
                }
                tried += 1;
                let mut sim = RingSim::new(&config).unwrap();
                sim.run();
                let bound = (n as u64).saturating_sub(3);
                if sim.status != Status::Gathered || sim.epochs > bound {
                    failures.push(format!(
                        "n {n} k {k} start {:?} order {:?} ties {:?}: {:?} after {} epochs (bound {bound})",
                        config.robots, config.order, config.ties, sim.status, sim.epochs
                    ));
                }
            }
        }
    }
    (tried, failures)
}

#[test]
fn distinct_starts_gather_within_n_minus_3_epochs() {
    let (tried, failures) = check(true);
    assert!(tried > 500);
    assert!(
        failures.is_empty(),
        "{} of {tried} failed:\n{}",
        failures.len(),
        failures[..failures.len().min(10)].join("\n")
    );
}

#[test]
fn starts_with_multiplicities_gather_within_n_minus_3_epochs() {
    let (tried, failures) = check(false);
    assert!(tried > 500);
    assert!(
        failures.is_empty(),
        "{} of {tried} failed:\n{}",
        failures.len(),
        failures[..failures.len().min(10)].join("\n")
    );
}
