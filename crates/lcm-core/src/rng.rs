//! The simulation's only source of randomness (docs/core-schema.md §4).
//!
//! `reference/portable_rng.py` implements the same streams in Python; any
//! change here must be made there too, or the parity fixtures stop matching.

const TWO_POW_M53: f64 = 1.0 / 9_007_199_254_740_992.0;

/// SplitMix64 with the `numpy.random.Generator` methods the simulator uses.
#[derive(Clone, Debug)]
pub struct Rng {
    state: u64,
}

impl Rng {
    #[must_use]
    pub const fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in [0, 1) with 53 random bits.
    #[allow(clippy::cast_precision_loss)]
    pub fn random(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * TWO_POW_M53
    }

    pub fn uniform(&mut self, low: f64, high: f64) -> f64 {
        low + (high - low) * self.random()
    }

    pub fn exponential(&mut self, scale: f64) -> f64 {
        -scale * libm::log1p(-self.random())
    }

    /// Uniform integer in `0..n` without modulo bias. `n` must be non-zero.
    #[allow(clippy::cast_possible_truncation)]
    pub fn index(&mut self, n: usize) -> usize {
        debug_assert!(n > 0);
        let n = n as u64;
        let threshold = n.wrapping_neg() % n;
        loop {
            let v = self.next_u64();
            if v >= threshold {
                return (v % n) as usize;
            }
        }
    }

    /// Fisher–Yates from the end, like `Generator.shuffle` on a list.
    pub fn shuffle<T>(&mut self, xs: &mut [T]) {
        for i in (1..xs.len()).rev() {
            let j = self.index(i + 1);
            xs.swap(i, j);
        }
    }

    /// `k` distinct values from `0..n`, in draw order.
    pub fn choice(&mut self, n: usize, k: usize) -> Vec<usize> {
        let mut pool: Vec<usize> = (0..n).collect();
        let k = k.min(n);
        for i in 0..k {
            let j = i + self.index(n - i);
            pool.swap(i, j);
        }
        pool.truncate(k);
        pool
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_reference_splitmix64() {
        // First outputs of SplitMix64 seeded with 0 (Vigna's reference implementation).
        let mut rng = Rng::new(0);
        assert_eq!(rng.next_u64(), 0xE220_A839_7B1D_CDAF);
        assert_eq!(rng.next_u64(), 0x6E78_9E6A_A1B9_65F4);
    }

    #[test]
    fn index_and_choice_stay_in_range() {
        let mut rng = Rng::new(9);
        for n in 1..50 {
            assert!(rng.index(n) < n);
        }
        let picked = rng.choice(10, 4);
        assert_eq!(picked.len(), 4);
        let mut sorted = picked.clone();
        sorted.sort_unstable();
        sorted.dedup();
        assert_eq!(sorted.len(), 4);
    }
}
