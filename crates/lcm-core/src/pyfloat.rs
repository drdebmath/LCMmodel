//! Float operations with the exact results of their CPython counterparts.
//!
//! The reference simulator is Python; where CPython's arithmetic differs from
//! the obvious Rust expression, the result of an algorithm can differ too.

/// `sum(xs)` for floats. CPython 3.12+ uses Neumaier-compensated summation,
/// so a plain fold would round differently.
pub fn py_sum(xs: impl IntoIterator<Item = f64>) -> f64 {
    let mut total = 0.0_f64;
    let mut compensation = 0.0_f64;
    for x in xs {
        let t = total + x;
        if total.abs() >= x.abs() {
            compensation += (total - t) + x;
        } else {
            compensation += (x - t) + total;
        }
        total = t;
    }
    if compensation != 0.0 && compensation.is_finite() {
        total += compensation;
    }
    total
}

/// `x % m` for floats: the result takes the sign of the divisor.
#[must_use]
pub fn py_mod(x: f64, m: f64) -> f64 {
    let r = x % m;
    if r == 0.0 {
        0.0_f64.copysign(m)
    } else if (m < 0.0) != (r < 0.0) {
        r + m
    } else {
        r
    }
}

const POW10: [f64; 23] = [
    1e0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9, 1e10, 1e11, 1e12, 1e13, 1e14, 1e15, 1e16,
    1e17, 1e18, 1e19, 1e20, 1e21, 1e22,
];
const POW10_NEG: [f64; 23] = [
    1e0, 1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8, 1e-9, 1e-10, 1e-11, 1e-12, 1e-13, 1e-14,
    1e-15, 1e-16, 1e-17, 1e-18, 1e-19, 1e-20, 1e-21, 1e-22,
];

/// `10 ** -p` / `math.pow(10, -p)`: the correctly rounded value of 10⁻ᵖ.
#[must_use]
pub fn pow10_neg(p: u8) -> f64 {
    POW10_NEG[usize::from(p).min(22)]
}

/// `round(x, p)`: round the exact decimal value of `x` to `p` places, ties to
/// even, and return the nearest double to that decimal.
///
/// Fast path: when `x·10ᵖ` is not near a half-way point, the integer it rounds
/// to is the same as for the exact product, and dividing two exactly
/// representable integers is correctly rounded. Otherwise fall back to exact
/// decimal formatting, which is what CPython does.
#[must_use]
pub fn py_round(x: f64, p: u8) -> f64 {
    if !x.is_finite() {
        return x;
    }
    let p = usize::from(p).min(22);
    let scale = POW10[p];
    let y = x * scale;
    if y.abs() < 4.5e15 {
        let floor = y.floor();
        let frac = y - floor;
        let guard = (y.abs() * f64::EPSILON * 4.0).max(f64::MIN_POSITIVE);
        if (frac - 0.5).abs() > guard {
            let k = if frac > 0.5 { floor + 1.0 } else { floor };
            let r = k / scale;
            return if r == 0.0 { 0.0_f64.copysign(x) } else { r };
        }
    }
    format!("{x:.p$}").parse().unwrap_or(x)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sum_is_compensated_like_cpython() {
        // Python 3.12: sum([0.1] * 10) == 1.0, while a plain fold gives 0.9999999999999999
        assert_eq!(py_sum(std::iter::repeat_n(0.1, 10)), 1.0);
        assert_eq!(py_sum([1e100, 1.0, -1e100, 1.0]), 2.0);
        assert_eq!(py_sum([]), 0.0);
    }

    #[test]
    fn modulo_follows_divisor_sign() {
        assert_eq!(py_mod(-1.0, 3.0), 2.0);
        assert_eq!(py_mod(1.0, -3.0), -2.0);
        assert_eq!(py_mod(-0.0, 3.0).to_bits(), 0.0_f64.to_bits());
        let two_pi = 2.0 * std::f64::consts::PI;
        assert_eq!(
            py_mod(-std::f64::consts::FRAC_PI_2, two_pi),
            3.0 * std::f64::consts::FRAC_PI_2
        );
    }

    #[test]
    fn round_matches_cpython() {
        // Values checked against CPython 3.12.
        assert_eq!(py_round(2.675, 2), 2.67); // 2.675 is really 2.67499999…
        assert_eq!(py_round(0.125, 2), 0.12); // exact tie -> even
        assert_eq!(py_round(0.375, 2), 0.38);
        assert_eq!(py_round(1.000_004_999, 5), 1.0);
        assert_eq!(py_round(-1.234_565, 5), -1.23456);
        assert_eq!(py_round(123.456_789, 3), 123.457);
    }
}
