//! `py_round` against 5,000 results of CPython 3.12 `round(x, p)`
//! (fixtures/py_round_cases.txt: `x.hex() p round(x, p).hex()`).

use lcm_core::pyfloat::py_round;

fn parse_hex(s: &str) -> f64 {
    let (neg, s) = s.strip_prefix('-').map_or((false, s), |r| (true, r));
    let s = s.strip_prefix("0x").expect("hex float");
    let (mantissa, exp) = s.split_once('p').expect("exponent");
    let (int, frac) = mantissa.split_once('.').unwrap_or((mantissa, ""));
    let mut value = u64::from_str_radix(int, 16).unwrap() as f64;
    let mut scale = 1.0 / 16.0;
    for c in frac.chars() {
        value += f64::from(c.to_digit(16).unwrap()) * scale;
        scale /= 16.0;
    }
    let v = value * 2f64.powi(exp.parse().unwrap());
    if neg {
        -v
    } else {
        v
    }
}

#[test]
fn matches_cpython_round() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../fixtures/py_round_cases.txt"
    );
    let text = std::fs::read_to_string(path).unwrap();
    let mut checked = 0;
    for line in text.lines() {
        let mut parts = line.split_whitespace();
        let x = parse_hex(parts.next().unwrap());
        let p: u8 = parts.next().unwrap().parse().unwrap();
        let expected = parse_hex(parts.next().unwrap());
        let got = py_round(x, p);
        assert!(
            got.to_bits() == expected.to_bits() || got == expected,
            "round({x:e}, {p}) = {got:e}, python {expected:e}"
        );
        checked += 1;
    }
    assert_eq!(checked, 5000);
}
