//! MIL-F-8785C Dryden translational gust filters, SI units.
//! Longitudinal H=sqrt(2a)/(s+a); lateral/vertical
//! H=sqrt(a)*(sqrt(3)s+a)/(s+a)^2, with a=V/L.
//! Independent unit white noises; exact discrete state/noise covariance.
//! Outputs are along-wind, cross-wind, down. No angular gusts or terrain model.
use crate::Result;

pub struct Dryden {
    seed: u32,
    state: [[f64; 2]; 3],
}

impl Dryden {
    pub fn new(seed: u32) -> Result<Self> {
        if seed == 0 {
            return Err("Dryden seed must be nonzero".into());
        }
        Ok(Self {
            seed,
            state: [[0.0; 2]; 3],
        })
    }

    fn uniform(&mut self) -> f64 {
        self.seed ^= self.seed << 13;
        self.seed ^= self.seed >> 17;
        self.seed ^= self.seed << 5;
        (self.seed as f64 + 0.5) / 4294967296.0
    }

    fn normal(&mut self) -> f64 {
        (-2.0 * self.uniform().ln()).sqrt() * (std::f64::consts::TAU * self.uniform()).cos()
    }

    /// Scale lengths are metres, intensities are component RMS m/s.
    /// Speed is relative to the mean airmass, with a 0.5 m/s numerical floor.
    /// Configuration changes preserve filter state. dt=0 does not advance RNG.
    pub fn step(
        &mut self,
        dt: f64,
        speed: f64,
        lengths: [f64; 3],
        sigma: [f64; 3],
    ) -> Result<[f64; 3]> {
        if !dt.is_finite()
            || !(0.0..=1.0).contains(&dt)
            || !speed.is_finite()
            || !(0.0..=200.0).contains(&speed)
            || lengths
                .iter()
                .any(|v| !v.is_finite() || !(0.1..=10000.0).contains(v))
            || sigma
                .iter()
                .any(|v| !v.is_finite() || !(0.0..=20.0).contains(v))
        {
            return Err("Invalid Dryden timestep, speed, scale length or intensity".into());
        }
        let mut result = [0.0; 3];
        for axis in 0..3 {
            if dt > 0.0 {
                let q = speed.max(0.5) * dt / lengths[axis];
                let e = (-q).exp();
                let q11 = -(-2.0 * q).exp_m1();
                let (q12, q22) = if q < 0.05 {
                    // Integrals 2*int(t*exp(-2t)) and 2*int(t²*exp(-2t)).
                    // Series avoids cancellation for very small time steps.
                    let mut term = 1.0;
                    let (mut a, mut b) = (0.0, 0.0);
                    for n in 0..12 {
                        a += 2.0 * term * q * q / (n + 2) as f64;
                        b += 2.0 * term * q * q * q / (n + 3) as f64;
                        term *= -2.0 * q / (n + 1) as f64;
                    }
                    (a, b)
                } else {
                    (0.5 * q11 - q * e * e, 0.5 * q11 - (q + q * q) * e * e)
                };
                let n1 = self.normal();
                let n2 = self.normal();
                let l11 = q11.sqrt();
                let l21 = q12 / l11;
                let noise2 = l21 * n1 + (q22 - l21 * l21).max(0.0).sqrt() * n2;
                let old = self.state[axis];
                self.state[axis] = [e * old[0] + l11 * n1, e * (q * old[0] + old[1]) + noise2];
            }
            let [x, y] = self.state[axis];
            result[axis] = sigma[axis]
                * if axis == 0 {
                    x
                } else {
                    (3.0_f64.sqrt() * x + (1.0 - 3.0_f64.sqrt()) * y) / 2.0_f64.sqrt()
                };
        }
        Ok(result)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn deterministic_and_validated() {
        let mut a = Dryden::new(7).unwrap();
        let mut b = Dryden::new(7).unwrap();
        assert!(Dryden::new(0).is_err());
        assert!(a.step(f64::NAN, 4.0, [20.0; 3], [1.0; 3]).is_err());
        for _ in 0..100 {
            assert_eq!(
                a.step(0.02, 4.0, [20.0; 3], [1.0; 3]).unwrap(),
                b.step(0.02, 4.0, [20.0; 3], [1.0; 3]).unwrap()
            );
            a.step(0.0, 4.0, [20.0; 3], [1.0; 3]).unwrap();
        }
    }
    #[test]
    fn stationary_statistics_across_timesteps() {
        for dt in [0.01, 0.05] {
            let mut d = Dryden::new(119).unwrap();
            let lag = (0.5 / dt) as usize;
            let mut history = vec![[0.0; 3]; lag];
            let mut sum = [0.0; 3];
            let mut covariance = [0.0; 3];
            let n = (20000.0 / dt) as usize;
            for i in 0..n + 10000 {
                let v = d.step(dt, 10.0, [10.0; 3], [1.0; 3]).unwrap();
                if i >= 10000 {
                    for j in 0..3 {
                        sum[j] += v[j] * v[j];
                        covariance[j] += v[j] * history[i % lag][j];
                    }
                }
                history[i % lag] = v;
            }
            for j in 0..3 {
                assert!((sum[j] / n as f64 - 1.0).abs() < 0.07);
                let expected = (-0.5_f64).exp() * if j == 0 { 1.0 } else { 0.75 };
                assert!((covariance[j] / sum[j] - expected).abs() < 0.05);
            }
        }
    }
}
