//! Assisted inverted hand launch, in NED. The hand and cloth motion are prescribed;
//! the returned release state hands off to the normal two-body/contact solver.
//! Timing follows the documented technique, not a measured inflation calibration.
use crate::{config::Parameters, state::*};
use nalgebra::UnitQuaternion;
use std::f64::consts::PI;
pub const RELEASE_S: f64 = 5.8;
fn smooth(t: f64) -> f64 {
    let x = t.clamp(0., 1.);
    (x * x * x * (10. + x * (-15. + 6. * x))).clamp(0., 1.)
}
// Integral of the quintic velocity ramp, extended linearly after completion.
// Position, velocity and acceleration agree at both ends of each phase.
fn travel(t: f64, start: f64, duration: f64) -> f64 {
    let x = ((t - start) / duration).clamp(0., 1.);
    duration * (2.5 * x.powi(4) - 3. * x.powi(5) + x.powi(6)) + (t - start - duration).max(0.)
}
fn poses(trim: &State, p: &Parameters, t: f64, height: f64) -> State {
    let mut y = State::zeros();
    let invert = smooth(t / 2.);
    let swing = smooth((t - 2.8) / 1.);
    let angle = PI * invert * (1. - swing);
    let qp0 = UnitQuaternion::from_matrix(&rotation(trim, 0));
    let qc0 = UnitQuaternion::from_matrix(&rotation(trim, 13));
    let qp = UnitQuaternion::from_axis_angle(&V::y_axis(), angle) * qp0;
    let qc = UnitQuaternion::from_axis_angle(&V::y_axis(), PI * (1. - swing)) * qc0;
    // Start walking during the overhead swing; blend the steps into the toss
    // without stopping or introducing an impulse in the synthetic IMU data.
    let x = -0.4 * invert + 0.8 * swing + 1.5 * travel(t, 3.2, 1.6) + 3. * travel(t, 5.0, 0.8);
    let h = 0.10
        + (height - 0.10) * invert
        // The hand rises from inverted setup to overhead release.
        + 0.6 * swing
        + 0.4 * (PI * swing).sin()
        + 0.6 * travel(t, 5.0, 0.8);
    y.fixed_rows_mut::<3>(0).copy_from(&V::new(x, 0., -h));
    y.fixed_rows_mut::<4>(3).copy_from(&qp.coords);
    y.fixed_rows_mut::<4>(16).copy_from(&qc.coords);
    let attached =
        vector(&y, 0) + qp * V::new(p.hinge_x, p.hinge_y, p.hinge_z) - qc * V::new(0., 0., 0.85);
    let laid_out = V::new(-1.6, 0., -0.04);
    let canopy =
        laid_out * (1. - swing) + attached * swing + V::new(-0.35 * (PI * swing).sin(), 0., 0.);
    y.fixed_rows_mut::<3>(13).copy_from(&canopy);
    y
}
/// State, followed by inflation fraction. Velocity and gyro describe the same
/// prescribed motion as position/attitude, including the IMU's lever-arm motion.
pub fn sample(trim: &State, p: &Parameters, t: f64, height: f64) -> crate::Result<Vec<f64>> {
    validate(trim)?;
    if !t.is_finite() || !(0. ..=RELEASE_S).contains(&t) || !height.is_finite() || height < 0.5 {
        return Err("invalid inverted launch time or hand height".into());
    }
    let mut y = poses(trim, p, t, height);
    let eps = 1e-5;
    let a = poses(trim, p, t - eps, height);
    let b = poses(trim, p, t + eps, height);
    for k in [0, 13] {
        let v = (vector(&b, k) - vector(&a, k)) / (2. * eps);
        let qa = UnitQuaternion::from_matrix(&rotation(&a, k));
        let qb = UnitQuaternion::from_matrix(&rotation(&b, k));
        let omega = (qb * qa.inverse()).scaled_axis() / (2. * eps);
        y.fixed_rows_mut::<3>(k + 7).copy_from(&v);
        y.fixed_rows_mut::<3>(k + 10).copy_from(&omega);
    }
    let mut out = y.as_slice().to_vec();
    out.push(smooth((t - 2.8) / 1.));
    Ok(out)
}
