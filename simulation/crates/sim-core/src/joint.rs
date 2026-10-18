//! Effective rotational suspension loads. The positional point constraint is separate.
use crate::{config::Parameters, state::*};
use nalgebra::{Rotation3, UnitQuaternion};
#[derive(Clone, Debug)]
pub struct JointLoads {
    /// World-frame torque on the payload; canopy receives the opposite.
    pub spring: V,
    pub damping: V,
    pub total: V,
}
/// Decompose the same elastic and dissipative loads used by the plant.
/// Returns None when the reduced bifilar construction has no real height.
pub fn joint_loads(rp: M, rc: M, wp: V, wc: V, mp: f64, p: &Parameters) -> Option<JointLoads> {
    let err = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(
        rp.transpose() * rc,
    ))
    .scaled_axis();
    let dw = rp.transpose() * (wc - wp);
    let angle = err.norm();
    let e = skew(err);
    let coefficient = if angle < 1e-4 {
        1. / 12. + angle * angle / 720.
    } else {
        1. / (angle * angle) - (1. + angle.cos()) / (2. * angle * angle.sin())
    };
    let jacobian = M::identity() - 0.5 * e + coefficient * e * e;
    let mut grad = V::new(p.joint_k_roll, p.joint_k_pitch, p.joint_k_yaw).component_mul(&err);
    if p.geometry_yaw > 0. {
        let hs = p.yaw_vertical_separation * p.yaw_vertical_separation
            - 2. * p.yaw_payload_halfspacing * p.yaw_canopy_halfspacing * (1. - err.z.cos());
        if hs <= 0. {
            return None;
        }
        grad.z = mp * 9.80665 * p.yaw_payload_halfspacing * p.yaw_canopy_halfspacing * err.z.sin()
            / hs.sqrt();
    }
    let spring_local = jacobian.transpose() * grad;
    let damping_local = V::new(p.joint_c_roll, p.joint_c_pitch, p.joint_c_yaw).component_mul(&dw);
    let joint = rp * (spring_local + damping_local);
    Some(JointLoads {
        spring: rp * spring_local,
        damping: rp * damping_local,
        total: joint,
    })
}
