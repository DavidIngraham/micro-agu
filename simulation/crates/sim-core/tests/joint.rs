use nalgebra::{Rotation3, UnitQuaternion};
use sim_core::{
    joint::joint_loads,
    state::{M, V},
    Parameters,
};
fn rot(v: V) -> M {
    UnitQuaternion::from_scaled_axis(v)
        .to_rotation_matrix()
        .into_inner()
}
fn potential(rp: M, rc: M, m: f64, p: &Parameters) -> f64 {
    let e = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(
        rp.transpose() * rc,
    ))
    .scaled_axis();
    let yaw = if p.geometry_yaw > 0. {
        m * 9.80665
            * (p.yaw_vertical_separation
                - (p.yaw_vertical_separation.powi(2)
                    - 2. * p.yaw_payload_halfspacing * p.yaw_canopy_halfspacing * (1. - e.z.cos()))
                .sqrt())
    } else {
        0.5 * p.joint_k_yaw * e.z * e.z
    };
    0.5 * (p.joint_k_roll * e.x * e.x + p.joint_k_pitch * e.y * e.y) + yaw
}
#[test]
fn elastic_torque_matches_energy_gradient_and_damping_is_passive() {
    for geometry in [0., 1.] {
        let p = Parameters {
            geometry_yaw: geometry,
            ..Parameters::default()
        };
        let rp = rot(V::new(0.2, -0.1, 0.4));
        let rc = rp * rot(V::new(-0.15, 0.12, 0.25));
        let wp = V::new(0.2, -0.4, 0.1);
        let wc = V::new(-0.1, 0.3, -0.2);
        let loads = joint_loads(rp, rc, wp, wc, 1.568, &p).unwrap();
        assert!((-loads.damping.dot(&(wc - wp))) < 0.);
        for i in 0..3 {
            let mut delta = V::zeros();
            delta[i] = 1e-6;
            let derivative = (potential(rp, rot(delta) * rc, 1.568, &p)
                - potential(rp, rot(-delta) * rc, 1.568, &p))
                / 2e-6;
            assert!((derivative - loads.spring[i]).abs() < 1e-8);
        }
        assert!((loads.total - loads.spring - loads.damping).norm() < 1e-12);
    }
}
#[test]
fn geometry_replaces_linear_yaw_and_zero_rate_has_no_damper() {
    let p = Parameters {
        geometry_yaw: 1.,
        joint_k_yaw: 1000.,
        ..Parameters::default()
    };
    let eps = 1e-5;
    let load = joint_loads(
        M::identity(),
        rot(V::new(0., 0., eps)),
        V::zeros(),
        V::zeros(),
        1.568,
        &p,
    )
    .unwrap();
    let expected = 1.568 * 9.80665 * p.yaw_payload_halfspacing * p.yaw_canopy_halfspacing
        / p.yaw_vertical_separation;
    assert!((load.spring.z / eps - expected).abs() < 1e-9);
    assert_eq!(load.damping, V::zeros());
}
