//! Curved canopy panels and constrained two-body spatial dynamics.
use crate::config::Parameters;
use crate::state::*;
use nalgebra::Rotation3;

/// Loads at each aerodynamic integration point, in the canopy FRD frame.
/// Diagnostics share the force calculation used by the plant.
#[derive(Clone, Debug)]
pub struct PanelLoad {
    pub position: V,
    pub inclination: f64,
    pub local_velocity: V,
    pub brake: f64,
    pub alpha: f64,
    pub lift: V,
    pub drag: V,
    pub force: V,
}

pub fn panel_loads(v: V, w: V, left: f64, right: f64, p: &Parameters) -> [PanelLoad; 5] {
    let curvature = p.canopy_half_arc;
    let mut positions = [V::zeros(); 5];
    let mut etas = [0.; 5];
    let mut angles = [0.; 5];
    for i in 0..5 {
        etas[i] = (i as f64 + 0.5) / 5. * 2. - 1.;
        angles[i] = etas[i] * curvature;
        positions[i] = if curvature.abs() < 1e-7 {
            V::new(0., etas[i] * p.b / 2., 0.)
        } else {
            V::new(
                0.,
                p.b / (2. * curvature) * angles[i].sin(),
                p.b / (2. * curvature) * (1. - angles[i].cos()),
            )
        };
    }
    let zmean = positions.iter().map(|v| v.z / 5.).sum::<f64>();
    std::array::from_fn(|i| {
        positions[i].z -= zmean;
        let rotation = Rotation3::from_axis_angle(
            &V::x_axis(),
            angles[i] + p.brake_tilt_rad_per_m * 0.072 * (right - left),
        );
        let local = rotation.inverse() * (v + w.cross(&positions[i]));
        let brake = if etas[i] < 0. {
            left
        } else if etas[i] > 0. {
            right
        } else {
            (left + right) / 2.
        };
        let alpha = local.z.atan2(local.x);
        let cl = (p.cl0 + p.cla * alpha + p.clbrake * brake) / (1. + (alpha.abs() / 0.6).powi(4));
        let cd = p.cd0 + p.k_drag * cl * cl + p.cdbrake * brake * brake;
        let f = rotation
            * (0.5 * p.rho * p.s / 5.
                * (local.x.hypot(local.z) * cl * V::new(local.z, 0., -local.x)
                    - local.norm() * cd * local));
        let scale = 0.5 * p.rho * p.s / 5.;
        PanelLoad {
            position: positions[i],
            inclination: angles[i] + p.brake_tilt_rad_per_m * 0.072 * (right - left),
            local_velocity: local,
            brake,
            alpha,
            lift: rotation * (scale * local.x.hypot(local.z) * cl * V::new(local.z, 0., -local.x)),
            drag: rotation * (-scale * local.norm() * cd * local),
            force: f,
        }
    })
}

pub fn panel(v: V, w: V, left: f64, right: f64, p: &Parameters) -> (V, V) {
    let (mut force, mut moment) = (V::zeros(), V::zeros());
    for load in panel_loads(v, w, left, right, p) {
        force += load.force;
        moment += load.position.cross(&load.force);
    }
    (force, moment)
}
pub fn rotor_moment(omega: V, rotation: M, inertia: f64, rpm: f64, rate: f64) -> V {
    let axis = rotation * V::x();
    let factor = 2. * std::f64::consts::PI / 60.;
    -omega.cross(&(axis * (inertia * rpm * factor))) - axis * (inertia * rate * factor)
}
pub fn brake_path(rp: M, rc: M, p: &Parameters, side: f64) -> f64 {
    let anchor = rp * V::new(0., side * p.riser_halfspacing, 0.);
    let guide = rp * V::new(p.guide_x, side * p.guide_halfspacing, 0.);
    let support = rc * V::new(0., side * p.support_halfspacing, -p.support_height);
    let ring = anchor + p.ring_height * (support - anchor).normalize();
    let cascade = support + rc * V::new(p.cascade_x, 0., 0.);
    (guide - ring).norm() + (ring - cascade).norm()
}
pub fn canopy_inertia(p: &Parameters) -> M6 {
    let b1 = constraint(V::new(0., 0., p.added_z1));
    let b2 = constraint(V::new(0., 0., p.added_z2));
    let mut h = b1.transpose() * M::from_diagonal(&V::new(p.added_mx, 0., 0.)) * b1
        + b2.transpose() * M::from_diagonal(&V::new(0., p.added_my, p.added_mz)) * b2;
    for i in 0..3 {
        h[(i, i)] += p.canopy_mass;
    }
    for (i, base) in [0.075, 0.008, 0.077].iter().enumerate() {
        h[(i + 3, i + 3)] += base + [p.added_jx, p.added_jy, p.added_jz][i];
    }
    h
}
pub fn payload_inertia(p: &Parameters) -> M {
    M::new(
        p.ixx, p.ixy, p.ixz, p.ixy, p.iyy, p.iyz, p.ixz, p.iyz, p.izz,
    )
}
#[allow(clippy::too_many_arguments)]
pub fn rhs(
    y: &State,
    input: &[f64; 3],
    p: &Parameters,
    mass: f64,
    wind: V,
    rpm: f64,
    rpm_rate: f64,
    shaft_torque: Option<f64>,
) -> State {
    let rp = rotation(y, PAYLOAD);
    let rc = rotation(y, CANOPY);
    let mut u = *input;
    if p.routing_enabled > 0. {
        for i in 0..2 {
            u[i + 1] = (u[i + 1] + brake_path(rp, rc, p, if i == 0 { -1. } else { 1. })
                - [p.route_reference_left, p.route_reference_right][i])
                .max(0.);
        }
    }
    let mp = mass - p.canopy_mass;
    let mc = p.canopy_mass;
    let arm_p = rp * V::new(p.hinge_x, p.hinge_y, p.hinge_z);
    let arm_c = rc * V::new(0., 0., 0.85);
    let vp = vector(y, 7);
    let vc = vector(y, 20);
    let wp = vector(y, 10);
    let wc = vector(y, 23);
    let ip = rp * payload_inertia(p) * rp.transpose();
    let ipi = ip.try_inverse().unwrap_or_else(|| M::repeat(f64::NAN));
    let va = vp - wind;
    let thrust = rp * V::new(p.thrust_max * u[0], 0., 0.);
    let fp = V::new(0., 0., mp * 9.80665) + thrust - 0.5 * p.rho * p.payload_cda * va.norm() * va;
    let mut moment_p = (rp * V::new(p.thrust_x, p.thrust_y, p.thrust_arm_z)).cross(&thrust)
        + rp * V::new(p.prop_torque * p.thrust_max * u[0], 0., 0.);
    if let Some(t) = shaft_torque {
        moment_p += rp
            * V::new(
                -p.rotor_spin_sign * t - p.prop_torque * p.thrust_max * u[0],
                0.,
                0.,
            );
    }
    moment_p += rotor_moment(
        wp,
        rp,
        p.rotor_spin_inertia,
        p.rotor_spin_sign * rpm,
        p.spin_acceleration_scale * p.rotor_spin_sign * rpm_rate,
    );
    let mut fc = V::new(0., 0., mc * 9.80665);
    let mut moment_c = V::zeros();
    let vb = rc.transpose() * (vc - wind);
    let speed = vb.norm();
    if speed > 0.1 {
        let alpha = vb.z.atan2(vb.x);
        let beta = (vb.y / speed).clamp(-1., 1.).asin();
        let q = 0.5 * p.rho * speed * speed;
        let diff = (u[2] - u[1]) / 0.072;
        let e = vb / speed;
        let mut down = e.cross(&V::y());
        if down.norm() < 1e-8 {
            down = V::z() - e * e.z;
        }
        down.normalize_mut();
        let side = down.cross(&e);
        let (panel_f, panel_m) = panel(vb, rc.transpose() * wc, u[1] / 0.072, u[2] / 0.072, p);
        fc += rc * (panel_f + q * p.s * p.cybeta * beta * side);
        let dimensions = V::new(p.b, p.c, p.b);
        let rates = (rc.transpose() * wc).component_mul(&dimensions) / (2. * speed);
        let co = V::new(
            p.clbeta * beta + p.clp * rates.x + p.cldelta * diff,
            p.cm0 + p.cma * alpha + p.cmq * rates.y,
            p.cnbeta * beta + p.cnr * rates.z + p.cndelta * diff,
        );
        moment_c = rc * (panel_m + q * p.s * dimensions.component_mul(&co));
    }
    let Some(joint) = crate::joint::joint_loads(rp, rc, wp, wc, mp, p) else {
        return State::repeat(f64::NAN);
    };
    moment_p += joint.total;
    moment_c -= joint.total;
    let hc = canopy_inertia(p);
    let mut nu = V6::zeros();
    nu.fixed_rows_mut::<3>(0)
        .copy_from(&(rc.transpose() * (vc - wind)));
    nu.fixed_rows_mut::<3>(3).copy_from(&(rc.transpose() * wc));
    let linear = nu.fixed_rows::<3>(0).into_owned();
    let angular = nu.fixed_rows::<3>(3).into_owned();
    let momentum = hc * nu;
    let pm = momentum.fixed_rows::<3>(0).into_owned();
    let am = momentum.fixed_rows::<3>(3).into_owned();
    let mut forcing = V6::zeros();
    forcing
        .fixed_rows_mut::<3>(0)
        .copy_from(&(rc.transpose() * fc - angular.cross(&pm)));
    forcing
        .fixed_rows_mut::<3>(3)
        .copy_from(&(rc.transpose() * moment_c - angular.cross(&am) - linear.cross(&pm)));
    let Some(chol) = hc.cholesky() else {
        return State::repeat(f64::NAN);
    };
    let mut local = chol.solve(&forcing);
    let corrected = local.fixed_rows::<3>(0).into_owned() + angular.cross(&linear);
    local.fixed_rows_mut::<3>(0).copy_from(&corrected);
    let r6 = spatial_rotation(rc);
    let hci = r6 * chol.inverse() * r6.transpose();
    let mut hpi = M6::zeros();
    hpi.fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&(M::identity() / mp));
    hpi.fixed_view_mut::<3, 3>(3, 3).copy_from(&ipi);
    let mut cp = V6::zeros();
    cp.fixed_rows_mut::<3>(0).copy_from(&(fp / mp));
    cp.fixed_rows_mut::<3>(3)
        .copy_from(&(ipi * (moment_p - wp.cross(&(ip * wp)))));
    let mut cc = r6 * local;
    let gp = constraint(arm_p);
    let gc = constraint(arm_c);
    let a = gp * hpi * gp.transpose() + gc * hci * gc.transpose();
    let mismatch = gp * cp + wp.cross(&wp.cross(&arm_p)) - gc * cc - wc.cross(&wc.cross(&arm_c));
    let Some(solve) = a.cholesky() else {
        return State::repeat(f64::NAN);
    };
    let reaction = solve.solve(&-mismatch);
    cp += hpi * gp.transpose() * reaction;
    cc -= hci * gc.transpose() * reaction;
    let mut dy = State::zeros();
    for (k, acceleration) in [(PAYLOAD, cp), (CANOPY, cc)] {
        let w = vector(y, k + 10);
        let qv = vector(y, k + 3);
        dy.fixed_rows_mut::<3>(k).copy_from(&vector(y, k + 7));
        dy.fixed_rows_mut::<3>(k + 3)
            .copy_from(&(0.5 * (y[k + 6] * w + w.cross(&qv))));
        dy[k + 6] = -0.5 * w.dot(&qv);
        dy.fixed_rows_mut::<6>(k + 7).copy_from(&acceleration);
    }
    dy
}
