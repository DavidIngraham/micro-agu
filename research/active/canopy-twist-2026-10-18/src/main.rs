use nalgebra::{Rotation3, SVector};
use serde::Deserialize;
use sim_core::{
    config::Parameters,
    dynamics::{self, BodyLoads},
    propulsion::*,
    state::*,
};
use std::io::{self, BufRead};
type Extended = SVector<f64, 28>;
#[derive(Clone, Deserialize)]
struct Experiment {
    enabled: bool,
    frequency: f64,
    damping: f64,
    arm_fraction: f64,
    beta_gain: f64,
}
#[derive(Deserialize)]
struct Request {
    parameters: Vec<f64>,
    mass: f64,
    state: Vec<f64>,
    commands: Vec<[f64; 3]>,
    wind: [f64; 3],
    voltage: f64,
    rpm: f64,
    experiment: Experiment,
    #[serde(default = "steps")]
    substeps: usize,
    #[serde(default = "dt")]
    dt: f64,
    #[serde(default)]
    twist: [f64; 2],
}
fn steps() -> usize {
    32
}
fn dt() -> f64 {
    0.04
}
const MODAL_INERTIA: f64 = 0.001;
const SOFT_ANGLE: f64 = 0.35;
// Positive linear+cubic elastic energy; angular stiffness stiffens at large twist.
fn structural_accel(q: f64, qd: f64, e: &Experiment) -> f64 {
    let w = 2. * std::f64::consts::PI * e.frequency;
    -w * w * (q + q.powi(3) / SOFT_ANGLE.powi(2)) - 2. * e.damping * w * qd
}
fn aero(
    y: &State,
    u: &[f64; 3],
    p: &Parameters,
    wind: V,
    q: f64,
    qd: f64,
    e: &Experiment,
) -> (BodyLoads, f64) {
    let rc = rotation(y, 13);
    let rp = rotation(y, 0);
    let v = rc.transpose() * (vector(y, 20) - wind);
    let w = rc.transpose() * vector(y, 23);
    if !e.enabled || v.norm() <= 0.1 {
        return (BodyLoads::default(), 0.);
    }
    let mut brakes = [u[1], u[2]];
    if p.routing_enabled > 0. {
        for i in 0..2 {
            brakes[i] = (brakes[i]
                + dynamics::brake_path(rp, rc, p, if i == 0 { -1. } else { 1. })
                - [p.route_reference_left, p.route_reference_right][i])
                .max(0.);
        }
    }
    let base = dynamics::panel_loads(v, w, brakes[0] / 0.072, brakes[1] / 0.072, p);
    let mut df = V::zeros();
    let mut dm = V::zeros();
    let mut generalized = 0.;
    for (i, b) in base.iter().enumerate() {
        let eta = (i as f64 + 0.5) / 5. * 2. - 1.;
        let span = Rotation3::from_axis_angle(&V::x_axis(), b.inclination);
        let twist = Rotation3::from_axis_angle(&V::y_axis(), eta * q);
        let rotation = span * twist;
        let arm = V::new(e.arm_fraction * p.c, 0., 0.);
        let moving = twist * arm;
        let displacement = span * (moving - arm);
        let tangent = span * (eta * V::y().cross(&moving));
        let position = b.position + displacement;
        let local = rotation.inverse() * (v + w.cross(&position) + tangent * qd);
        let alpha = local.z.atan2(local.x);
        let cl = (p.cl0 + p.cla * alpha + p.clbrake * b.brake) / (1. + (alpha.abs() / 0.6).powi(4));
        let cd = p.cd0 + p.k_drag * cl * cl + p.cdbrake * b.brake * b.brake;
        let f = rotation
            * (0.5 * p.rho * p.s / 5.
                * (local.x.hypot(local.z) * cl * V::new(local.z, 0., -local.x)
                    - local.norm() * cd * local));
        df += f - b.force;
        dm += position.cross(&f) - b.position.cross(&b.force);
        // Virtual work: generalized torque is d(position)/d(twist) dot panel force.
        generalized += tangent.dot(&f);
    }
    let beta = (v.y / v.norm()).clamp(-1., 1.).asin();
    let omega = 2. * std::f64::consts::PI * e.frequency;
    // Optional phenomenological side-load term, scaled with dynamic pressure.
    generalized += MODAL_INERTIA * omega * omega * e.beta_gain * beta * (v.norm() / 4.).powi(2);
    (
        BodyLoads {
            canopy_force: rc * df,
            canopy_moment: rc * dm,
            ..Default::default()
        },
        generalized,
    )
}
fn derivative(
    z: &Extended,
    u: &[f64; 3],
    p: &Parameters,
    r: &Request,
    rpm: f64,
    rate: f64,
) -> Extended {
    let y = State::from_column_slice(&z.as_slice()[..26]);
    let wind = V::from(r.wind);
    let (loads, g) = aero(&y, u, p, wind, z[26], z[27], &r.experiment);
    let axial = (rotation(&y, 0).transpose() * (vector(&y, 7) - wind)).x;
    let mut applied = *u;
    applied[0] = thrust(rpm, axial) / p.thrust_max;
    let dy = dynamics::rhs_with_loads(
        &y,
        &applied,
        p,
        r.mass,
        wind,
        rpm,
        rate,
        Some(torque(rpm, axial)),
        &loads,
        true,
    );
    let mut out = Extended::zeros();
    out.as_mut_slice()[..26].copy_from_slice(dy.as_slice());
    if r.experiment.enabled {
        out[26] = z[27];
        out[27] = g / MODAL_INERTIA + structural_accel(z[26], z[27], &r.experiment);
    }
    out
}
fn run(r: Request) -> Result<serde_json::Value, String> {
    let p = Parameters::from_packed(&r.parameters)?;
    p.validate(r.mass)?;
    if r.state.len() != 26
        || r.commands.is_empty()
        || r.substeps == 0
        || r.substeps > 1000
        || r.dt <= 0.
        || r.dt > 0.1
        || r.experiment.frequency <= 0.
        || r.experiment.damping < 0.
        || r.voltage <= 0.
    {
        return Err("invalid request".into());
    }
    let mut z = Extended::zeros();
    z.as_mut_slice()[..26].copy_from_slice(&r.state);
    z[26] = r.twist[0];
    z[27] = r.twist[1];
    let mut rpm = r.rpm;
    let mut rows = Vec::new();
    for (i, u) in r.commands.iter().enumerate() {
        let mut row = z.as_slice().to_vec();
        row.push(rpm);
        rows.push(row);
        if i + 1 == r.commands.len() {
            break;
        }
        let h = r.dt / r.substeps as f64;
        for _ in 0..r.substeps {
            let target = target_rpm(u[0], r.voltage);
            let f = |x: &Extended, t: f64| {
                let n = target + (rpm - target) * (-t / MOTOR_TAU_S).exp();
                derivative(x, u, &p, &r, n, (target - n) / MOTOR_TAU_S)
            };
            let a = f(&z, 0.);
            let b = f(&(z + a * h * 0.5), h * 0.5);
            let c = f(&(z + b * h * 0.5), h * 0.5);
            let d = f(&(z + c * h), h);
            z += h / 6. * (a + 2. * b + 2. * c + d);
            let mut y = State::from_column_slice(&z.as_slice()[..26]);
            normalize(&mut y);
            validate(&y)?;
            z.as_mut_slice()[..26].copy_from_slice(y.as_slice());
            if !z.iter().all(|x| x.is_finite()) || z[26].abs() > 1. {
                return Err("invalid or >1 rad twist".into());
            }
            rpm = target + (rpm - target) * (-h / MOTOR_TAU_S).exp();
        }
    }
    Ok(serde_json::json!({"rows":rows}))
}
fn main() {
    for line in io::stdin().lock().lines() {
        let result = line
            .map_err(|e| e.to_string())
            .and_then(|s| serde_json::from_str::<Request>(&s).map_err(|e| e.to_string()))
            .and_then(run);
        println!(
            "{}",
            match result {
                Ok(v) => v,
                Err(e) => serde_json::json!({"error":e}),
            }
        );
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    fn setup() -> (State, Parameters, Experiment) {
        let mut y = State::zeros();
        y[6] = 1.;
        y[19] = 1.;
        y[7] = 4.;
        y[20] = 4.;
        y[22] = 1.;
        (
            y,
            Parameters::default(),
            Experiment {
                enabled: true,
                frequency: 1.6,
                damping: 0.3,
                arm_fraction: 0.03,
                beta_gain: 0.,
            },
        )
    }
    #[test]
    fn zero_twist_preserves_panel_loads() {
        let (y, p, e) = setup();
        let (l, _) = aero(&y, &[0.4, 0.01, 0.02], &p, V::zeros(), 0., 0., &e);
        assert!(l.canopy_force.norm() < 1e-12);
        assert!(l.canopy_moment.norm() < 1e-12);
    }
    #[test]
    fn symmetric_flow_does_not_drive_twist() {
        let (y, p, e) = setup();
        let (_, g) = aero(&y, &[0.4, 0.01, 0.01], &p, V::zeros(), 0., 0., &e);
        assert!(g.abs() < 1e-12);
    }
    #[test]
    fn positive_structure_restores_and_dissipates() {
        let (_, _, e) = setup();
        assert!(structural_accel(0.1, 0., &e) < 0.);
        assert!(structural_accel(-0.1, 0., &e) > 0.);
        assert!(structural_accel(0., 0.1, &e) < 0.);
    }
    #[test]
    fn twist_mirrors_lateral_loads() {
        let (y, p, e) = setup();
        let (a, ga) = aero(&y, &[0.4, 0.01, 0.01], &p, V::zeros(), 0.03, 0., &e);
        let (b, gb) = aero(&y, &[0.4, 0.01, 0.01], &p, V::zeros(), -0.03, 0., &e);
        assert!((a.canopy_force.y + b.canopy_force.y).abs() < 1e-12);
        assert!((a.canopy_moment.x + b.canopy_moment.x).abs() < 1e-12);
        assert!((ga + gb).abs() < 1e-12);
    }

    #[test]
    fn generalized_load_matches_virtual_work_at_zero_twist() {
        let (y, p, e) = setup();
        let u = [0.4, 0.01, 0.025];
        let eps = 1e-6;
        let (_, g) = aero(&y, &u, &p, V::zeros(), 0., 0., &e);
        let base = dynamics::panel_loads(
            V::new(4., 0., 1.),
            V::zeros(),
            u[1] / 0.072,
            u[2] / 0.072,
            &p,
        );
        let mut numeric = 0.;
        for (i, b) in base.iter().enumerate() {
            let eta = (i as f64 + 0.5) / 5. * 2. - 1.;
            let r = Rotation3::from_axis_angle(&V::x_axis(), b.inclination);
            let arm = V::new(e.arm_fraction * p.c, 0., 0.);
            let plus = r * (Rotation3::from_axis_angle(&V::y_axis(), eta * eps) * arm - arm);
            let minus = r * (Rotation3::from_axis_angle(&V::y_axis(), -eta * eps) * arm - arm);
            numeric += (plus - minus).dot(&b.force) / (2. * eps);
        }
        assert!((numeric - g).abs() < 1e-10);
    }
}
