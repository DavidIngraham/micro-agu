// Research-only reduced tendon mechanics; no new production parameters.
use sim_core::{config::Parameters, dynamics, propulsion::*, state::*};

use serde::Deserialize;
use std::io::{self, BufRead};

#[derive(Deserialize)]
struct Request {
    parameters: Vec<f64>,
    mass: f64,
    state: Vec<f64>,
    commands: Vec<[f64; 3]>,
    wind: [f64; 3],
    voltage: f64,
    rpm: f64,
    #[serde(default)]
    stiffness: f64,
    #[serde(default)]
    frozen: bool,
    #[serde(default)]
    frozen_reference: Option<[f64; 2]>,
    #[serde(default = "step")]
    dt: f64,
    #[serde(default = "substeps")]
    substeps: usize,
}
fn step() -> f64 {
    0.04
}
fn substeps() -> usize {
    32
}
fn path(y: &State, p: &Parameters) -> [f64; 2] {
    let rp = rotation(y, 0);
    let rc = rotation(y, 13);
    [-1., 1.].map(|side| dynamics::brake_path(rp, rc, p, side))
}
fn effective(y: &State, u: &[f64; 3], p: &Parameters) -> [f64; 2] {
    let l = path(y, p);
    [
        (u[1] + l[0] - p.route_reference_left).max(0.),
        (u[2] + l[1] - p.route_reference_right).max(0.),
    ]
}
// With the hinge constraint imposed, route length is a function of relative
// orientation alone. Virtual work gives Qc=-T*dL/dtheta_c and Qp=-Qc.
// This includes ideal massless guide reactions, not a point force guessed at a servo.
fn load(y: &State, u: &[f64; 3], p: &Parameters, k: f64) -> V {
    if k == 0. {
        return V::zeros();
    }
    let tension = effective(y, u, p).map(|b| k * b);
    let rp = rotation(y, 0);
    let rc = rotation(y, 13);
    let mut gradient = V::zeros();
    for (i, side) in [-1., 1.].iter().enumerate() {
        let anchor = rp * V::new(0., side * p.riser_halfspacing, 0.);
        let guide = rp * V::new(p.guide_x, side * p.guide_halfspacing, 0.);
        let support = rc * V::new(0., side * p.support_halfspacing, -p.support_height);
        let length = (support - anchor).norm();
        let n = (support - anchor) / length;
        let ring = anchor + p.ring_height * n;
        let cascade = support + rc * V::new(p.cascade_x, 0., 0.);
        let first = (guide - ring).normalize();
        let second = (ring - cascade).normalize();
        for axis in 0..3 {
            let mut e = V::zeros();
            e[axis] = 1.;
            let ds = e.cross(&support);
            let dr = p.ring_height / length * (ds - n * n.dot(&ds));
            let dc = e.cross(&cascade);
            gradient[axis] += tension[i] * (-first.dot(&dr) + second.dot(&(dr - dc)));
        }
    }
    gradient
}
fn run(r: Request) -> Result<serde_json::Value, String> {
    let p = Parameters::from_packed(&r.parameters)?;
    p.validate(r.mass)?;
    if r.state.len() != 26
        || r.commands.is_empty()
        || r.stiffness < 0.
        || !r.stiffness.is_finite()
        || r.dt <= 0.
        || r.dt > 1.
        || r.substeps == 0
        || r.substeps > 10000
    {
        return Err("invalid request".into());
    }
    let mut y = State::from_column_slice(&r.state);
    validate(&y)?;
    let wind = V::from(r.wind);
    let mut rpm = r.rpm;
    let frozen_path = r.frozen_reference.unwrap_or_else(|| path(&y, &p));
    let mut aero = p.clone();
    if r.frozen {
        aero.routing_enabled = 0.;
    }
    let mut rows = Vec::new();
    for (index, raw) in r.commands.iter().enumerate() {
        let mut row = y.as_slice().to_vec();
        row.push(rpm);
        row.extend(effective(&y, raw, &p));
        row.extend(load(&y, raw, &p, r.stiffness).as_slice());
        rows.push(row);
        if index + 1 == r.commands.len() {
            break;
        }
        let mut u = *raw;
        if r.frozen {
            u[1] = (u[1] + frozen_path[0] - p.route_reference_left).max(0.);
            u[2] = (u[2] + frozen_path[1] - p.route_reference_right).max(0.);
        }
        let h = r.dt / r.substeps as f64;
        for _ in 0..r.substeps {
            let target = target_rpm(u[0], r.voltage);
            let derivative = |z: &State, t: f64| {
                let speed = target + (rpm - target) * (-t / MOTOR_TAU_S).exp();
                let axial = (rotation(z, 0).transpose() * (vector(z, 7) - wind)).x;
                dynamics::rhs_with_joint_torque(
                    z,
                    &[thrust(speed, axial) / p.thrust_max, u[1], u[2]],
                    &aero,
                    r.mass,
                    wind,
                    speed,
                    (target - speed) / MOTOR_TAU_S,
                    Some(torque(speed, axial)),
                    load(z, raw, &p, r.stiffness),
                )
            };
            let a = derivative(&y, 0.);
            let b = derivative(&(y + 0.5 * h * a), 0.5 * h);
            let c = derivative(&(y + 0.5 * h * b), 0.5 * h);
            let d = derivative(&(y + h * c), h);
            y += h / 6. * (a + 2. * b + 2. * c + d);
            normalize(&mut y);
            validate(&y)?;
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
    fn setup() -> (State, Parameters) {
        let mut y = State::zeros();
        y[6] = 1.;
        y[19] = 1.;
        let mut p = Parameters::default();
        p.routing_enabled = 1.;
        p.riser_halfspacing = 0.066;
        p.guide_halfspacing = 0.0455;
        p.guide_x = -0.0225;
        p.ring_height = 0.06;
        p.support_halfspacing = 0.32;
        p.support_height = 0.85;
        p.cascade_x = -0.1;
        let l = path(&y, &p);
        p.route_reference_left = l[0];
        p.route_reference_right = l[1];
        (y, p)
    }
    #[test]
    fn slack_and_zero_have_no_load() {
        let (y, p) = setup();
        assert_eq!(load(&y, &[0., -0.1, -0.1], &p, 300.), V::zeros());
        assert_eq!(load(&y, &[0., 0.02, 0.02], &p, 0.), V::zeros());
    }
    #[test]
    fn symmetric_pull_has_no_roll_or_yaw() {
        let (y, p) = setup();
        let q = load(&y, &[0., 0.02, 0.02], &p, 100.);
        assert!(q.x.abs() < 1e-9 && q.z.abs() < 1e-9 && q.y.abs() > 1e-4);
    }
    #[test]
    fn virtual_work_matches_elastic_energy() {
        let (mut y, p) = setup();
        let u = [0., 0.02, 0.01];
        let k = 100.;
        let q = load(&y, &u, &p, k);
        let direction = V::new(0.3, -0.7, 0.2);
        let h = 1e-5;
        let energy = |z: &State| {
            effective(z, &u, &p)
                .iter()
                .map(|b| 0.5 * k * b * b)
                .sum::<f64>()
        };
        let e0 = energy(&y);
        let rot = nalgebra::UnitQuaternion::from_scaled_axis(direction * h);
        y.fixed_rows_mut::<4>(16).copy_from(&rot.coords);
        assert!(((energy(&y) - e0) / h - q.dot(&direction)).abs() < 1e-6);
    }
}
