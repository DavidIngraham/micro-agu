//! Named physical coefficients. Packed order exists only for legacy parity adapters.
use crate::state::{State, M};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema, ts_rs::TS)]
#[serde(deny_unknown_fields)]
pub struct Parameters {
    pub rho: f64,
    #[serde(rename = "S")]
    pub s: f64,
    pub b: f64,
    pub c: f64,
    #[serde(rename = "CL0")]
    pub cl0: f64,
    #[serde(rename = "CLa")]
    pub cla: f64,
    #[serde(rename = "CD0")]
    pub cd0: f64,
    pub k_drag: f64,
    #[serde(rename = "CYbeta")]
    pub cybeta: f64,
    #[serde(rename = "CLbrake")]
    pub clbrake: f64,
    #[serde(rename = "CDbrake")]
    pub cdbrake: f64,
    #[serde(rename = "Clbeta")]
    pub clbeta: f64,
    #[serde(rename = "Clp")]
    pub clp: f64,
    #[serde(rename = "Cldelta")]
    pub cldelta: f64,
    #[serde(rename = "Cm0")]
    pub cm0: f64,
    #[serde(rename = "Cma")]
    pub cma: f64,
    #[serde(rename = "Cmq")]
    pub cmq: f64,
    #[serde(rename = "payload_CDA")]
    pub payload_cda: f64,
    pub thrust_max: f64,
    pub thrust_z: f64,
    pub prop_torque: f64,
    pub joint_k_roll: f64,
    pub joint_k_pitch: f64,
    pub joint_k_yaw: f64,
    pub joint_c_roll: f64,
    pub joint_c_pitch: f64,
    pub joint_c_yaw: f64,
    #[serde(rename = "Cnbeta")]
    pub cnbeta: f64,
    #[serde(rename = "Cnr")]
    pub cnr: f64,
    #[serde(rename = "Cndelta")]
    pub cndelta: f64,
    pub added_mx: f64,
    pub added_my: f64,
    pub added_mz: f64,
    #[serde(rename = "added_Jx")]
    pub added_jx: f64,
    #[serde(rename = "added_Jy")]
    pub added_jy: f64,
    #[serde(rename = "added_Jz")]
    pub added_jz: f64,
    pub added_z1: f64,
    pub added_z2: f64,
    pub canopy_half_arc: f64,
    pub hinge_x: f64,
    pub hinge_y: f64,
    pub hinge_z: f64,
    #[serde(rename = "Ixx")]
    pub ixx: f64,
    #[serde(rename = "Iyy")]
    pub iyy: f64,
    #[serde(rename = "Izz")]
    pub izz: f64,
    #[serde(rename = "Ixy")]
    pub ixy: f64,
    #[serde(rename = "Ixz")]
    pub ixz: f64,
    #[serde(rename = "Iyz")]
    pub iyz: f64,
    pub thrust_x: f64,
    pub thrust_y: f64,
    pub thrust_arm_z: f64,
    pub canopy_mass: f64,
    pub brake_tilt_rad_per_m: f64,
    pub rotor_spin_inertia: f64,
    pub rotor_spin_sign: f64,
    pub geometry_yaw: f64,
    pub yaw_payload_halfspacing: f64,
    pub yaw_canopy_halfspacing: f64,
    pub yaw_vertical_separation: f64,
    pub spin_acceleration_scale: f64,
    pub routing_enabled: f64,
    pub riser_halfspacing: f64,
    pub guide_halfspacing: f64,
    pub guide_x: f64,
    pub ring_height: f64,
    pub support_halfspacing: f64,
    pub support_height: f64,
    pub cascade_x: f64,
    pub route_reference_left: f64,
    pub route_reference_right: f64,
    /// Quasi-static aft center-of-pressure shift, as a fraction of chord at full brake.
    /// Research option; zero preserves the calibrated production model.
    #[serde(default)]
    pub brake_cp_shift: f64,
    /// Blend from uniform half-wing braking to outboard loading, preserving mean brake.
    #[serde(default)]
    pub brake_outboard_fraction: f64,
}
impl Default for Parameters {
    fn default() -> Self {
        Self {
            rho: 1.225,
            s: 1.16,
            b: 2.15,
            c: 0.54,
            cl0: 0.4,
            cla: 2.0,
            cd0: 0.15,
            k_drag: 0.3,
            cybeta: -0.2,
            clbrake: 1.0,
            cdbrake: 1.0,
            clbeta: -0.03,
            clp: -0.1,
            cldelta: 0.05,
            cm0: 0.02,
            cma: -0.2,
            cmq: -2.0,
            payload_cda: 0.015,
            thrust_max: 10.0,
            thrust_z: -0.1,
            prop_torque: -0.02,
            joint_k_roll: 0.4,
            joint_k_pitch: 0.0,
            joint_k_yaw: 0.05,
            joint_c_roll: 0.03,
            joint_c_pitch: 0.015,
            joint_c_yaw: 0.02,
            cnbeta: 0.03,
            cnr: -0.05,
            cndelta: 0.04,
            added_mx: 0.01,
            added_my: 0.1,
            added_mz: 0.5,
            added_jx: 0.01,
            added_jy: 0.005,
            added_jz: 0.005,
            added_z1: 0.0,
            added_z2: 0.5,
            canopy_half_arc: 0.6,
            hinge_x: 0.0,
            hinge_y: 0.0,
            hinge_z: -0.13,
            ixx: 0.006,
            iyy: 0.008,
            izz: 0.003,
            ixy: 0.0,
            ixz: 0.0,
            iyz: 0.0,
            thrust_x: 0.0,
            thrust_y: 0.0,
            thrust_arm_z: -0.058,
            canopy_mass: 0.19,
            brake_tilt_rad_per_m: 0.0,
            rotor_spin_inertia: 0.0,
            rotor_spin_sign: 0.0,
            geometry_yaw: 0.0,
            yaw_payload_halfspacing: 0.0743,
            yaw_canopy_halfspacing: 0.5,
            yaw_vertical_separation: 0.85,
            spin_acceleration_scale: 0.0,
            routing_enabled: 0.0,
            riser_halfspacing: 0.066,
            guide_halfspacing: 0.0455,
            guide_x: -0.0225,
            ring_height: 0.06,
            support_halfspacing: 0.3235535694,
            support_height: 0.85,
            cascade_x: -0.1,
            route_reference_left: 0.0,
            route_reference_right: 0.0,
            brake_cp_shift: 0.0,
            brake_outboard_fraction: 0.0,
        }
    }
}
impl Parameters {
    pub const KEYS: [&'static str; 72] = [
        "rho",
        "S",
        "b",
        "c",
        "CL0",
        "CLa",
        "CD0",
        "k_drag",
        "CYbeta",
        "CLbrake",
        "CDbrake",
        "Clbeta",
        "Clp",
        "Cldelta",
        "Cm0",
        "Cma",
        "Cmq",
        "payload_CDA",
        "thrust_max",
        "thrust_z",
        "prop_torque",
        "joint_k_roll",
        "joint_k_pitch",
        "joint_k_yaw",
        "joint_c_roll",
        "joint_c_pitch",
        "joint_c_yaw",
        "Cnbeta",
        "Cnr",
        "Cndelta",
        "added_mx",
        "added_my",
        "added_mz",
        "added_Jx",
        "added_Jy",
        "added_Jz",
        "added_z1",
        "added_z2",
        "canopy_half_arc",
        "hinge_x",
        "hinge_y",
        "hinge_z",
        "Ixx",
        "Iyy",
        "Izz",
        "Ixy",
        "Ixz",
        "Iyz",
        "thrust_x",
        "thrust_y",
        "thrust_arm_z",
        "canopy_mass",
        "brake_tilt_rad_per_m",
        "rotor_spin_inertia",
        "rotor_spin_sign",
        "geometry_yaw",
        "yaw_payload_halfspacing",
        "yaw_canopy_halfspacing",
        "yaw_vertical_separation",
        "spin_acceleration_scale",
        "routing_enabled",
        "riser_halfspacing",
        "guide_halfspacing",
        "guide_x",
        "ring_height",
        "support_halfspacing",
        "support_height",
        "cascade_x",
        "route_reference_left",
        "route_reference_right",
        "brake_cp_shift",
        "brake_outboard_fraction",
    ];
    pub fn packed(&self) -> [f64; 72] {
        [
            self.rho,
            self.s,
            self.b,
            self.c,
            self.cl0,
            self.cla,
            self.cd0,
            self.k_drag,
            self.cybeta,
            self.clbrake,
            self.cdbrake,
            self.clbeta,
            self.clp,
            self.cldelta,
            self.cm0,
            self.cma,
            self.cmq,
            self.payload_cda,
            self.thrust_max,
            self.thrust_z,
            self.prop_torque,
            self.joint_k_roll,
            self.joint_k_pitch,
            self.joint_k_yaw,
            self.joint_c_roll,
            self.joint_c_pitch,
            self.joint_c_yaw,
            self.cnbeta,
            self.cnr,
            self.cndelta,
            self.added_mx,
            self.added_my,
            self.added_mz,
            self.added_jx,
            self.added_jy,
            self.added_jz,
            self.added_z1,
            self.added_z2,
            self.canopy_half_arc,
            self.hinge_x,
            self.hinge_y,
            self.hinge_z,
            self.ixx,
            self.iyy,
            self.izz,
            self.ixy,
            self.ixz,
            self.iyz,
            self.thrust_x,
            self.thrust_y,
            self.thrust_arm_z,
            self.canopy_mass,
            self.brake_tilt_rad_per_m,
            self.rotor_spin_inertia,
            self.rotor_spin_sign,
            self.geometry_yaw,
            self.yaw_payload_halfspacing,
            self.yaw_canopy_halfspacing,
            self.yaw_vertical_separation,
            self.spin_acceleration_scale,
            self.routing_enabled,
            self.riser_halfspacing,
            self.guide_halfspacing,
            self.guide_x,
            self.ring_height,
            self.support_halfspacing,
            self.support_height,
            self.cascade_x,
            self.route_reference_left,
            self.route_reference_right,
            self.brake_cp_shift,
            self.brake_outboard_fraction,
        ]
    }
    pub fn from_packed(values: &[f64]) -> crate::Result<Self> {
        if !matches!(values.len(), 70 | 72) || !values.iter().all(|x| x.is_finite()) {
            return Err("expected 70 legacy or 72 finite parameters".into());
        }
        Ok(Self {
            rho: values[0],
            s: values[1],
            b: values[2],
            c: values[3],
            cl0: values[4],
            cla: values[5],
            cd0: values[6],
            k_drag: values[7],
            cybeta: values[8],
            clbrake: values[9],
            cdbrake: values[10],
            clbeta: values[11],
            clp: values[12],
            cldelta: values[13],
            cm0: values[14],
            cma: values[15],
            cmq: values[16],
            payload_cda: values[17],
            thrust_max: values[18],
            thrust_z: values[19],
            prop_torque: values[20],
            joint_k_roll: values[21],
            joint_k_pitch: values[22],
            joint_k_yaw: values[23],
            joint_c_roll: values[24],
            joint_c_pitch: values[25],
            joint_c_yaw: values[26],
            cnbeta: values[27],
            cnr: values[28],
            cndelta: values[29],
            added_mx: values[30],
            added_my: values[31],
            added_mz: values[32],
            added_jx: values[33],
            added_jy: values[34],
            added_jz: values[35],
            added_z1: values[36],
            added_z2: values[37],
            canopy_half_arc: values[38],
            hinge_x: values[39],
            hinge_y: values[40],
            hinge_z: values[41],
            ixx: values[42],
            iyy: values[43],
            izz: values[44],
            ixy: values[45],
            ixz: values[46],
            iyz: values[47],
            thrust_x: values[48],
            thrust_y: values[49],
            thrust_arm_z: values[50],
            canopy_mass: values[51],
            brake_tilt_rad_per_m: values[52],
            rotor_spin_inertia: values[53],
            rotor_spin_sign: values[54],
            geometry_yaw: values[55],
            yaw_payload_halfspacing: values[56],
            yaw_canopy_halfspacing: values[57],
            yaw_vertical_separation: values[58],
            spin_acceleration_scale: values[59],
            routing_enabled: values[60],
            riser_halfspacing: values[61],
            guide_halfspacing: values[62],
            guide_x: values[63],
            ring_height: values[64],
            support_halfspacing: values[65],
            support_height: values[66],
            cascade_x: values[67],
            route_reference_left: values[68],
            route_reference_right: values[69],
            brake_cp_shift: values.get(70).copied().unwrap_or(0.),
            brake_outboard_fraction: values.get(71).copied().unwrap_or(0.),
        })
    }
    pub fn validate(&self, mass: f64) -> crate::Result<()> {
        let p = self.packed();
        if !p.iter().all(|x| x.is_finite())
            || !mass.is_finite()
            || mass <= self.canopy_mass
            || self.canopy_mass <= 0.
        {
            return Err("invalid mass or nonfinite parameters".into());
        }
        if [self.rho, self.s, self.b, self.c, self.thrust_max]
            .iter()
            .any(|x| *x <= 0.)
        {
            return Err("density, dimensions and thrust scale must be positive".into());
        }
        if p[30..36].iter().any(|x| *x < 0.) {
            return Err("added inertia must be nonnegative".into());
        }
        if !(0.0..=0.35).contains(&self.brake_cp_shift)
            || !(0.0..=1.0).contains(&self.brake_outboard_fraction)
        {
            return Err(
                "brake load model requires CP shift in 0..0.35 chord and outboard fraction in 0..1"
                    .into(),
            );
        }
        let inertia = M::new(
            self.ixx, self.ixy, self.ixz, self.ixy, self.iyy, self.iyz, self.ixz, self.iyz,
            self.izz,
        );
        if inertia.cholesky().is_none() {
            return Err("payload inertia must be positive definite".into());
        }
        if self.geometry_yaw > 0.
            && (self.yaw_payload_halfspacing <= 0.
                || self.yaw_canopy_halfspacing <= 0.
                || self.yaw_vertical_separation <= 0.
                || self.yaw_vertical_separation.powi(2)
                    <= 4. * self.yaw_payload_halfspacing * self.yaw_canopy_halfspacing)
        {
            return Err("invalid bifilar geometry".into());
        }
        if self.routing_enabled > 0.
            && (self.riser_halfspacing <= 0. || self.support_height <= 0. || self.ring_height < 0.)
        {
            return Err("invalid brake routing geometry".into());
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema, ts_rs::TS)]
#[serde(deny_unknown_fields)]
pub struct SimulationConfig {
    pub schema_version: u32,
    pub parameters: Parameters,
    pub mass_kg: f64,
    pub initial_state: [f64; 26],
    pub voltage_v: f64,
    pub initial_rpm: f64,
    pub substeps: usize,
}
impl SimulationConfig {
    pub fn validate(&self) -> crate::Result<()> {
        if self.schema_version != 1 {
            return Err("unsupported configuration schema".into());
        }
        self.parameters.validate(self.mass_kg)?;
        crate::state::validate(&State::from_column_slice(&self.initial_state))?;
        if !self.voltage_v.is_finite()
            || self.voltage_v <= 0.
            || !self.initial_rpm.is_finite()
            || self.initial_rpm < 0.
            || self.substeps == 0
            || self.substeps > 10000
        {
            return Err("invalid voltage, RPM or integration resolution".into());
        }
        Ok(())
    }
}
