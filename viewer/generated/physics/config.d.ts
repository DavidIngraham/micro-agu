// Generated from Rust; do not edit.
export type Parameters = { rho: number, S: number, b: number, c: number, CL0: number, CLa: number, CD0: number, k_drag: number, CYbeta: number, CLbrake: number, CDbrake: number, Clbeta: number, Clp: number, Cldelta: number, Cm0: number, Cma: number, Cmq: number, payload_CDA: number, thrust_max: number, thrust_z: number, prop_torque: number, joint_k_roll: number, joint_k_pitch: number, joint_k_yaw: number, joint_c_roll: number, joint_c_pitch: number, joint_c_yaw: number, Cnbeta: number, Cnr: number, Cndelta: number, added_mx: number, added_my: number, added_mz: number, added_Jx: number, added_Jy: number, added_Jz: number, added_z1: number, added_z2: number, canopy_half_arc: number, hinge_x: number, hinge_y: number, hinge_z: number, Ixx: number, Iyy: number, Izz: number, Ixy: number, Ixz: number, Iyz: number, thrust_x: number, thrust_y: number, thrust_arm_z: number, canopy_mass: number, brake_tilt_rad_per_m: number, rotor_spin_inertia: number, rotor_spin_sign: number, geometry_yaw: number, yaw_payload_halfspacing: number, yaw_canopy_halfspacing: number, yaw_vertical_separation: number, spin_acceleration_scale: number, routing_enabled: number, riser_halfspacing: number, guide_halfspacing: number, guide_x: number, ring_height: number, support_halfspacing: number, support_height: number, cascade_x: number, route_reference_left: number, route_reference_right: number,
/**
 * Quasi-static aft center-of-pressure shift, as a fraction of chord at full brake.
 * Research option; zero preserves the calibrated production model.
 */
brake_cp_shift: number,
/**
 * Blend from uniform half-wing braking to outboard loading, preserving mean brake.
 */
brake_outboard_fraction: number, };
export type SimulationConfig = { schema_version: number, parameters: Parameters, mass_kg: number, initial_state: [number, number, number, number, number, number, number, number, number, number, number, number, number, number, number, number, number, number, number, number, number, number, number, number, number, number], voltage_v: number, initial_rpm: number, substeps: number, };
