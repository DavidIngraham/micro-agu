use sim_core::{
    dynamics::{panel, panel_loads},
    state::V,
    Parameters,
};
#[test]
fn legacy_packed_parameters_default_new_physics_off() {
    let p = Parameters::default();
    let old = p.packed()[..70].to_vec();
    let q = Parameters::from_packed(&old).unwrap();
    assert_eq!(q.packed(), p.packed());
    let mut value = serde_json::to_value(&p).unwrap();
    value.as_object_mut().unwrap().remove("brake_cp_shift");
    value
        .as_object_mut()
        .unwrap()
        .remove("brake_outboard_fraction");
    let parsed: Parameters = serde_json::from_value(value).unwrap();
    assert_eq!(parsed.packed(), p.packed());
}
#[test]
fn shifted_loads_preserve_mirror_symmetry_and_power_identity() {
    let mut p = Parameters::default();
    p.brake_cp_shift = 0.25;
    p.brake_outboard_fraction = 0.7;
    p.brake_tilt_rad_per_m = 0.;
    let v = V::new(4., 0., 0.7);
    let zero = V::zeros();
    let (f, m) = panel(v, zero, 0.2, 0.6, &p);
    let (fm, mm) = panel(v, zero, 0.6, 0.2, &p);
    assert!((f.x - fm.x).abs() < 1e-12 && (f.y + fm.y).abs() < 1e-12 && (f.z - fm.z).abs() < 1e-12);
    assert!((m.x + mm.x).abs() < 1e-12 && (m.y - mm.y).abs() < 1e-12 && (m.z + mm.z).abs() < 1e-12);
    let w = V::new(0.1, -0.2, 0.3);
    let loads = panel_loads(v, w, 0.2, 0.6, &p);
    let (f, m) = panel(v, w, 0.2, 0.6, &p);
    let work: f64 = loads
        .iter()
        .map(|l| l.force.dot(&(v + w.cross(&l.position))))
        .sum();
    assert!((f.dot(&v) + m.dot(&w) - work).abs() < 1e-12);
    assert!(work <= 0.);
    assert!((loads.iter().map(|l| l.brake).sum::<f64>() - 2.5 * (0.2 + 0.6)).abs() < 1e-12);
}
#[test]
fn symmetric_brake_cp_shift_changes_pitch_without_spurious_turn() {
    let mut p = Parameters::default();
    let v = V::new(4., 0., 0.7);
    let w = V::zeros();
    let (f, m) = panel(v, w, 0.5, 0.5, &p);
    p.brake_cp_shift = 0.2;
    let (f2, m2) = panel(v, w, 0.5, 0.5, &p);
    assert!((f - f2).norm() < 1e-12);
    assert!(m2.y < m.y);
    assert!(m2.x.abs() < 1e-12 && m2.z.abs() < 1e-12);
    p.brake_cp_shift = 0.36;
    assert!(p.validate(1.).is_err());
    p.brake_cp_shift = 0.2;
    p.brake_outboard_fraction = -0.1;
    assert!(p.validate(1.).is_err());
}
