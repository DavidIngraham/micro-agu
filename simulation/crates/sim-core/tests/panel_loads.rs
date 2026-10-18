use sim_core::{
    dynamics::{panel, panel_loads},
    state::V,
    Parameters,
};
#[test]
fn panel_diagnostics_conserve_loads_and_show_symmetry() {
    let p = Parameters::default();
    for (w, left, right) in [(V::zeros(), 0.1, 0.1), (V::new(0.2, -0.1, 0.3), 0., 0.4)] {
        let v = V::new(7., 0., 0.5);
        let loads = panel_loads(v, w, left, right, &p);
        let (force, moment) = panel(v, w, left, right, &p);
        let f: V = loads.iter().map(|l| l.lift + l.drag).sum();
        let m: V = loads
            .iter()
            .map(|l| l.position.cross(&(l.lift + l.drag)))
            .sum();
        assert!((force - f).norm() < 1e-12);
        assert!((moment - m).norm() < 1e-12);
        assert!((loads[2].brake - (left + right) / 2.).abs() < 1e-12);
        if left == right {
            assert!(force.y.abs() < 1e-12);
            assert!(moment.x.abs() < 1e-12 && moment.z.abs() < 1e-12);
            assert!((loads[0].lift.norm() - loads[4].lift.norm()).abs() < 1e-12);
        }
    }
}
