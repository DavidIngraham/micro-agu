use sim_core::{launch, state::*, Parameters};
fn trim() -> State {
    let mut y = State::zeros();
    y[6] = 1.;
    y[19] = 1.;
    y[2] = -100.;
    y[15] = -101.;
    y
}
#[test]
fn inverted_launch_is_continuous_and_releases_a_consistent_two_body_state() {
    let p = Parameters::default();
    let reference = trim();
    let mut previous = launch::sample(&reference, &p, 0., 1.2).unwrap();
    assert!(previous[6] > 0.99 && previous[17].abs() > 0.99);
    for i in 1..=5800 {
        let t = i as f64 / 1000.;
        let sample = launch::sample(&reference, &p, t, 1.2).unwrap();
        assert!(sample.iter().all(|x| x.is_finite()));
        for k in [0, 13] {
            let displacement = (V::from_row_slice(&sample[k..k + 3])
                - V::from_row_slice(&previous[k..k + 3]))
            .norm();
            assert!(displacement < 0.02, "position discontinuity at {t}");
            let dv = (V::from_row_slice(&sample[k + 7..k + 10])
                - V::from_row_slice(&previous[k + 7..k + 10]))
            .norm();
            assert!(dv < 0.1, "velocity discontinuity at {t}: {dv}");
        }
        if (2. ..2.8).contains(&t) {
            assert!(sample[4].abs() > 0.99);
        }
        previous = sample;
    }
    let y = State::from_column_slice(&previous[..26]);
    let hinge = vector(&y, 0) + rotation(&y, 0) * V::new(p.hinge_x, p.hinge_y, p.hinge_z);
    let canopy_hinge = vector(&y, 13) + rotation(&y, 13) * V::new(0., 0., 0.85);
    assert!((hinge - canopy_hinge).norm() < 1e-9);
    assert!((vector(&y, 7) - vector(&y, 20)).norm() < 1e-7);
    assert!((y[7] - 4.5).abs() < 1e-6);
    assert!(y[15] < y[2] && previous[26] > 0.99);
    assert!(launch::sample(&reference, &p, f64::NAN, 1.2).is_err());
}

#[test]
fn walk_and_toss_have_continuous_acceleration() {
    let p = Parameters::default();
    let reference = trim();
    let acceleration = |t: f64| {
        let a = launch::sample(&reference, &p, t - 0.0001, 1.2).unwrap();
        let b = launch::sample(&reference, &p, t + 0.0001, 1.2).unwrap();
        (V::from_row_slice(&b[7..10]) - V::from_row_slice(&a[7..10])) / 0.0002
    };
    for t in [3.2, 3.8, 4.8, 5.0, 5.4] {
        assert!(
            (acceleration(t + 0.001) - acceleration(t - 0.001)).norm() < 0.3,
            "acceleration jump at {t}"
        );
    }
}
