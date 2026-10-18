    // Study-only tension sensitivity. See PLAN.md; never part of production.
    let mut reaction = solve.solve(&-mismatch);
    let axis = arm_c.normalize();
    let base = -axis.dot(&reaction);
    let reference = p.tension_reference;
    let geom = rp * jacobian.transpose() * V::new(
        p.tension_roll_fraction * p.joint_k_roll * err.x, 0.,
        p.tension_yaw_fraction * p.joint_k_yaw * err.z);
    let mut fp_sensitivity = V6::zeros();
    let mut fc_sensitivity = V6::zeros();
    fp_sensitivity.fixed_rows_mut::<3>(3).copy_from(&geom);
    fc_sensitivity.fixed_rows_mut::<3>(3).copy_from(&-geom);
    let dp = hpi * fp_sensitivity;
    let dc = hci * fc_sensitivity;
    let dj = solve.solve(&-(gp * dp - gc * dc));
    let dtension = -axis.dot(&dj);
    let denominator = reference - dtension;
    let (mut raw, mut scale) = (1., 1.);
    if p.tension_roll_fraction != 0. || p.tension_yaw_fraction != 0. {
        if reference <= 0. || denominator <= 1e-6 * reference {
            return State::repeat(f64::NAN);
        }
        raw = (base - dtension) / denominator;
        scale = raw.clamp(0.25, 2.);
        cp += (scale - 1.) * dp;
        cc += (scale - 1.) * dc;
        reaction += (scale - 1.) * dj;
    }
    let tension = -axis.dot(&reaction);
    let residual = gp * (cp + hpi * gp.transpose() * reaction)
        + wp.cross(&wp.cross(&arm_p))
        - gc * (cc - hci * gc.transpose() * reaction)
        - wc.cross(&wc.cross(&arm_c));
    LOAD_DIAG.with(|d| d.set([
        tension, reference, scale, raw, denominator / reference, reaction.norm(),
        scale - (tension / reference).clamp(0.25, 2.), residual.norm(),
    ]));
