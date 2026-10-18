//! Regenerate the explanatory applet data from the same panel kernel as the plant.
//! cargo run -p sim-io --example canopy_panels -- data/presets/as-built-trout-lake-sitl.json
use serde_json::json;
use sim_core::{config::Parameters, dynamics::panel_loads, state::V};
fn vector(v: V) -> [f64; 3] {
    [v.x, v.y, v.z]
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args().nth(1).expect("provide a SITL preset path");
    let input: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(&path)?)?;
    let p: Parameters = serde_json::from_value(input["config"]["parameters"].clone())?;
    let speed = 7.0_f64;
    let alpha = 0.08_f64;
    let velocity = V::new(speed * alpha.cos(), 0., speed * alpha.sin());
    let rates = V::zeros();
    let cases: Vec<_> = [
        ("symmetric", "Symmetric flight", 0.010, 0.010),
        ("differential", "Right brake", 0.0, 0.030),
    ]
    .into_iter()
    .map(|(id, label, left, right)| {
        let loads = panel_loads(velocity, rates, left / 0.072, right / 0.072, &p);
        let force: V = loads.iter().map(|l| l.force).sum();
        let moment: V = loads.iter().map(|l| l.position.cross(&l.force)).sum();
        // At this zero-sideslip, zero-angular-rate condition, the non-panel
        // aerodynamic moment formula reduces to these three terms.
        let q = 0.5 * p.rho * speed * speed;
        let diff = (right - left) / 0.072;
        let empirical = q
            * p.s
            * V::new(
                p.b * p.cldelta * diff,
                p.c * (p.cm0 + p.cma * alpha),
                p.b * p.cndelta * diff,
            );
        json!({"id":id,"label":label,"left_m":left,"right_m":right,
            "force":vector(force),"panelMoment":vector(moment),"additionalMoment":vector(empirical),
            "totalMoment":vector(moment+empirical),
            "panels":loads.iter().map(|l| json!({"position":vector(l.position),
                "inclination":l.inclination,"localVelocity":vector(l.local_velocity),
                "brake_m":l.brake*0.072,"alpha":l.alpha,"lift":vector(l.lift),
                "drag":vector(l.drag),"force":vector(l.force)})).collect::<Vec<_>>()})
    })
    .collect();
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({"sourcePreset":path,
        "speed":speed,"alpha":alpha,"angularRate":vector(rates),"area":p.s,
        "brakeTiltRadPerM":p.brake_tilt_rad_per_m,"arcSpan":p.b,"halfArc":p.canopy_half_arc,"chord":p.c,"cases":cases}))?
    );
    Ok(())
}
