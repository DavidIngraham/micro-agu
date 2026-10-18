//! Recorded prescribed motion and yaw geometry for the joint explanation applet.
//! This is a torque evaluation, not a free-response simulation.
use serde_json::json;
use sim_core::{
    joint::joint_loads,
    state::{M, V},
    Parameters,
};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args().nth(1).expect("SITL preset path");
    let input: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(&path)?)?;
    let p: Parameters = serde_json::from_value(input["config"]["parameters"].clone())?;
    let mp = input["config"]["mass_kg"].as_f64().unwrap() - p.canopy_mass;
    let mut axes = Vec::new();
    let amplitude = 15_f64.to_radians();
    let period = 6.;
    for (axis, name, k, c) in [
        (0, "Roll", p.joint_k_roll, p.joint_c_roll),
        (1, "Pitch", p.joint_k_pitch, p.joint_c_pitch),
        (2, "Yaw", p.joint_k_yaw, p.joint_c_yaw),
    ] {
        let samples:Vec<_>=(0..=360).map(|i|{
            let phase=i as f64*std::f64::consts::TAU/360.;
            let angle=amplitude*phase.cos();let rate=-amplitude*std::f64::consts::TAU/period*phase.sin();
            let mut e=V::zeros();e[axis]=angle;
            let skew=sim_core::state::skew(e);
            let theta=e.norm();
            let rc=if theta<1e-10 {M::identity()}else{M::identity()+skew*(theta.sin()/theta)+skew*skew*((1.-theta.cos())/(theta*theta))};
            let mut wc=V::zeros();wc[axis]=rate;
            let load=joint_loads(M::identity(),rc,V::zeros(),wc,mp,&p).unwrap();
            json!({"angle":angle,"rate":rate,"spring":-load.spring[axis],"damper":-load.damping[axis],"total":-load.total[axis]})
        }).collect();
        axes.push(json!({"name":name,"stiffness":k,"damping":c,"samples":samples}));
    }
    let mut geometry = p.clone();
    geometry.geometry_yaw = 1.;
    let a = p.yaw_payload_halfspacing;
    let b = p.yaw_canopy_halfspacing;
    let h = p.yaw_vertical_separation;
    let yaw:Vec<_>=(-60..=60).map(|deg|{
        let angle=(deg as f64).to_radians();let (sn,cs)=angle.sin_cos();
        let rc=M::new(cs,-sn,0.,sn,cs,0.,0.,0.,1.);
        let load=joint_loads(M::identity(),rc,V::zeros(),V::zeros(),mp,&geometry).unwrap();
        json!({"degrees":deg,"height":(h*h-2.*a*b*(1.-cs)).sqrt(),"torque":-load.spring.z,"linearTorque":-p.joint_k_yaw*angle})
    }).collect();
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({"sourcePreset":path,"payloadMass":mp,
        "geometryEnabled":p.geometry_yaw>0.,"a":a,"b":b,"height":h,"period":period,"amplitude":amplitude,
        "geometryStiffness":mp*9.80665*a*b/h,"axes":axes,"yaw":yaw}))?
    );
    Ok(())
}
