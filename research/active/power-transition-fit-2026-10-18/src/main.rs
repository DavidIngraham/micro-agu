use serde::Deserialize;
use sim_core::{config::Parameters,dynamics::rhs,propulsion::{thrust,torque},state::*};
use std::io::{self,BufRead};
#[derive(Deserialize)]
struct Request {parameters:Vec<f64>,mass:f64,state:Vec<f64>,commands:Vec<[f64;3]>,rpm:Vec<f64>,wind:[f64;3],substeps:usize,power_damping:f64}
fn run(r:Request)->Result<Vec<Vec<f64>>,String>{
 let p=Parameters::from_packed(&r.parameters)?;
 if r.state.len()!=26 || r.rpm.len()!=r.commands.len() || r.substeps==0{return Err("shape".into())}
 let mut y=State::from_column_slice(&r.state);let wind=V::from_column_slice(&r.wind);let mut out=vec![y.as_slice().to_vec()];let h=0.04/r.substeps as f64;
 for i in 0..r.commands.len()-1 {
  let dr=(r.rpm[i+1]-r.rpm[i])/0.04;
  for j in 0..r.substeps{
   let f=|z:&State,t:f64| {let rpm=r.rpm[i]+dr*(j as f64*h+t);let axial=(rotation(z,0).transpose()*(vector(z,7)-wind)).x;let mut pp=p.clone();pp.joint_c_yaw*=1.+r.power_damping*(rpm/8000.).powi(2);rhs(z,&[thrust(rpm,axial)/p.thrust_max,r.commands[i][1],r.commands[i][2]],&pp,r.mass,wind,rpm,dr,Some(torque(rpm,axial)))};
   let a=f(&y,0.);let b=f(&(y+a*h*0.5),h*0.5);let c=f(&(y+b*h*0.5),h*0.5);let d=f(&(y+c*h),h);y+=h/6.*(a+2.*b+2.*c+d);normalize(&mut y);validate(&y)?;
  }
  out.push(y.as_slice().to_vec());
 }
 Ok(out)
}
fn main(){for line in io::stdin().lock().lines(){let result=line.map_err(|e|e.to_string()).and_then(|s|serde_json::from_str::<Request>(&s).map_err(|e|e.to_string())).and_then(run);println!("{}",serde_json::to_string(&result).unwrap());}}
