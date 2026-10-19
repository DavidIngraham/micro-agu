use wasm_bindgen::prelude::*;
use serde::Deserialize;
use sim_core::{config::Parameters,dynamics::{rhs_with_loads,BodyLoads},propulsion::*,state::*};
use nalgebra::SVector;
type X=SVector<f64,28>;
#[derive(Deserialize,Clone)]
struct Spec{kind:String,hz:f64,amplitude:f64,ratio:f64,phase:f64,damping:f64,gain:f64}
#[wasm_bindgen]
pub struct Experiment{p:Parameters,mass:f64,time:f64,q:f64,qd:f64,spec:Option<Spec>}
#[wasm_bindgen]
impl Experiment{
 #[wasm_bindgen(constructor)]
 pub fn new(parameters:&[f64],mass:f64)->Result<Experiment,JsValue>{Ok(Self{p:Parameters::from_packed(parameters).map_err(|e|JsValue::from_str(&e))?,mass,time:0.,q:0.,qd:0.,spec:None})}
 pub fn configure(&mut self,s:&str)->Result<(),JsValue>{self.spec=Some(serde_json::from_str(s).map_err(|e|JsValue::from_str(&e.to_string()))?);self.time=0.;self.q=0.;self.qd=0.;Ok(())}
 pub fn modal(&self)->Vec<f64>{vec![self.time,self.q,self.qd]}
 pub fn step(&mut self,state:&[f64],u:&[f64],wind:&[f64],voltage:f64,rpm:f64,dt:f64,substeps:usize)->Result<Vec<f64>,JsValue>{
 let mut x=X::zeros();x.fixed_rows_mut::<26>(0).copy_from(&State::from_column_slice(state));x[26]=self.q;x[27]=self.qd;
 let spec=self.spec.as_ref().unwrap();let h=dt/substeps as f64;let w=2.*std::f64::consts::PI*spec.hz;let mut r=rpm;let target=target_rpm(u[0],voltage);let wind=V::from_column_slice(wind);
 for j in 0..substeps{
  let f=|z:&X,local:f64|{let y=State::from_column_slice(&z.as_slice()[..26]);let rp=rotation(&y,0);let rc=rotation(&y,13);let t=self.time+j as f64*h+local;let rr=target+(r-target)*(-local/MOTOR_TAU_S).exp();let axial=(rp.transpose()*(vector(&y,7)-wind)).x;
   let ramp=((t-10.)/2.).clamp(0.,1.)*((50.-t)/2.).clamp(0.,1.);let mut moment=V::zeros();let mut canopy=V::zeros();let mut drive=0.;
   if spec.kind=="forcing"{moment=V::new(0.,spec.amplitude*spec.ratio*(w*t+spec.phase).sin(),spec.amplitude*(w*t).sin())*ramp;}
   if spec.kind=="mode" && t>=10.{let rel=rp.transpose()*(vector(&y,23)-vector(&y,10));drive=rel.z/w+0.2*(u[2]-u[1])/0.072;moment=V::new(0.,spec.ratio*spec.gain*z[26],spec.gain*z[26]);canopy=-(rp*moment);if t<12.{moment.z+=spec.amplitude*(w*t).sin();}}
   let loads=BodyLoads{payload_moment:rp*moment,canopy_moment:canopy,..BodyLoads::default()};
   let dy=rhs_with_loads(&y,&[thrust(rr,axial)/self.p.thrust_max,u[1],u[2]],&self.p,self.mass,wind,rr,(target-rr)/MOTOR_TAU_S,Some(torque(rr,axial)),&loads,true);
   let mut out=X::zeros();out.fixed_rows_mut::<26>(0).copy_from(&dy);out[26]=z[27];out[27]=w*w*(drive-z[26])-2.*spec.damping*w*z[27];out};
  let a=f(&x,0.);let b=f(&(x+a*h*0.5),h*0.5);let c=f(&(x+b*h*0.5),h*0.5);let d=f(&(x+c*h),h);x+=h/6.*(a+2.*b+2.*c+d);let mut y=State::from_column_slice(&x.as_slice()[..26]);normalize(&mut y);validate(&y).map_err(|e|JsValue::from_str(&e))?;x.fixed_rows_mut::<26>(0).copy_from(&y);r=target+(r-target)*(-h/MOTOR_TAU_S).exp();
 }
 self.time+=dt;self.q=x[26];self.qd=x[27];let mut out=x.as_slice()[..26].to_vec();out.push(r);Ok(out)
 }
}
