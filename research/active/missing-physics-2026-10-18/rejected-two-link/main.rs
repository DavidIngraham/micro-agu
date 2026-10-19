
use sim_core::{config::Parameters,dynamics::{self,BodyLoads},propulsion::*,state::*};
use serde::Deserialize;
use std::io::{self,BufRead};

#[derive(Clone,Deserialize,Default)]
struct Experiment {
 #[serde(default)] prop_normal:f64,
 #[serde(default)] drag_center:[f64;3],
 #[serde(default)] suspension_k:f64,
 #[serde(default)] suspension_c:f64,
 #[serde(default)] rest_lengths:[f64;2],
}
#[derive(Deserialize)]
struct Request {
 parameters:Vec<f64>,mass:f64,state:Vec<f64>,commands:Vec<[f64;3]>,wind:[f64;3],voltage:f64,rpm:f64,
 #[serde(default)] experiment:Experiment,
 #[serde(default)] derivative:bool,
 #[serde(default="dt")] dt:f64,
 #[serde(default="steps")] substeps:usize,
}
fn dt()->f64{0.04}
fn steps()->usize{32}
fn endpoints(y:&State,p:&Parameters,side:f64)->(V,V,V,V){
 let rp=rotation(y,0);let rc=rotation(y,13);
 let ap=rp*V::new(p.hinge_x,p.hinge_y+side*p.riser_halfspacing,p.hinge_z);
 let ac=rc*V::new(0.,side*p.support_halfspacing,0.);
 (vector(y,0)+ap,vector(y,13)+ac,ap,ac)
}
fn routing(y:&State,p:&Parameters,u:&[f64;3])->[f64;3]{
 let rp=rotation(y,0);let rc=rotation(y,13);
 let hinge=vector(y,0)+rp*V::new(p.hinge_x,p.hinge_y,p.hinge_z);
 let mut result=*u;
 for (i,side) in [-1.,1.].iter().enumerate(){
  let (a,b,_,_)=endpoints(y,p,*side);
  let guide=hinge+rp*V::new(p.guide_x,side*p.guide_halfspacing,0.);
  let ring=a+p.ring_height*(b-a).normalize();
  let cascade=b+rc*V::new(p.cascade_x,0.,0.);
  result[i+1]=(u[i+1]+(guide-ring).norm()+(ring-cascade).norm()-[p.route_reference_left,p.route_reference_right][i]).max(0.);
 }
 result
}
fn loads(y:&State,p:&Parameters,w:V,rpm:f64,e:&Experiment)->(BodyLoads,[f64;2]){
 let rp=rotation(y,0);let vp=vector(y,7)-w;let wp=vector(y,10);
 let mut l=BodyLoads::default();
 // Replace CG drag by the same drag law evaluated at a body-fixed center.
 let arm=rp*V::from(e.drag_center);
 let flow=vp+wp.cross(&arm);
 let drag=-0.5*p.rho*p.payload_cda*flow.norm()*flow;
 let old=-0.5*p.rho*p.payload_cda*vp.norm()*vp;
 l.payload_force+=drag-old;l.payload_moment+=arm.cross(&drag);
 // Dissipative propeller disk-normal (in-plane) force; no intrinsic disk moment.
 let prop_arm=rp*V::new(p.thrust_x,p.thrust_y,p.thrust_arm_z);
 let prop_flow=rp.transpose()*(vp+wp.cross(&prop_arm));
 let t=thrust(rpm,prop_flow.x).max(0.);
 let induced=(t/(2.*p.rho*std::f64::consts::PI*0.1143_f64.powi(2))).sqrt();
 let denom=(prop_flow.x.powi(2)+induced.powi(2)+1e-6).sqrt();
 let normal=rp*(-e.prop_normal*t/denom*V::new(0.,prop_flow.y,prop_flow.z));
 l.payload_force+=normal;l.payload_moment+=prop_arm.cross(&normal);
 let mut tensions=[0.;2];
 if e.suspension_k>0. {
  for (i,side) in [-1.,1.].iter().enumerate(){
   let (a,b,ap,ac)=endpoints(y,p,*side);let dist=(b-a).norm();let n=(b-a)/dist;
   let rel=vector(y,20)+vector(y,23).cross(&ac)-vector(y,7)-wp.cross(&ap);
   let extension=dist-e.rest_lengths[i];
   let tension=if extension>0. {(e.suspension_k*extension+e.suspension_c*rel.dot(&n)).max(0.)}else{0.};
   let f=tension*n;tensions[i]=tension;
   l.payload_force+=f;l.canopy_force-=f;
   l.payload_moment+=ap.cross(&f);l.canopy_moment-=ac.cross(&f);
  }
 }
 (l,tensions)
}
fn derivative(y:&State,u:&[f64;3],p:&Parameters,mass:f64,w:V,rpm:f64,rate:f64,e:&Experiment)->State{
 let (l,_)=loads(y,p,w,rpm,e);let mut model=p.clone();let mut applied=*u;
 if e.suspension_k>0. {
  // Explicit suspension replaces the point support and effective spring;
  // keep the prior rotational dissipative terms as an explicit uncertainty.
  model.geometry_yaw=0.;model.joint_k_roll=0.;model.joint_k_pitch=0.;model.joint_k_yaw=0.;
  applied=routing(y,p,u);model.routing_enabled=0.;
 }
 let axial=(rotation(y,0).transpose()*(vector(y,7)-w)).x;
 applied[0]=thrust(rpm,axial)/p.thrust_max;
 dynamics::rhs_with_loads(y,&applied,&model,mass,w,rpm,rate,Some(torque(rpm,axial)),&l,e.suspension_k==0.)
}
fn run(r:Request)->Result<serde_json::Value,String>{
 let p=Parameters::from_packed(&r.parameters)?;p.validate(r.mass)?;
 if r.state.len()!=26 || r.commands.is_empty() || r.substeps==0 || r.substeps>10000 || r.dt<=0. || r.dt>1.
 || r.experiment.prop_normal<0. || r.experiment.suspension_k<0. || r.experiment.suspension_c<0.
 || r.voltage<=0. || r.rpm<0. {return Err("invalid input".into());}
 if r.experiment.suspension_k>0. && r.experiment.rest_lengths.iter().any(|x|*x<=0.) {return Err("positive rest lengths required".into());}
 let mut y=State::from_column_slice(&r.state);validate(&y)?;let w=V::from(r.wind);let mut rpm=r.rpm;
 if r.derivative {
  let dy=derivative(&y,&r.commands[0],&p,r.mass,w,rpm,0.,&r.experiment);
  return Ok(serde_json::json!({"derivative":dy.as_slice()}));
 }
 let mut rows=Vec::new();
 for (i,u) in r.commands.iter().enumerate(){
  let (l,t)=loads(&y,&p,w,rpm,&r.experiment);
  let mut row=y.as_slice().to_vec();row.push(rpm);row.extend(t);
  row.extend(l.payload_moment.as_slice());rows.push(row);
  if i+1==r.commands.len(){break;}
  let h=r.dt/r.substeps as f64;
  for _ in 0..r.substeps{
   let target=target_rpm(u[0],r.voltage);
   let f=|z:&State,time:f64|{
    let speed=target+(rpm-target)*(-time/MOTOR_TAU_S).exp();
    derivative(z,u,&p,r.mass,w,speed,(target-speed)/MOTOR_TAU_S,&r.experiment)
   };
   let a=f(&y,0.);let b=f(&(y+0.5*h*a),h*0.5);let c=f(&(y+0.5*h*b),h*0.5);let d=f(&(y+h*c),h);
   y+=h/6.*(a+2.*b+2.*c+d);normalize(&mut y);validate(&y)?;
   rpm=target+(rpm-target)*(-h/MOTOR_TAU_S).exp();
  }
 }
 Ok(serde_json::json!({"rows":rows}))
}
fn main(){
 for line in io::stdin().lock().lines(){
  let result=line.map_err(|e|e.to_string()).and_then(|s|serde_json::from_str::<Request>(&s).map_err(|e|e.to_string())).and_then(run);
  println!("{}",match result{Ok(v)=>v,Err(e)=>serde_json::json!({"error":e})});
 }
}
#[cfg(test)]mod tests{
 use super::*;
 fn setup()->(State,Parameters){
  let mut y=State::zeros();y[6]=1.;y[19]=1.;y[15]=-0.9;
  let mut p=Parameters::default();p.hinge_z=-0.05;p.riser_halfspacing=0.066;p.support_halfspacing=0.32;
  (y,p)
 }
 #[test]fn suspension_conserves_force_and_torque(){
  let(y,p)=setup();let e=Experiment{suspension_k:1000.,rest_lengths:[0.8,0.8],..Default::default()};
  let(l,t)=loads(&y,&p,V::zeros(),0.,&e);assert!(t[0]>0.);
  assert!((l.payload_force+l.canopy_force).norm()<1e-12);
  let total=l.payload_moment+vector(&y,0).cross(&l.payload_force)+l.canopy_moment+vector(&y,13).cross(&l.canopy_force);
  assert!(total.norm()<1e-10);
 }
 #[test]fn slack_has_no_suspension_force(){
  let(y,p)=setup();let e=Experiment{suspension_k:1000.,rest_lengths:[1.5,1.5],..Default::default()};
  let(l,t)=loads(&y,&p,V::zeros(),0.,&e);assert_eq!(t,[0.,0.]);assert_eq!(l.payload_force,V::zeros());
 }
 #[test]fn prop_force_opposes_transverse_flow_and_disappears_at_zero_rpm(){
  let(mut y,p)=setup();y[7]=4.;y[8]=1.;y[9]=-0.5;
  let e=Experiment{prop_normal:0.3,..Default::default()};
  let(l,_)=loads(&y,&p,V::zeros(),8000.,&e);
  assert!(l.payload_force.dot(&vector(&y,7))<0.);assert!(l.payload_force.x.abs()<1e-12);
  assert!(loads(&y,&p,V::zeros(),0.,&e).0.payload_force.norm()<1e-12);
 }
 #[test]fn offset_drag_total_power_is_dissipative(){
  let(mut y,p)=setup();y[7]=4.;y[8]=0.5;y[11]=1.;
  let e=Experiment{drag_center:[0.05,0.,-0.05],..Default::default()};
  let(l,_)=loads(&y,&p,V::zeros(),0.,&e);
  let v=vector(&y,7);let original=-0.5*p.rho*p.payload_cda*v.norm()*v;
  assert!((l.payload_force+original).dot(&v)+l.payload_moment.dot(&vector(&y,10))<0.);
 }
}
