/** AI-assisted research sensor and full linearization-chart truth diagnostic. */
import { Quaternion, Vector3 } from "../../../viewer/packages/contracts/src/math";
import { makeSensor as previous } from "../proprioception-2026-10-17/sensor";
import trim from "./trim.json";

export function chart(state: number[], rpm: number, wind: number[]) {
  const p=new Quaternion().fromArray(state,3).normalize();
  const c=new Quaternion().fromArray(state,16).normalize();
  const rel=new Quaternion().fromArray(trim.relative).invert().multiply(p.clone().invert().multiply(c));
  if(rel.w<0)rel.fromArray([-rel.x,-rel.y,-rel.z,-rel.w]);
  const length=Math.hypot(rel.x,rel.y,rel.z);
  const factor=length>1e-12?2*Math.atan2(length,rel.w)/length:2;
  const roll=Math.atan2(2*(p.w*p.x+p.y*p.z),1-2*(p.x*p.x+p.y*p.y));
  const pitch=Math.asin(Math.max(-1,Math.min(1,2*(p.w*p.y-p.z*p.x))));
  const body=(v:number[],q:Quaternion)=>new Vector3(...v as [number,number,number]).applyQuaternion(q.clone().invert()).toArray();
  const sub=(a:number[],b:number[])=>a.map((v,i)=>v-b[i]);
  const x=[roll-trim.euler[0],pitch-trim.euler[1],rel.x*factor,rel.y*factor,rel.z*factor,
    ...sub(body(sub(state.slice(7,10),wind),p),trim.velocity),
    ...sub(body(state.slice(10,13),p),trim.payloadRates),
    ...sub(body(state.slice(23,26),c),trim.canopyRates),(rpm-trim.rpm)/1000,-state[2]];
  return x.map((v,i)=>v/trim.scale[i]);
}

export function makeSensor(kind:string,seed:number,physics:{rpm:number}) {
  if(!['ideal','nodelay','truth'].includes(kind))return previous(kind,seed);
  const sensor=previous(kind==='ideal'?'angles':'sensor',seed);
  // nodelay reuses identical noise generator but compensates the queue time by
  // using a dedicated zero-delay sensor below, rather than shifting timestamps.
  const clean=previous('angles',seed);
  let random=(seed+93719)>>>0;
  const u=()=>{random^=random<<13;random^=random>>>17;random^=random<<5;return ((random>>>0)+1)/4294967297;};
  const normal=()=>Math.sqrt(-2*Math.log(u()))*Math.cos(2*Math.PI*u());
  let next=-Infinity,last:ReturnType<typeof clean>;
  return (state:number[],time:number)=>{
    let result=sensor(state,time);
    if(kind==='ideal')return {...result,microagu_mode:3};
    if(kind==='nodelay'){
      if(time>=next){
        last=clean(state,time);
        const q=new Quaternion(last.microagu_relative[1],last.microagu_relative[2],last.microagu_relative[3],last.microagu_relative[0]);
        const a=[normal(),normal(),normal()].map(v=>v*.5*Math.PI/180),m=Math.hypot(...a),s=m?Math.sin(m/2)/m:0;
        q.multiply(new Quaternion(a[0]*s,a[1]*s,a[2]*s,Math.cos(m/2)));
        last={...last,microagu_relative:[q.w,q.x,q.y,q.z],microagu_mode:3};next=time+.02-1e-8;
      }
      return last;
    }
    const wind=(physics as unknown as {windNED:number[]}).windNED;
    const x=chart(state,physics.rpm,wind);
    const channels:Record<string,unknown>={...result,microagu_mode:4};
    for(let i=0;i<6;i++)channels[`microagu_truth${i}`]=[0,1,2].map(j=>x[3*i+j]??0);
    return channels;
  };
}
