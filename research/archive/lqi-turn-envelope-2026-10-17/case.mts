/** AI-assisted normal CRUISE stick-path turn envelope experiment. */
import {readFile,writeFile} from 'node:fs/promises';
import {resolve} from 'node:path';
import {pathToFileURL} from 'node:url';
import {HOME,exampleMission} from '../../../viewer/apps/simulator/missions/default';
import {Simulator} from '../../../viewer/packages/sim-browser/src/session/runtime';
import {calmWeather} from '../../../viewer/packages/contracts/src/weather';
import type {PhysicsConfig} from '../../../viewer/packages/sim-browser/src/physics/physics';
Object.defineProperty(globalThis,'crossOriginIsolated',{value:true});
const [specPath,output]=process.argv.slice(2);
const spec=JSON.parse(await readFile(specPath,'utf8'));
const config:PhysicsConfig=JSON.parse(await readFile('assets/runtime/highdof.json','utf8'));
for(const [name,scale] of Object.entries(spec.scales??{})) {
  const i=config.parameter_keys.indexOf(name);if(i<0)throw Error(name);
  config.parameters[i]*=Number(scale);
}
config.brake_rate_mm_s*=spec.brakeRateScale??1;
for(const [k,v] of Object.entries({CP3_RMAX:spec.rate,PG_TURN_RMAX:spec.rate,CP3_HRTC:.7,ARSPD_TYPE:0,ARSPD_USE:0}))config.defaults+=`\n${k} ${v}\n`;
const sim=new Simulator(HOME);sim.truthIntervalMs=0;
let start=Infinity,last=-Infinity,phase='initializing',failure:string|null=null,complete=false;
let latest:number[]|undefined,diagnostic:number[]=[];
const samples:unknown[]=[],events:unknown[]=[],logs:string[]=[];
sim.onLog=s=>{if(s.startsWith('TURNID,'))diagnostic=s.slice(7).split(',').map(Number);else logs.push(s);};
sim.onError=s=>{failure=s;};
sim.onTruth=f=>{
  latest=f.state;const t=f.time_s-start;if(t<0||t-last<.099)return;last=t;
  samples.push({time:t,phase,truth:f.state,wind:f.wind_ned_m_s,rpm:f.rpm,command:[...sim.physics.command],brakes:[...sim.physics.brakes],diagnostic:[...diagnostic],active:sim.state.controllerActive,alt:sim.state.alt,targetAlt:sim.state.targetAlt,roll:sim.state.roll,pitch:sim.state.pitch});
  if(!f.state.every(Number.isFinite))failure='Nonfinite physics';
  if(-f.state[2]<15)failure='Truth altitude below 15 m';
  if(Math.abs(sim.state.roll)>Math.PI/3||Math.abs(sim.state.pitch)>Math.PI/4)failure='Attitude envelope exceeded (60 roll / 45 pitch deg)';
  if(t>5&&sim.state.controllerActive!==(spec.scheme==='lqi'))failure='Unexpected controller ownership';
};
async function save(){await writeFile(output,JSON.stringify({spec,complete,failure,events,samples,logs})+'\n');}
const timeout=setTimeout(async()=>{failure='Wall-clock timeout';await save();process.exit(1);},350000);
const delay=()=>new Promise(r=>setTimeout(r,10));
try {
  await sim.start(pathToFileURL(resolve(spec.runtime,'arduplane.js')).href,config,pathToFileURL(resolve('generated/physics')).href+'/');
  await sim.upload(exampleMission(),()=>{});await sim.launch('AUTO');await sim.speedup(8);
  const launched=sim.physics.time;while(sim.physics.time<launched+30&&!failure)await delay();
  await sim.mode('CRUISE');await sim.controller(spec.scheme);await sim.altitude(60);
  const selected=sim.physics.time;let quiet=selected;
  while(sim.physics.time-selected<180&&!failure){
    if(!latest||Math.abs(-latest[2]-60)>.7||Math.abs(latest[9])>.15)quiet=sim.physics.time;
    if(sim.physics.time-quiet>=8)break;await delay();
  }
  if(sim.physics.time-selected>=180)throw Error('Failed initial altitude settling');
  start=sim.physics.time;phase='settle';
  const stages=[{t:20,name:'turn1',stick:spec.sign},{t:70,name:'reverse',stick:-spec.sign},{t:120,name:'recover1',stick:0},{t:160,name:'turn2',stick:spec.sign},{t:200,name:'recover2',stick:0}];
  let next=0;
  if(spec.wind||spec.turbulence){
    const q=latest!.slice(3,7);const yaw=Math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2]));
    sim.physics.setWind(spec.wind??0,(yaw*180/Math.PI+90+360)%360,0,{...calmWeather,turbulence:spec.turbulence??0,seed:spec.seed});
  }
  while(sim.physics.time-start<250&&!failure){
    const t=sim.physics.time-start;
    if(next<stages.length&&t>=stages[next].t){const s=stages[next++];phase=s.name;sim.controls(s.stick,0,.5);events.push({time:t,...s});}
    await delay();
  }
  complete=!failure;
}catch(e){failure=String(e);}
clearTimeout(timeout);await save();console.log(JSON.stringify({rate:spec.rate,id:spec.id,complete,failure,samples:samples.length}));process.exit(complete?0:1);
