/** Real packaged AP, EKF and Rust physics. No truth inputs to controller. */
import {readFile,writeFile} from 'node:fs/promises';
import {resolve} from 'node:path';
import {pathToFileURL} from 'node:url';
import {HOME,exampleMission} from '../../../viewer/apps/simulator/missions/default';
import {Simulator} from '../../../viewer/packages/sim-browser/src/session/runtime';
import {calmWeather} from '../../../viewer/packages/contracts/src/weather';
Object.defineProperty(globalThis,'crossOriginIsolated',{value:true});
const [specPath,output]=process.argv.slice(2), spec=JSON.parse(await readFile(specPath,'utf8'));
const config=JSON.parse(await readFile(new URL('envelope-physics.json',import.meta.url),'utf8'));
config.voltage=spec.voltage??22.2;
for(const [name,scale] of Object.entries(spec.scales??{})) {const i=config.parameter_keys.indexOf(name);if(i<0)throw Error(name);config.parameters[i]*=Number(scale);}
config.brake_rate_mm_s*=spec.slew??1;
for(const [k,v] of Object.entries({CP3_HUP:spec.up??spec.limit,CP3_HDN:spec.down??spec.limit,CP3_HRTC:.7,TECS_CLMB_MAX:spec.upDemand??spec.demand,TECS_SINK_MAX:spec.downDemand??spec.demand,ARSPD_TYPE:0,ARSPD_USE:0,...spec.parameters}))config.defaults+='\n'+k+' '+v+'\n';
const sim=new Simulator(HOME);sim.truthIntervalMs=0;
let start=Infinity,last=-Infinity,phase='settle',failure:string|null=null,complete=false;
let expectedActive=spec.scheme==='lqi', ownershipAfter=5;
let initLast=-Infinity;
const initialization:any[]=[];
const samples:any[]=[],logs:string[]=[],events:any[]=[];
sim.onLog=s=>logs.push(s);sim.onError=s=>{failure=s};
sim.onTruth=f=>{
 const t=f.time_s-start;if(t<0){if(f.time_s-initLast>=1){initLast=f.time_s;initialization.push({time:f.time_s,height:-f.state[2],vertical:-f.state[9],alt:sim.state.alt,pitch:sim.state.pitch,command:[...sim.physics.command]});}return;}if(t-last<.099)return;last=t;
 samples.push({time:t,phase,truth:f.state,wind:f.wind_ned_m_s,rpm:f.rpm,command:[...sim.physics.command],brakes:[...sim.physics.brakes],active:sim.state.controllerActive,alt:sim.state.alt,targetAlt:sim.state.targetAlt,roll:sim.state.roll,pitch:sim.state.pitch});
 if(-f.state[2]<15)failure='Height below 15 m';
 if(Math.abs(sim.state.roll)>Math.PI/3||Math.abs(sim.state.pitch)>Math.PI/4)failure='60/45 deg roll/pitch guard';
 if(t>ownershipAfter&&sim.state.controllerActive!==expectedActive)failure='Controller ownership';
};
const delay=()=>new Promise(r=>setTimeout(r,10));
const save=()=>writeFile(output,JSON.stringify({spec,complete,failure,samples,events,logs,initialization,finalState:sim.physics?.state,finalTelemetry:sim.state})+'\n');
const timeout=setTimeout(async()=>{failure='Timeout';await save();process.exit(1)},240000);
try {
 await sim.start(pathToFileURL(resolve(spec.runtime??new URL('./runtime-original/',import.meta.url).pathname,'arduplane.js')).href,config,pathToFileURL(resolve('generated/physics')).href+'/',new Uint8Array(await readFile(spec.gainFile)));
 await sim.upload(exampleMission(),()=>{});await sim.launch('AUTO');await sim.speedup(8);
 await sim.mode('CRUISE');await sim.controller(spec.scheme);await sim.altitude(80);
 const selected=sim.physics.time;let quiet=selected;
 while(sim.physics.time-selected<220&&!failure){
  if(Math.abs(-sim.physics.state[2]-80)>.7||Math.abs(sim.physics.state[9])>.15)quiet=sim.physics.time;
  if(sim.physics.time-quiet>5)break;await delay();
 }
 if(sim.physics.time-selected>=220)throw Error('Initial settling failed');
 const weather={...calmWeather,turbulenceModel:'dryden' as const,gustLength:20,verticalRatio:.5,turbulence:spec.turbulence??0,seed:spec.seed??73,thermalStrength:spec.thermal??0,thermalRadius:20,thermalNorth:sim.physics.state[0]+40,thermalEast:sim.physics.state[1]};
 sim.physics.setWind(spec.wind??0,270,0,weather);
 start=sim.physics.time;
 const stages=[{t:10,name:'climb',lever:1},{t:35,name:'hold_high',lever:.5},{t:50,name:'descend',lever:0},{t:75,name:'hold_low',lever:.5},{t:90,name:'up_reversal',lever:1},{t:100,name:'down_reversal',lever:0},{t:110,name:'recover',lever:.5}];
 let next=0, transition=0;
 const switches=[{t:115,mode:'CRUISE',scheme:'baseline'},{t:125,mode:'CRUISE',scheme:spec.scheme},{t:135,mode:'FBWA',scheme:spec.scheme},{t:140,mode:'CRUISE',scheme:spec.scheme}];
 sim.controls((spec.turn??0)/35,0,.5);
 while(sim.physics.time-start<155&&!failure){
  const t=sim.physics.time-start;
  if(spec.voltageStep&&t>40)sim.physics.config.voltage=t<110?spec.voltageStep:22.2;
  if(next<stages.length&&t>=stages[next].t){const s=stages[next++];phase=s.name;events.push({time:t,...s});
   sim.controls(s.name==='recover'?0:(spec.turn??0)/35,0,s.lever);
   if(s.name==='recover'){sim.physics.setWind(0,0,0,{...weather,turbulence:0,thermalStrength:0});await sim.altitude(80);}
  }
  if(spec.transitions&&transition<switches.length&&t>=switches[transition].t){
   const change=switches[transition++];ownershipAfter=t+5;expectedActive=change.scheme==='lqi'&&change.mode!=='FBWA';
   events.push({time:t,...change});phase='transition-'+change.mode;
   await sim.controller(change.scheme);await sim.mode(change.mode);sim.controls(0,0,change.mode==='FBWA'?.45:.5);
   if(change.mode==='CRUISE')await sim.altitude(80);
  }
  await delay();
 }
 complete=!failure;
}catch(e){failure=String(e)}
clearTimeout(timeout);await save();console.log(JSON.stringify({id:spec.id,complete,failure}));process.exit(complete?0:1);
