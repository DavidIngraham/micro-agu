import {readFile,writeFile} from 'node:fs/promises';
import {Simulator} from '../../../viewer/packages/sim-browser/src/session/runtime';
import {HOME} from '../../../viewer/apps/simulator/missions/default';
import {calmWeather} from '../../../viewer/packages/contracts/src/weather';
import {defaultLidar} from '../../../viewer/packages/sim-browser/src/physics/lidar';
Object.defineProperty(globalThis,'crossOriginIsolated',{value:true});
const [input,output]=process.argv.slice(2),spec=JSON.parse(await readFile(input,'utf8'));
const sim=new Simulator(HOME),samples:any[]=[],logs:any[]=[];
let start=Infinity,last=-Infinity,controlTime=-Infinity,failure:string|null=null;
let target:any=null,environment:any=null,contact:any=null,previous:any=null;
let handoff:number|null=null;
let course:number|undefined;
let pitchRate=0;
let lidarRange:number|null=null,lidarTime=-Infinity,lastGoodTime=-Infinity,lastGoodHeight=0;
const lidarPackets:any[]=[],haglPackets:any[]=[];
let hagl:number|null=null,haglTime=-Infinity;
(sim as any).listeners.add((m:any)=>{
 if(m.name==='GLOBAL_POSITION_INT'&&Math.hypot(m.values.vx,m.values.vy)>50)
   course=Math.atan2(m.values.vy,m.values.vx);
 if(m.name==='DISTANCE_SENSOR'&&Number(m.values.orientation)===25) {
   const v=m.values;lidarTime=sim.physics.time;
   lidarRange=(v.current_distance>=v.min_distance&&v.current_distance<=v.max_distance)?v.current_distance/100:null;
   lidarPackets.push({time:lidarTime,values:{...v}});
 }
 if(m.name==='NAMED_VALUE_FLOAT'&&m.values.name==='EKF_HAGL') {
   hagl=Number(m.values.value);haglTime=sim.physics.time;
   haglPackets.push({time:haglTime,height:hagl,stamp:m.values.time_boot_ms});
 }
 if(m.name==='ATTITUDE')pitchRate=Number(m.values.pitchspeed);
});
sim.truthIntervalMs=0;sim.onLog=text=>logs.push({time:sim.physics?.time,text});
sim.onError=text=>failure=text;
sim.onTruth=f=>{
 if(f.time_s<start||failure)return;
 const time=f.time_s-start;
 const sample={time,state:[...f.state],mode:sim.state.mode,active:sim.state.controllerActive,
   command:[...sim.physics.command],brakes:[...sim.physics.brakes],guidance:{flare:['flare','touchdown','landed'].includes(sim.state.landing?.phase??'')},
   telemetry:structuredClone(sim.state),lidar:{range:lidarRange,received:lidarTime,generated:sim.physics.lidarReading,hagl,haglTime}};
 if(!contact&&(-f.state[2]<=.12||-f.state[15]<=.05))contact={incoming:previous??sample,current:sample};
 if(time-last>=.049||contact){samples.push(sample);last=time}
 previous=sample;
 if(!contact&&(Math.abs(sim.state.roll)>Math.PI/3||Math.abs(sim.state.pitch)>Math.PI/3))failure='Attitude guard';
 if(sim.state.mode===0&&handoff===null)handoff=time;
 if(sim.state.landing?.phase==='aborted')failure=sim.state.landing.message;

};
const delay=()=>new Promise(r=>setTimeout(r,10));
const save=()=>writeFile(output,JSON.stringify({spec,failure,complete:!!contact&&!failure,
 target,environment,handoff,samples,contact,logs,lidarPackets,haglPackets,finalLanding:sim.state.landing})+'\n');
process.on('unhandledRejection',async e=>{failure=String(e);await save();sim.dispose();process.exit(1)});
const timer=setTimeout(async()=>{failure='Wall timeout';await save();process.exit(1)},360000);
try {
 const c=JSON.parse(await readFile(new URL('../../../aircraft/microagu/configurations/browser-physics.json',import.meta.url),'utf8'));
 c.lidar={...defaultLidar,...spec.lidar};
 c.defaults+='\nRC1_DZ 0\nRC2_DZ 0\nRNGFND1_TYPE 100\nRNGFND1_ORIENT 25\nRNGFND1_MIN 0.1\nRNGFND1_MAX 40\n';
 await sim.start(new URL('../../../viewer/generated/ardupilot/arduplane.js',import.meta.url).href,
 c,new URL('../../../viewer/generated/physics/',import.meta.url).href,
 new Uint8Array(await readFile(new URL('../../../aircraft/microagu/controllers/lqi-gains.bin',import.meta.url))));
 await sim.command(511,[132,50000]);
 await sim.controller('baseline');await sim.speedup(8);await sim.launch('CRUISE');await sim.altitude(60);
 const climb=sim.physics.time;let quiet=climb;
 while(sim.physics.time-climb<240) {
   if(Math.abs(-sim.physics.state[2]-60)>1||Math.abs(sim.physics.state[9])>.3)quiet=sim.physics.time;
   if(sim.physics.time-quiet>3)break;await delay();
 }
 if(sim.physics.time-climb>=240)throw Error('Settling timeout');
 await sim.controller(spec.scheme??'lqi');
 const selected=sim.physics.time;while(sim.physics.time-selected<20)await delay();
 const heading=Math.atan2(sim.physics.state[8],sim.physics.state[7]);
 const dn=Math.cos(heading),de=Math.sin(heading);
 const n=(spec.along??0)*dn-(spec.across??0)*de,e=(spec.along??0)*de+(spec.across??0)*dn;
 sim.physics.setWind(Math.hypot(n,e),(Math.atan2(-e,-n)*180/Math.PI+360)%360,0,
 {...calmWeather,turbulence:spec.turbulence??0,seed:spec.seed??73});
 const weather=sim.physics.time;while(sim.physics.time-weather<15)await delay();
 environment={windNED:[n,e,0],initial:[...sim.physics.state]};
 await sim.land();
 target={...sim.state.landing!.target,heading};start=sim.physics.time;
 while(sim.physics.time-start<180&&!failure) {
   if(sim.state.simulationEnded)break;
   if(contact&&sim.physics.time-start-contact.current.time>8){failure="No landing confirmation after contact";break;}
   await delay();
 }
 if(!contact&&!failure)failure='Landing timeout';
}catch(e){failure=String(e)}
clearTimeout(timer);sim.onTruth=()=>{};sim.onError=()=>{};await save();sim.dispose();
console.log(JSON.stringify({id:spec.id,complete:!!contact&&!failure,failure}));process.exit(failure?1:0);
