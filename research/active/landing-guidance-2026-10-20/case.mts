import {readFile,writeFile} from 'node:fs/promises';
import {Simulator} from '../../../viewer/packages/sim-browser/src/session/runtime';
import {HOME} from '../../../viewer/apps/simulator/missions/default';
import {calmWeather} from '../../../viewer/packages/contracts/src/weather';
import {LandingGuidance} from './guidance';
Object.defineProperty(globalThis,'crossOriginIsolated',{value:true});
const [input,output]=process.argv.slice(2),spec=JSON.parse(await readFile(input,'utf8'));
const sim=new Simulator(HOME),samples:any[]=[],logs:any[]=[];
let start=Infinity,last=-Infinity,controlTime=-Infinity,failure:string|null=null;
let target:any=null,environment:any=null,contact:any=null,previous:any=null;
let guidance:LandingGuidance|null=null,requestManual=false,handoff:number|null=null,command:any=null;
let course:number|undefined;
let pitchRate=0;
(sim as any).listeners.add((m:any)=>{
 if(m.name==='GLOBAL_POSITION_INT'&&Math.hypot(m.values.vx,m.values.vy)>50)
   course=Math.atan2(m.values.vy,m.values.vx);
 if(m.name==='ATTITUDE')pitchRate=Number(m.values.pitchspeed);
});
sim.truthIntervalMs=0;sim.onLog=text=>logs.push({time:sim.physics?.time,text});
sim.onError=text=>failure=text;
sim.onTruth=f=>{
 if(f.time_s<start||contact||failure)return;
 const time=f.time_s-start;
 const sample={time,state:[...f.state],mode:sim.state.mode,active:sim.state.controllerActive,
   command:[...sim.physics.command],brakes:[...sim.physics.brakes],guidance:command,
   telemetry:{...sim.state}};
 if(-f.state[2]<=.12||-f.state[15]<=.05)contact={incoming:previous??sample,current:sample};
 if(time-last>=.049||contact){samples.push(sample);last=time}
 previous=sample;
 if(Math.abs(sim.state.roll)>Math.PI/3||Math.abs(sim.state.pitch)>Math.PI/3)failure='Attitude guard';
 if(contact||!guidance||time-controlTime<.049)return;
 const dt=Number.isFinite(controlTime)?time-controlTime:.05;controlTime=time;
 // Only AGL comes from truth. All navigation, attitude and vertical speed below
 // come from simulated sensor fusion. Height bias is an explicit test input.
 const agl=-f.state[2]+(spec.heightBias??0);
 if(agl<=3&&!requestManual) {
   requestManual=true;handoff=time;
   void sim.mode('MANUAL').catch(e=>failure=String(e));
 }
 if(requestManual&&sim.state.mode!==0&&time-handoff!>2)failure='Manual handoff timeout';
 command=guidance.update({
   north:(sim.state.lat-HOME.lat)*111319.49,
   east:(sim.state.lon-HOME.lon)*111319.49*Math.cos(HOME.lat*Math.PI/180),
   agl,speed:sim.state.speed,sink:-sim.state.climb,yaw:sim.state.yaw,pitch:sim.state.pitch,course,pitchRate
 },dt,sim.state.mode===0,time);
 sim.controls(command.roll,command.pitch,command.throttle);
 // The UI timer is wall-clock based. Research RC updates must follow sim time,
 // particularly during a sub-second flare at 8x playback.
 (sim as any).sendRC();
};
const delay=()=>new Promise(r=>setTimeout(r,10));
const save=()=>writeFile(output,JSON.stringify({spec,failure,complete:!!contact&&!failure,
 target,environment,handoff,samples,contact,logs})+'\n');
const timer=setTimeout(async()=>{failure='Wall timeout';await save();process.exit(1)},360000);
try {
 const c=JSON.parse(await readFile(new URL('../../../aircraft/microagu/configurations/browser-physics.json',import.meta.url),'utf8'));
 c.defaults+='\nRC1_DZ 0\nRC2_DZ 0\n';
 await sim.start(new URL('../../../viewer/generated/ardupilot/arduplane.js',import.meta.url).href,
 c,new URL('../../../viewer/generated/physics/',import.meta.url).href,
 new Uint8Array(await readFile(new URL('../../../aircraft/microagu/controllers/lqi-gains.bin',import.meta.url))));
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
 const speed=Math.max(2,sim.state.speed),height=-sim.physics.state[2];
 const distance=(height-3)*speed/.8+3*speed/1.38+.3*speed;
 target={north:sim.physics.state[0]+distance*dn,east:sim.physics.state[1]+distance*de,heading,speed};
 environment={windNED:[n,e,0],distance,initial:[...sim.physics.state]};
 if(spec.variant==='stock') {
   const point=(d:number,alt:number,cmd:number)=>({command:cmd,frame:3,params:[0,cmd===16?8:0,0,0],
     lat:HOME.lat+(sim.physics.state[0]+d*dn)/111319.49,
     lon:HOME.lon+(sim.physics.state[1]+d*de)/(111319.49*Math.cos(HOME.lat*Math.PI/180)),alt});
   await sim.upload([point(20,height,16),point(distance-35,12,16),point(distance,0,21)],()=>{});
   start=sim.physics.time;await sim.mode('AUTO');
 }else{
   guidance=new LandingGuidance(target,{feedback:spec.variant==='feedback',
     commonMm:spec.commonMm??54,flareHeight:spec.flareHeight??1.05});
   start=sim.physics.time;
 }
 while(sim.physics.time-start<180&&!contact&&!failure)await delay();
 if(!contact&&!failure)failure='Landing timeout';
}catch(e){failure=String(e)}
clearTimeout(timer);sim.onTruth=()=>{};sim.onError=()=>{};await save();sim.dispose();
console.log(JSON.stringify({id:spec.id,complete:!!contact&&!failure,failure}));process.exit(failure?1:0);
