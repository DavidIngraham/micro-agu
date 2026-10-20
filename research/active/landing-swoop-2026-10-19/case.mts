import {readFile,writeFile} from 'node:fs/promises';
import {HOME} from '../../../viewer/apps/simulator/missions/default';
import {Simulator} from '../../../viewer/packages/sim-browser/src/session/runtime';
import {calmWeather} from '../../../viewer/packages/contracts/src/weather';
Object.defineProperty(globalThis,'crossOriginIsolated',{value:true});
const [input,output]=process.argv.slice(2);
const spec=JSON.parse(await readFile(input,'utf8'));
const sim=new Simulator(HOME);
const samples:any[]=[],logs:any[]=[];
let failure:string|null=null,complete=false,start=Infinity,last=-Infinity;
let touchdown:any=null,previous:any=null,target:any=null,environment:any=null;
sim.truthIntervalMs=0;
sim.onLog=text=>logs.push({time:sim.physics?.time,text});
sim.onError=text=>failure=text;
sim.onTruth=f=>{
 if(f.time_s<start||touchdown||failure)return;
 const sample={time:f.time_s-start,state:[...f.state],command:[...sim.physics.command],
   brakes:[...sim.physics.brakes],mode:sim.state.mode,active:sim.state.controllerActive,
   alt:sim.state.alt,targetAlt:sim.state.targetAlt,roll:sim.state.roll,pitch:sim.state.pitch,phase:f.launch_phase};
 // Production phase is the LAST substep's contact flag. Stop above the contact
 // plane instead, before plastic impulses can erase incoming velocity.
 if(-f.state[2]<=.12||-f.state[15]<=.05){
   touchdown={time:sample.time,incoming:previous,contact:sample,definition:'20 mm above model contact surface'};
 }
 previous=sample;
 if(sample.time-last>=.049||touchdown?.time===sample.time){samples.push(sample);last=sample.time}
 if(spec.kind==='recovery'&&-f.state[2]<25)failure='25 m recovery floor';
 if(Math.abs(sim.state.roll)>Math.PI/3||Math.abs(sim.state.pitch)>Math.PI/3)failure='60 degree attitude guard';
};
const save=()=>writeFile(output,JSON.stringify({spec,complete,failure,target,environment,touchdown,samples,logs})+'\n');
const delay=()=>new Promise(r=>setTimeout(r,10));
const timer=setTimeout(async()=>{failure='Wall timeout';await save();process.exit(1)},300000);
try{
 const config=JSON.parse(await readFile(new URL('../../../aircraft/microagu/configurations/browser-physics.json',import.meta.url),'utf8'));
 for(const [key,value] of Object.entries(spec.parameters??{}))config.defaults+='\n'+key+' '+value+'\n';
 await sim.start(new URL('../../../viewer/generated/ardupilot/arduplane.js',import.meta.url).href,
   config,new URL('../../../viewer/generated/physics/',import.meta.url).href,
   new Uint8Array(await readFile(new URL('../../../aircraft/microagu/controllers/lqi-gains.bin',import.meta.url))));
 await sim.controller('baseline');
 // Common classical climb; select the comparison controller only at altitude.
 await sim.speedup(8);await sim.launch('CRUISE');await sim.altitude(60);
 const selected=sim.physics.time;let quiet=selected;
 while(sim.physics.time-selected<240){
   if(Math.abs(-sim.physics.state[2]-60)>1||Math.abs(sim.physics.state[9])>.3)quiet=sim.physics.time;
   if(sim.physics.time-quiet>3)break;
   await delay();
 }
 if(sim.physics.time-selected>=240)throw Error('Approach settling timeout');
 await sim.controller(spec.scheme);
 const changed=sim.physics.time;
 while(sim.physics.time-changed<20)await delay();
 if(spec.scheme==='lqi'&&!sim.state.controllerActive)throw Error('LQI unavailable before approach');
 const heading=Math.atan2(sim.physics.state[8],sim.physics.state[7]),dn=Math.cos(heading),de=Math.sin(heading);
 const along=spec.north??0,across=spec.east??0;
 const n=along*dn-across*de,e=along*de+across*dn;
 sim.physics.setWind(Math.hypot(n,e),(Math.atan2(-e,-n)*180/Math.PI+360)%360,0,
   {...calmWeather,turbulence:spec.turbulence??0,seed:spec.seed??73});
 const weatherStart=sim.physics.time;
 while(sim.physics.time-weatherStart<15)await delay();
 const p=sim.physics.state;
 environment={windNED:[n,e,0],along,across,heading,initialState:[...p],
   controllerActive:sim.state.controllerActive};
 target={north:p[0]+200*dn,east:p[1]+200*de,heading};
 if(spec.kind==='landing'){
   const point=(distance:number,alt:number,command:number)=>({
     command,frame:3,params:[0,command===16?8:0,0,0],
     lat:HOME.lat+(p[0]+distance*dn)/111319.49,
     lon:HOME.lon+(p[1]+distance*de)/(111319.49*Math.cos(HOME.lat*Math.PI/180)),alt});
   await sim.upload([point(20,60,16),point(100,30,16),point(200,0,21)],()=>{});
   start=sim.physics.time;await sim.mode('AUTO');
   while(sim.physics.time-start<160&&!touchdown&&!failure)await delay();
   if(!touchdown&&!failure)failure='Landing timeout';
 }else{
   start=sim.physics.time;
   // Conservative altitude-only descending turn / recovery precursor.
   sim.controls(spec.turnRate/35,0,spec.descentLever??0);
   while(sim.physics.time-start<8&&!failure)await delay();
   sim.controls(0,0,spec.climbLever??1);
   while(sim.physics.time-start<18&&!failure)await delay();
   sim.controls(0,0,.5);await sim.altitude(60);
   while(sim.physics.time-start<65&&!failure)await delay();
 }
 complete=!failure;
}catch(e){failure=String(e)}
clearTimeout(timer);sim.onTruth=()=>{};sim.onError=()=>{};await save();sim.dispose();
console.log(JSON.stringify({id:spec.id,complete,failure,samples:samples.length}));
process.exit(complete?0:1);
