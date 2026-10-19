import {readFile,writeFile} from 'node:fs/promises';
import {resolve} from 'node:path';
import {pathToFileURL} from 'node:url';
import {HOME,exampleMission} from '../../../viewer/apps/simulator/missions/default';
import {Simulator} from '../../../viewer/packages/sim-browser/src/session/runtime';
import {calmWeather} from '../../../viewer/packages/contracts/src/weather';
Object.defineProperty(globalThis,'crossOriginIsolated',{value:true});
const [input,output]=process.argv.slice(2),spec=JSON.parse(await readFile(input,'utf8'));
const config=JSON.parse(await readFile(new URL('../../../aircraft/microagu/configurations/browser-physics.json',import.meta.url),'utf8'));
for(const [k,v] of Object.entries({ARSPD_TYPE:0,ARSPD_USE:0,CP3_HRTC:.7,CP3_HUP:.75,CP3_HDN:1,...spec.parameters}))config.defaults+='\n'+k+' '+v+'\n';
const sim=new Simulator(HOME);sim.truthIntervalMs=0;const samples:any[]=[],logs:string[]=[];let start=Infinity,last=-Infinity,failure:string|null=null,complete=false;
sim.onLog=s=>logs.push(s);sim.onError=s=>failure=s;
sim.onTruth=f=>{const t=f.time_s-start;if(t<0||t-last<.049)return;last=t;samples.push({time:t,state:f.state,rpm:f.rpm,command:[...sim.physics.command],brakes:[...sim.physics.brakes],active:sim.state.controllerActive,alt:sim.state.alt,targetAlt:sim.state.targetAlt,roll:sim.state.roll,pitch:sim.state.pitch,modal:Array.from((sim.physics as any).kernel.modal())});if(-f.state[2]<15)failure='Height below 15m';if(Math.abs(sim.state.roll)>Math.PI/3||Math.abs(sim.state.pitch)>Math.PI/4)failure='Attitude guard';if(t>5&&sim.state.controllerActive!==(spec.scheme==='lqi'))failure='Controller ownership';};
const delay=()=>new Promise(r=>setTimeout(r,10));
const save=()=>writeFile(output,JSON.stringify({spec,complete,failure,samples,logs})+'\n');
const timeout=setTimeout(async()=>{failure='Wall timeout';await save();process.exit(1)},300000);
try{
 await sim.start(new URL('runtime/arduplane.js',import.meta.url).href,config,new URL('physics/',import.meta.url).href,new Uint8Array(await readFile(new URL('../../../aircraft/microagu/controllers/lqi-gains.bin',import.meta.url))));
 await sim.upload(exampleMission(),()=>{});await sim.launch('AUTO');await sim.speedup(8);await sim.mode('CRUISE');await sim.controller(spec.scheme);await sim.altitude(80);
 const selected=sim.physics.time;let quiet=selected;
 while(sim.physics.time-selected<220&&!failure){if(Math.abs(-sim.physics.state[2]-80)>1||Math.abs(sim.physics.state[9])>.2)quiet=sim.physics.time;if(sim.physics.time-quiet>4)break;await delay();}
 if(sim.physics.time-selected>=220)throw Error('Initial settling failed');
 sim.physics.setWind(0,0,0,{...calmWeather,turbulence:0,thermalStrength:0});
 start=sim.physics.time;(sim.physics as any).kernel.configure(JSON.stringify(spec.experiment));sim.controls(0,0,.5);
 let stage=0;
 while(sim.physics.time-start<85&&!failure){const t=sim.physics.time-start;
  if(spec.maneuver&&stage===0&&t>=20){sim.controls(20/35,0,1);stage++;}
  if(spec.maneuver&&stage===1&&t>=35){sim.controls(-20/35,0,0);stage++;}
  if(spec.maneuver&&stage===2&&t>=50){sim.controls(0,0,.5);await sim.altitude(80);stage++;}
  await delay();
 }
 complete=!failure;
}catch(e){failure=String(e)}
clearTimeout(timeout);await save();console.log(JSON.stringify({id:spec.id,complete,failure}));process.exit(complete?0:1);
