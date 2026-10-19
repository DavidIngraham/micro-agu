import {readFile,writeFile} from 'node:fs/promises';
import {HOME,exampleMission} from '../../../viewer/apps/simulator/missions/default';
import {Simulator} from '../../../viewer/packages/sim-browser/src/session/runtime';
import {defaultWeather} from '../../../viewer/apps/simulator/weather';
import {calmWeather} from '../../../viewer/packages/contracts/src/weather';
Object.defineProperty(globalThis,'crossOriginIsolated',{value:true});
const [input,output]=process.argv.slice(2);
const spec=JSON.parse(await readFile(input,'utf8'));
const sim=new Simulator(HOME);
const samples:any[]=[],logs:any[]=[];const startup:Record<string,number>={};
let failure:string|null=null,complete=false,last=-Infinity,stage=false,eligibilityFlags=0,handoff:number|null=null;
sim.truthIntervalMs=0;
sim.onLog=text=>logs.push({time:sim.physics?.time,text});
sim.onError=text=>failure=text;
(sim as any).listeners.add((m:any)=>{if(m.name==='NAMED_VALUE_FLOAT')startup[String(m.values.name)]=Number(m.values.value);if(m.name==='NAMED_VALUE_FLOAT'&&String(m.values.name)==='TKST')stage=Number(m.values.value)>.5;if(m.name==='NAMED_VALUE_FLOAT'&&String(m.values.name)==='TKWHY')eligibilityFlags=Number(m.values.value)});
sim.onTruth=f=>{
 if(!sim.physics.released||f.time_s-last<.049)return;
 last=f.time_s;
 samples.push({time:f.time_s,elapsed:sim.physics.launchElapsed,state:f.state,rpm:f.rpm,phase:f.launch_phase,powerReady:sim.physics.powerReady,command:[...sim.physics.command],brakes:[...sim.physics.brakes],mode:sim.state.mode,active:sim.state.controllerActive,takeoffStage:stage,eligibilityFlags,startup:{...startup},alt:sim.state.alt,roll:sim.state.roll,pitch:sim.state.pitch});
 if(sim.physics.launchElapsed>6&&f.launch_phase!==2)failure='Ground contact after release';
};
const save=()=>writeFile(output,JSON.stringify({spec,complete,failure,handoff,samples,logs}));
const timer=setTimeout(async()=>{failure='Wall timeout';await save();process.exit(1)},240000);
try{
 const config=JSON.parse(await readFile(new URL('../../../aircraft/microagu/configurations/browser-physics.json',import.meta.url),'utf8'));
 const firmware=new URL('runtime/arduplane.js',import.meta.url);
 for(const [k,v] of Object.entries({CP3_STMODE:{gate:0,zero:1,weighted:2}[spec.strategy],CP3_WPRN:spec.priorNorth??0,CP3_WPRE:spec.priorEast??0,CP3_WDLY:spec.windDelay??0}))config.defaults+='\n'+k+' '+v+'\n';
 await sim.start(firmware.href,config,new URL('../../../viewer/generated/physics/',import.meta.url).href,new Uint8Array(await readFile(new URL('../../../aircraft/microagu/controllers/lqi-gains.bin',import.meta.url))));
 await sim.controller(spec.scheme??'lqi');
 sim.physics.setWind(Math.hypot(spec.north??0,spec.east??0),(Math.atan2(-(spec.east??0),-(spec.north??0))*180/Math.PI+360)%360,0,{...(spec.weather==='calm'?calmWeather:defaultWeather),seed:spec.seed});
 await sim.upload(exampleMission(),()=>{});await sim.speedup(8);
 const switching=spec.switchTest?(async()=>{
  while(sim.physics.launchElapsed<10)await new Promise(r=>setTimeout(r,10));
  await sim.controller('baseline');
  while(sim.physics.launchElapsed<14)await new Promise(r=>setTimeout(r,10));
  await sim.controller('lqi');
 })():Promise.resolve();
 await sim.launch('AUTO');await switching;handoff=sim.physics.time;
 while(sim.physics.time-handoff<60&&!failure)await new Promise(r=>setTimeout(r,10));
 complete=!failure;
}catch(e){failure=String(e)}
clearTimeout(timer);await save();sim.onError=()=>{};sim.dispose();
console.log(JSON.stringify({id:spec.id,complete,failure,samples:samples.length,handoff}));process.exit(complete?0:1);
