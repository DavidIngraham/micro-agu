import {readFile,writeFile} from 'node:fs/promises';
import definitions from '../../../viewer/generated/mavlink/dialect.json';
// Standard metadata from installed pymavlink; extend research decoder before import.
Object.assign(definitions, {
 WIND:{id:168,crc:1,length:12,fields:[['direction','float',0,1],['speed','float',4,1],['speed_z','float',8,1]]},
 PARAM_REQUEST_READ:{id:20,crc:214,length:20,fields:[['param_index','int16_t',0,1],['target_system','uint8_t',2,1],['target_component','uint8_t',3,1],['param_id','char',4,16]]}
});
const {Simulator}=await import('../../../viewer/packages/sim-browser/src/session/runtime');
import {HOME,landingMission} from '../../../viewer/apps/simulator/missions/default';
import {calmWeather} from '../../../viewer/packages/contracts/src/weather';
import {defaultLidar} from '../../../viewer/packages/sim-browser/src/physics/lidar';
Object.defineProperty(globalThis,'crossOriginIsolated',{value:true});
const [input,output]=process.argv.slice(2),spec=JSON.parse(await readFile(input,'utf8'));
const sim=new Simulator(HOME),samples:any[]=[],logs:any[]=[];
const parameters:Record<string,number>={};
let start=Infinity,last=-Infinity,controlTime=-Infinity,failure:string|null=null;
let target:any=null,environment:any=null,contact:any=null,previous:any=null;
let handoff:number|null=null;
let course:number|undefined;
let pitchRate=0; let windEstimate:any=null; let airspeed:any=null;
let lidarRange:number|null=null,lidarTime=-Infinity,lastGoodTime=-Infinity,lastGoodHeight=0;
const lidarPackets:any[]=[],haglPackets:any[]=[];
let hagl:number|null=null,haglTime=-Infinity;
(sim as any).listeners.add((m:any)=>{
 if(m.name==='WIND') { const v=m.values, a=Number(v.direction)*Math.PI/180; windEstimate={ned:[-Number(v.speed)*Math.cos(a),-Number(v.speed)*Math.sin(a),Number(v.speed_z)],received:sim.physics.time}; }
 if(m.name==='VFR_HUD') airspeed={metresPerSecond:Number(m.values.airspeed),received:sim.physics.time};
 if(m.name==='PARAM_VALUE') parameters[String(m.values.param_id)]=Number(m.values.param_value);
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
   wind:{truth:f.wind_ned_m_s,estimate:windEstimate,airspeed},telemetry:structuredClone(sim.state),lidar:{range:lidarRange,received:lidarTime,generated:sim.physics.lidarReading,hagl,haglTime}};
 if(!contact&&(-f.state[2]<=.12||-f.state[15]<=.05))contact={incoming:previous??sample,current:sample};
 if(time-last>=.049||contact){samples.push(sample);last=time}
 previous=sample;
 if(!contact&&(Math.abs(sim.state.roll)>Math.PI/3||Math.abs(sim.state.pitch)>Math.PI/3))failure='Attitude guard';
 if(sim.state.landing?.phase==='final'&&handoff===null)handoff=time;
 if(sim.state.landing?.phase==='aborted')failure=sim.state.landing.message;

};
const delay=()=>new Promise(r=>setTimeout(r,10));
const save=()=>writeFile(output,JSON.stringify({spec,failure,complete:!!contact&&!failure,
 target,environment,handoff,parameters,samples,contact,logs,lidarPackets,haglPackets,finalLanding:sim.state.landing})+'\n');
process.on('unhandledRejection',async e=>{failure=String(e);await save();sim.dispose();process.exit(1)});
const timer=setTimeout(async()=>{failure='Wall timeout';await save();process.exit(1)},360000);
try {
 const c=JSON.parse(await readFile(new URL(spec.configuration??'./configuration.json',import.meta.url),'utf8'));
 c.lidar={...defaultLidar,...spec.lidar}; c.defaults += String.fromCharCode(10) + Object.entries(spec.parameters ?? {}).map(([k,v])=>`${k} ${v}`).join(String.fromCharCode(10));
 c.defaults+='\nRC1_DZ 0\nRC2_DZ 0\nRNGFND1_TYPE 100\nRNGFND1_ORIENT 25\nRNGFND1_MIN 0.1\nRNGFND1_MAX 40\n';
 await sim.start(new URL(spec.firmware,import.meta.url).href,
 c,new URL('../../../viewer/generated/physics/',import.meta.url).href,
 new Uint8Array(await readFile(new URL('../../../aircraft/microagu/controllers/lqi-gains.bin',import.meta.url))));
 await sim.command(511,[168,100000]);
 for(const param_id of ['ARSPD_TYPE','ARSPD_USE','AHRS_EKF_TYPE','EK3_WIND_P_NSE','EK3_BETA_MASK','EK3_EAS_I_GATE','LAND_FLARE_AIM','LAND_FLARE_ALT','LAND_FLARE_SEC','TECS_LAND_SINK','PG_FLARE_HGT']) sim.send('PARAM_REQUEST_READ',{param_id,param_index:-1});
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
 const [windN,windE,windD]=spec.windNED??[n,e,0];
 sim.physics.setWind(Math.hypot(windN,windE),(Math.atan2(-windE,-windN)*180/Math.PI+360)%360,-windD,
 {...calmWeather,...spec.weather,turbulence:spec.turbulence??0,seed:spec.seed??73});
 const weather=sim.physics.time;while(sim.physics.time-weather<15)await delay();
 environment={windNED:[windN,windE,windD],initial:[...sim.physics.state]};
 const navHeading=sim.state.course??sim.state.yaw;
 const speed=Math.max(2,sim.state.speed),height=sim.state.alt;
 const distance=height*speed/(spec.approachSink??.8);
 const lat=sim.state.lat,lon=sim.state.lon;
 const point=(dist:number,alt:number,command:number)=>({command,frame:3,params:[0,5,0,0],
 lat:lat+dist*Math.cos(navHeading)/111319.49,
 lon:lon+dist*Math.sin(navHeading)/(111319.49*Math.cos(HOME.lat*Math.PI/180)),alt});
 const mission=spec.fixedMission?landingMission():[point(30,height,16),point(distance+30,0,21)];
 target={north:(mission[mission.length-1].lat-HOME.lat)*111319.49,east:(mission[mission.length-1].lon-HOME.lon)*111319.49*Math.cos(HOME.lat*Math.PI/180),heading:navHeading};
 await sim.upload(mission,()=>{});await sim.execute();start=sim.physics.time;
 while(sim.physics.time-start<600&&!failure) {
   if(sim.state.simulationEnded)break;
   if(contact&&sim.physics.time-start-contact.current.time>8){failure="No landing confirmation after contact";break;}
   await delay();
 }
 if(!contact&&!failure)failure='Landing timeout';
}catch(e){failure=String(e)}
clearTimeout(timer);sim.onTruth=()=>{};sim.onError=()=>{};await save();sim.dispose();
console.log(JSON.stringify({id:spec.id,complete:!!contact&&!failure,failure}));process.exit(failure?1:0);







