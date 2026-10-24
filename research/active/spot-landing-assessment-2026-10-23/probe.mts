/** Read back packaged firmware parameters without flying or changing them. */
import {readFile, writeFile} from 'node:fs/promises';
import {Simulator} from '../../../viewer/packages/sim-browser/src/session/runtime';
import {HOME} from '../../../viewer/apps/simulator/missions/default';
import definitions from '../../../viewer/generated/mavlink/dialect.json';

// Standard message metadata checked against the installed pymavlink common dialect.
// Research-only extension of the encoder's dictionary; production files are untouched.
Object.assign(definitions, {PARAM_REQUEST_READ: {
  id:20, crc:214, length:20, fields:[
    ['param_index','int16_t',0,1], ['target_system','uint8_t',2,1],
    ['target_component','uint8_t',3,1], ['param_id','char',4,16],
  ],
}});
Object.defineProperty(globalThis, 'crossOriginIsolated', {value:true});
const names=['TECS_LAND_SINK','LAND_FLARE_AIM','LAND_FLARE_ALT','LAND_FLARE_SEC',
  'PG_FLARE_HGT','PG_FLARE_PCT','PG_FLARE_RATE','PG_FLARE_RES','PG_FLARE_SINK','PG_FLARE_GAIN'];
const sim=new Simulator(HOME), values:Record<string,number>={};
(sim as any).listeners.add((m:any)=>{
  if(m.name==='PARAM_VALUE' && names.includes(String(m.values.param_id)))
    values[String(m.values.param_id)]=Number(m.values.param_value);
});
const timer=setTimeout(()=>{sim.dispose();console.error('Probe timeout');process.exit(1)},60000);
try {
  const config=JSON.parse(await readFile(new URL('../../../aircraft/microagu/configurations/browser-physics.json',import.meta.url),'utf8'));
  await sim.start(new URL('../../../viewer/generated/ardupilot/arduplane.js',import.meta.url).href,
    config,new URL('../../../viewer/generated/physics/',import.meta.url).href,
    new Uint8Array(await readFile(new URL('../../../aircraft/microagu/controllers/lqi-gains.bin',import.meta.url))));
  for(const param_id of names) {
    sim.send('PARAM_REQUEST_READ',{param_id,param_index:-1});
    const deadline=performance.now()+5000;
    while(!(param_id in values)) {
      if(performance.now()>deadline) throw Error(`Missing ${param_id}`);
      await new Promise(r=>setTimeout(r,10));
    }
  }
  await writeFile(new URL('./parameters.json',import.meta.url),JSON.stringify(values,null,2)+'\n');
  console.log(values);
} finally {clearTimeout(timer);sim.dispose()}
process.exit(0);
