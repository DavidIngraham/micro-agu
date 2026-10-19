import fs from 'node:fs/promises';
import init,{Experiment} from './physics/sim_wasm_extra.js';
import initOld,{Kernel} from '../../../viewer/generated/physics/sim_wasm.js';
const cfg=JSON.parse(await fs.readFile(new URL('../../../aircraft/microagu/configurations/browser-physics.json',import.meta.url)));
await init({module_or_path:await fs.readFile(new URL('physics/sim_wasm_extra_bg.wasm',import.meta.url))});await initOld({module_or_path:await fs.readFile(new URL('../../../viewer/generated/physics/sim_wasm_bg.wasm',import.meta.url))});
const u=[cfg.command[0],...cfg.command.slice(1).map((v,i)=>v/1000-cfg.brake_offsets_m[i])];
export function run(spec,seconds=55,substeps=8){const e=new Experiment(new Float64Array(cfg.parameters),cfg.mass);e.configure(JSON.stringify(spec));let y=new Float64Array(cfg.state),rpm=cfg.rpm;const rows=[];
 for(let i=0;i<seconds*100;i++){const z=e.step(y,new Float64Array(u),new Float64Array([0,0,0]),cfg.voltage,rpm,.01,substeps);y=z.slice(0,26);rpm=z[26];if(i%4===0)rows.push([i*.01,...y]);}e.free();return rows;}
const common={kind:'forcing',hz:1.6,amplitude:0,ratio:.4,phase:-Math.PI/2,damping:.1,gain:0};
const e=new Experiment(new Float64Array(cfg.parameters),cfg.mass);e.configure(JSON.stringify(common));const old=new Kernel(new Float64Array(cfg.parameters),cfg.mass);let y=new Float64Array(cfg.state),rpm=cfg.rpm,err=0;
for(let i=0;i<100;i++){const z=e.step(y,new Float64Array(u),new Float64Array([0,0,0]),cfg.voltage,rpm,.01,8);const ref=old.replay(y,new Float64Array([...u,...u]),new Float64Array([0,0,0]),cfg.voltage,rpm,.01,8,true);for(let j=0;j<27;j++)err=Math.max(err,Math.abs(z[j]-ref[27+j]));y=z.slice(0,26);rpm=z[26];}
if(err>1e-7)throw Error('Zero torque parity '+err);
const rows=[];for(const hz of [1,1.6,2.1,3]){for(const axis of ['yaw','pitch']){const spec={...common,hz,amplitude:axis==='yaw'?.005:0.000005,ratio:axis==='yaw'?0:1000,phase:0};rows.push({hz,axis,spec,rows:run(spec)});}}
await fs.writeFile(new URL('calibration-basis.json',import.meta.url),JSON.stringify({parity:err,config:cfg,rows}));console.log('parity',err);
