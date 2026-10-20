import {readFile,writeFile} from 'node:fs/promises';
import init,{Kernel} from '../../../viewer/generated/physics/sim_wasm.js';
await init({module_or_path:await readFile(new URL('../../../viewer/generated/physics/sim_wasm_bg.wasm',import.meta.url))});
const c=JSON.parse(await readFile(new URL('../../../aircraft/microagu/configurations/browser-physics.json',import.meta.url),'utf8'));
const k=new Kernel(new Float64Array(c.parameters),c.mass);
const result=k.surface_step(new Float64Array(c.state),new Float64Array([0,.036,.04]),
 new Float64Array([1,.5,0]),c.voltage,c.rpm,.00125,new Float64Array([2,0,1]),
 new Float64Array([.1,1.2,4.5,3,.8,.5]));
await writeFile(new URL('wasm-parity.json',import.meta.url),JSON.stringify(Array.from(result))+'\n');
k.free();
