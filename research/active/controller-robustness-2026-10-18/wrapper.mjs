import initOriginal,{Kernel as Original,Dryden} from '../../../../viewer/generated/physics/sim_wasm.js';
import initExtra,{Experiment} from './sim_wasm_extra.js';
import {readFile} from 'node:fs/promises';
export {Dryden};
export default async function(opts){await initOriginal({module_or_path:await readFile(new URL('../../../../viewer/generated/physics/sim_wasm_bg.wasm',import.meta.url))});await initExtra(opts);}
export class Kernel{
 constructor(p,m){this.original=new Original(p,m);this.extra=new Experiment(p,m);this.enabled=false;}
 configure(s){this.extra.configure(s);this.enabled=true;}
 modal(){return this.extra.modal();}
 free(){this.original.free();this.extra.free();}
 inverted_launch(...a){return this.original.inverted_launch(...a);}
 derivative(...a){return this.original.derivative(...a);}
 surface_step(y,u,w,v,r,dt,env,settings){if(!this.enabled)return this.original.surface_step(y,u,w,v,r,dt,env,settings);return new Float64Array([...this.extra.step(y,u,w,v,r,dt,Math.max(1,Math.ceil(dt/.00125))),...env]);}
 replay(y,us,w,v,r,dt,n,causal){if(!this.enabled)return this.original.replay(y,us,w,v,r,dt,n,causal);return new Float64Array([...y,r,...this.extra.step(y,us.slice(0,3),w,v,r,dt,n)]);}
}
