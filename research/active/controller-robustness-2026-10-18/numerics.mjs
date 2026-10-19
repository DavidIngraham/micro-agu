import {run} from './calibrate.mjs';
import fs from 'node:fs/promises';
const forcing=JSON.parse(await fs.readFile(new URL('forcing-calibration.json',import.meta.url)))[2];
const mode={kind:'mode',hz:2.1,amplitude:.005,ratio:.4,phase:0,damping:.05,gain:.3};
const out=[forcing,mode].map(spec=>({spec,coarse:run(spec,55,8),fine:run(spec,55,16)}));
await fs.writeFile(new URL('numerics.json',import.meta.url),JSON.stringify(out));
