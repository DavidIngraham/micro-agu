import {run} from './calibrate.mjs';import fs from 'node:fs/promises';
const specs=JSON.parse(await fs.readFile(new URL('forcing-calibration.json',import.meta.url)));const out=[];for(const spec of specs)out.push({spec,rows:run(spec)});await fs.writeFile(new URL('calibration-verified.json',import.meta.url),JSON.stringify(out));
