import assert from 'node:assert/strict';
import { chart, makeSensor } from './sensor';
import fixtures from './chart-fixtures.json';
let worst=0;
for(const f of fixtures){
  const got=chart(f.state,f.rpm,f.wind);
  got.forEach((x,i)=>{worst=Math.max(worst,Math.abs(x-f.expected[i]));assert.ok(Math.abs(x-f.expected[i])<1e-10);});
}
const f=fixtures[0];
for(const kind of ['ideal','nodelay','truth','sensor','stress']){
  const a=makeSensor(kind,42,{rpm:f.rpm,windNED:f.wind} as any);
  const b=makeSensor(kind,42,{rpm:f.rpm,windNED:f.wind} as any);
  for(let i=0;i<30;i++)assert.deepEqual(a(f.state,i*.01),b(f.state,i*.01));
}
console.log(JSON.stringify({fixtures:fixtures.length,maxChartError:worst,seedReproducibility:true}));
