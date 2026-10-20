import test from "node:test";
import assert from "node:assert/strict";
import { Lidar, defaultLidar } from "../../packages/sim-browser/src/physics/lidar";
import { MAVLink } from "../../packages/sim-browser/src/ardupilot/mavlink";
const state=(h:number,angle=0)=>{
  const y=Array(26).fill(0);y[2]=-h;y[4]=Math.sin(angle/2);y[6]=Math.cos(angle/2);return y;
};
const config={...defaultLidar,noise_std_m:0,dropout_probability:0,latency_s:0};
test("lidar reports slant range, not vertical truth height",()=>{
  const x=new Lidar(config).sample(0,state(10,Math.PI/6));
  assert(x.valid);assert(Math.abs(x.range_m-10/Math.cos(Math.PI/6))<.01);
});
test("lidar enforces latency and sample hold",()=>{
  const l=new Lidar({...config,latency_s:.05});
  assert(!l.sample(0,state(10)).valid);
  assert(!l.sample(.025,state(9)).valid);
  assert.equal(l.sample(.05,state(8)).range_m,10);
  assert.equal(l.sample(.1,state(6)).range_m,8);
});
test("inverted, oblique, too near and too far returns are invalid",()=>{
  for(const [h,a] of [[10,Math.PI],[10,Math.PI*.4],[.05,0],[50,0]])
    assert(!new Lidar(config).sample(0,state(h,a)).valid);
});
test("dropout invalidates rather than holds the previous measurement",()=>{
  const l=new Lidar({...config,dropout_probability:1});
  assert.equal(l.sample(0,state(10)).range_m,41);
});
test("offset, deterministic noise and bad configuration",()=>{
  assert.equal(new Lidar({...config,offset_body_m:[0,0,.2]}).sample(0,state(10)).range_m,9.8);
  const a=new Lidar(defaultLidar),b=new Lidar(defaultLidar);
  for(let i=0;i<50;i++)assert.deepEqual(a.sample(i*.05,state(10)),b.sample(i*.05,state(10)));
  assert.throws(()=>new Lidar({...config,rate_hz:0}));
});
test("rangefinder telemetry survives the browser MAVLink decoder",()=>{
  const codec=new MAVLink();
  const packet=codec.encode("DISTANCE_SENSOR",{time_boot_ms:123,min_distance:10,
    max_distance:4000,current_distance:250,orientation:25,id:0,type:0});
  const decoded=codec.feed(packet)[0];
  assert.equal(decoded.name,"DISTANCE_SENSOR");
  assert.equal(decoded.values.current_distance,250);
  assert.equal(decoded.values.orientation,25);
});
