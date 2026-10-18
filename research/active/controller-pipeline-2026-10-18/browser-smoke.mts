/** AI-assisted smoke test against the actual release, including controller ownership. */
import { chromium, expect } from '../../../viewer/node_modules/@playwright/test/index.mjs';
import { writeFile } from 'node:fs/promises';
import { spawn, type ChildProcess } from 'node:child_process';
let [base,output]=process.argv.slice(2);
if(!base||!output)throw Error('Usage: browser-smoke.ts BASE_URL OUTPUT_JSON');
const browser=await chromium.launch({headless:true,args:['--enable-webgl','--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
const page=await browser.newPage({viewport:{width:1280,height:900}});
let server:ChildProcess|undefined;
const errors:string[]=[];page.on('pageerror',e=>errors.push(e.message));
try{
  if(base==='local'){
    base='http://127.0.0.1:4197/';
    server=spawn('/usr/bin/python3',['-m','http.server','4197','--bind','127.0.0.1','--directory',new URL('./'+(process.env.RELEASE_OUTPUT??'release'),import.meta.url).pathname],{stdio:'ignore'});
    let ready=false;
    for(let i=0;i<50&&!ready;i++){
      try{ready=(await fetch(base)).ok;}catch{}
      if(!ready)await new Promise(r=>setTimeout(r,100));
    }
    if(!ready)throw Error('Release preview server did not start');
  }
  await page.goto(new URL('simulator.html',base).href,{waitUntil:'domcontentloaded',timeout:60000});
  await expect(page.locator('#status')).toContainText('AUTO confirmed',{timeout:130000});
  await expect(page.locator('#panel')).toBeHidden();
  await page.locator('#panel-toggle').click();
  await page.locator('#controller').selectOption('lqi');
  await expect(page.locator('#controller-status')).toHaveText('LQI active',{timeout:20000});
  await page.waitForTimeout(10000);
  await expect(page.locator('#controller-status')).toHaveText('LQI active');
  await page.screenshot({path:output.replace(/\.json$/,'.png')});
  await page.locator('#mode').selectOption('MANUAL');
  await expect(page.locator('#mode-label')).toHaveText('MANUAL');
  await expect(page.locator('#controller-status')).not.toHaveText('LQI active');
  await page.locator('#mode').selectOption('CRUISE');
  await expect(page.locator('#controller-status')).toHaveText('LQI active',{timeout:15000});
  await page.locator('#controller').selectOption('baseline');
  await expect(page.locator('#controller-status')).toHaveText('Baseline');
  const config=await (await page.request.get(new URL('wasm/highdof.json',base).href)).json();
  const build=await (await page.request.get(new URL('wasm/build.json',base).href)).json();
  expect(config.defaults).toMatch(/CP3_HRTC 0\.7/);
  expect(build.controller_reference_filter.CP3_HRTC).toBe(.7);
  expect(build.controller_turn_envelope.CP3_RMAX).toBe(Number(process.env.RELEASE_RATE??35));
  expect(build.controller_turn_envelope.handover_blend_seconds).toBe(2);
  expect(config.defaults).toMatch(/CP3_TMAP 1/);
  expect(config.defaults).toMatch(/CP3_AWMODE 1/);
  expect(config.defaults).toMatch(/CP3_OBSTRIM 1/);
  expect(build.controller_gain_file.candidate).toBe('refit_damped');
  expect(build.sha256['arduplane.wasm']).toBe('466d4d86263d973cc38e850b3a015146e9b98a2cd2ae369fbca7a32eeb9ab504');
  expect(errors).toEqual([]);
  await writeFile(output,JSON.stringify({passed:true,url:base,firmware:build.sha256['arduplane.wasm'],checks:['AUTO launch','LQI engagement','MANUAL ownership','CRUISE re-engagement','baseline handover','configured filter','no page errors']},null,2));
}catch(error){
  await writeFile(output,JSON.stringify({passed:false,error:String(error),errors,status:await page.locator('#status').textContent()},null,2));
  throw error;
}finally{await browser.close();server?.kill();}
