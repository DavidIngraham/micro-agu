import { chromium } from '/home/david/.npm/_npx/e41f203b7505f1fb/node_modules/playwright/index.mjs';
import fs from 'node:fs/promises';
const root='/home/david/repos/ArduPilot-tinkering/';
const out='/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/landing-refinement-2026-10-13/';
const browser=await chromium.launch({headless:true,args:['--no-sandbox','--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
const page=await browser.newPage({viewport:{width:1280,height:1000}});const errors=[];
page.on('pageerror',e=>errors.push(String(e)));
await page.route('https://raw.githubusercontent.com/DavidIngraham/ArduPilot-tinkering/main/**',async route=>{
 const path=decodeURIComponent(route.request().url().split('/main/')[1].split('?')[0]);
 try{await route.fulfill({path:root+path});}catch(e){await route.abort();errors.push('missing local asset '+path);}
});
await page.goto('http://127.0.0.1:8075/research/?collection=ardupilot-tinkering&article=spot-landing');
await page.waitForSelector('mjx-container');await page.waitForTimeout(4000);
const report=await page.evaluate(()=>({math:document.querySelectorAll('mjx-container').length,mathErrors:document.querySelectorAll('mjx-merror').length,iframes:[...document.querySelectorAll('iframe')].map(x=>x.src),placeholders:document.body.innerText.includes('FINAL_RESULTS')}));
await page.screenshot({path:out+'article-check.png',fullPage:true});
for(const kind of ['calm','turbulent']){
 await page.goto('http://127.0.0.1:8075/assets/microagu-viewer/landing.html?case='+kind);
 await page.waitForFunction(()=>document.querySelector('#case-label')?.textContent?.includes('final controller'));
 await page.locator('#flare').click();await page.waitForTimeout(1000);await page.locator('#play').click();
 report[kind]=await page.locator('#case-label').innerText();
 if(!(await page.locator('canvas').count())||(await page.locator('#status').innerText()).includes('NaN'))throw Error('Invalid replay rendering');
 await page.screenshot({path:out+'applet-'+kind+'.png',fullPage:true});
}
report.errors=errors;await fs.writeFile(out+'publication-check.json',JSON.stringify(report,null,2));console.log(report);await browser.close();
if(report.mathErrors||errors.length||report.iframes.length!==2)process.exit(1);
