const { chromium } = require('C:/Users/ehart/repos/certaindeath/node_modules/playwright');
(async()=>{
 const browser=await chromium.launch({channel:'msedge',headless:true,args:['--no-proxy-server','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 try{
 const page=await browser.newPage();const errs=[],bad=[];
 page.on('pageerror',e=>errs.push(e.message));page.on('response',r=>{if(r.status()>=400)bad.push([r.status(),r.url()])});
 const root='http://127.0.0.1:18763/preliminary-design/';
 for(const [query,id,expected] of [['','002','continuous-cycle-96'],['?release=001','001','loaded-cycle-48'],['?release=002&case=score-to-climb-front','002','score-to-climb-front']]){
  await page.goto(root+query);
  const frame=page.frameLocator('iframe');
  await frame.locator('#time').evaluate(el=>new Promise((ok,fail)=>{const start=Date.now();const id=setInterval(()=>{if(el.oninput){clearInterval(id);ok()}else if(Date.now()-start>30000){clearInterval(id);fail('viewer init timeout')}},100)}));
  const selected=await page.locator('#release').inputValue(),u=await page.locator('#direct').getAttribute('href');
  if(selected!==id||!u.includes(expected)) throw Error(JSON.stringify([selected,u,expected]));
  await frame.locator('#time').evaluate(el=>{el.value=el.max;el.dispatchEvent(new Event('input'))});
  const desired=expected==='score-to-climb-front'?'Power cut':expected==='loaded-cycle-48'?'6/6 scored':'6/6 individual events';
  const context=page.frames().find(f=>f.url().includes('/releases/'+id+'/'));
  await context.waitForFunction(d=>document.querySelector('#liveMeasure').textContent.includes(d),desired,{timeout:20000});
  console.log(id,expected,await frame.locator('#liveMeasure').innerText());
 }
 await page.goto(root+'?release=missing');await page.locator('#error').waitFor({state:'visible'});
 await page.selectOption('#release','002');await page.waitForURL('**/preliminary-design/?release=002**');
 if(errs.length||bad.length)throw Error(JSON.stringify({errs,bad}));console.log('Latest, archived, deep-link, unknown and selection routes passed');
 } finally {await browser.close()}
})().catch(e=>{console.error(e);process.exitCode=1});


