const {chromium}=require('%USERPROFILE%/repos/certaindeath/node_modules/playwright');
const fs=require('node:fs');
(async()=>{
 const browser=await chromium.launch({channel:'msedge',headless:true,args:['--no-proxy-server','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 try {
  const page=await browser.newPage({viewport:{width:1450,height:900}}),errors=[];
  page.on('pageerror',err=>errors.push(err.message));
  await page.goto('http://192.168.1.129:8764/');
  await page.waitForFunction(()=>typeof document.querySelector('#time').oninput==='function');
  if(!page.url().includes('case=loaded-cycle-48'))throw Error(page.url());
  const first=await page.locator('#liveMeasure').innerText();
  await page.locator('#time').evaluate(el=>{el.value=el.max;el.dispatchEvent(new Event('input'))});
  await page.waitForFunction(()=>document.querySelector('#liveMeasure').textContent.includes('6/6 scored'));
  const last=await page.locator('#liveMeasure').innerText();
  await page.locator('#duty').selectOption({label:'Loaded lift and drive · 4.8 s candidate'}); await page.waitForFunction(()=>document.querySelector('#resources').textContent.includes('8.712 V'));
  const resources=await page.locator('#resources').innerText();
  if(errors.length||!(await page.locator('#loads').innerText()).includes('4.8 s candidate')||!resources.includes('55.963'))throw Error(JSON.stringify({errors,head:resources.slice(0,500)}));
  fs.writeFileSync('verified-view.json',JSON.stringify({url:page.url(),first,last,matchedPowerRow:true,errors},null,2)); console.log(last)
 }finally{await browser.close()}
})().catch(e=>{console.error(e); process.exitCode=1});
