const { chromium }=require('%USERPROFILE%/repos/certaindeath/node_modules/playwright');
const fs=require('node:fs');
(async()=>{
 const browser=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const page=await browser.newPage(), errors=[],rows=[];page.on('pageerror',e=>errors.push(String(e)));
 try{
  for(const [search,key] of [['','field-96'],['?case=field-r01','field-r01'],['?concept=field&case=field-climb','field-climb']]){
   await page.goto('http://192.168.1.129:8764/'+search);await page.waitForFunction(()=>typeof document.querySelector('#time').oninput==='function',null,{timeout:60000});
   const target=await page.evaluate(async(k)=>(await(await fetch('/data.json')).json()).concepts.field.cases.find(c=>c.key===k).measure.at(-1),key);
   await page.locator('#time').evaluate(el=>{el.value=el.max;el.dispatchEvent(new Event('input'));});
   await page.waitForFunction(v=>document.querySelector('#liveMeasure').textContent===v,target);
   rows.push({url:page.url(),title:await page.locator('h1').innerText(),end:target,metrics:await page.locator('#metrics').innerText()});
   if(!page.url().includes('concept=field')||!page.url().includes(key))throw Error('unexpected URL '+page.url());
  }
  if(errors.length)throw Error(errors);fs.writeFileSync('viewer-check.json',JSON.stringify(rows,null,2));console.log(rows.map(r=>r.end));
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1});

