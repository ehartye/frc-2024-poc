const {chromium}=require('C:/Users/ehart/repos/certaindeath/node_modules/playwright');
const fs=require('node:fs');
(async()=>{const b=await chromium.launch({channel:'msedge',headless:true,args:['--no-proxy-server','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
try{const p=await b.newPage({viewport:{width:1400,height:950}}),errors=[];p.on('pageerror',e=>errors.push(e.message));
const base='http://192.168.1.129:8764/?concept=guided&case=';
await p.goto(base+'observed-reset-96');await p.waitForFunction(()=>typeof document.querySelector('#time').oninput==='function');await p.selectOption('#motion',{label:'Slide motor cartridge service'});await p.waitForURL('**/\?concept=guided&case=slide-motor-service');await p.waitForFunction(()=>document.querySelector('h1').innerText.includes('motor service'));await p.locator('#time').evaluate(e=>{e.value=e.max;e.dispatchEvent(new Event('input'))});await p.waitForFunction(()=>document.querySelector('#liveMeasure').textContent.includes('45 / 45 mm'));
const tip=await p.locator('#liveMeasure').innerText(),weight=await p.locator('#resources').innerText();
await p.locator('#inventory tr').filter({hasText:'Double pinch hub'}).first().click();const selected=await p.locator('#inspector').innerText();if(!selected.includes('Double pinch hub'))throw Error(selected);
await p.selectOption('#motion',{label:'Six NOTES · full post-shot observation'});await p.waitForURL('**/\?concept=guided&case=observed-reset-96');await p.waitForFunction(()=>typeof document.querySelector('#time').oninput==='function');if(await p.locator('h1').innerText()==='Guided cassette · slide motor service'||errors.length)throw Error(JSON.stringify(errors));
fs.writeFileSync('browser-verified.json',JSON.stringify({tip,weight,selected,returned:p.url(),errors},null,2));console.log(tip,weight.slice(0,180));}finally{await b.close()}})().catch(e=>{console.error(e);process.exitCode=1})

