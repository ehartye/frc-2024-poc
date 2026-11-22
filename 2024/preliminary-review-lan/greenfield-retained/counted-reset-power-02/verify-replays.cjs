const { chromium }=require('C:/Users/ehart/repos/certaindeath/node_modules/playwright');
const fs=require('node:fs');
(async()=>{
 const browser=await chromium.launch({channel:'msedge',headless:true,args:['--no-proxy-server','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 try {
  const p=await browser.newPage({viewport:{width:1450,height:960}});const errors=[];p.on('pageerror',e=>errors.push(e.message));
  await p.goto('http://192.168.1.129:8764/?concept=guided');
  await p.waitForFunction(()=>typeof document.querySelector('#time').oninput==='function');
  const results=[];
  for(const [key,match] of [['continuous-cycle-96','6/6 individual events'],['score-to-climb-front','12.0 mm clearance'],['score-to-climb-rear','90.0 mm clearance']]){
   if(!p.url().includes(key)){const label=key==='score-to-climb-front'?'Sixth shot to climb · forward CG':'Sixth shot to climb · rearward CG';await p.selectOption('#motion',{label})}
   await p.waitForFunction(k=>new URL(location.href).searchParams.get('case')===k,key);
   await p.locator('#time').evaluate(el=>{el.value=el.max;el.dispatchEvent(new Event('input'))});
   await p.waitForFunction(m=>document.querySelector('#liveMeasure').textContent.includes(m),match);
   results.push([key,await p.locator('#liveMeasure').innerText()]);
  }
  await p.selectOption('#motion',{label:'Slide motor cartridge service'});
  await p.waitForURL('**/?concept=guided&case=slide-motor-service');
  await p.waitForFunction(()=>typeof document.querySelector('#time').oninput==='function');
  await p.locator('#time').evaluate(el=>{el.value=el.max;el.dispatchEvent(new Event('input'))});
  await p.waitForFunction(()=>document.querySelector('#liveMeasure').textContent.includes('45 / 45 mm'));
  results.push(['slide-motor-service',await p.locator('#liveMeasure').innerText()]);
  if(errors.length)throw Error(errors.join('\n'));
  fs.writeFileSync('browser-replays.json',JSON.stringify({results,errors},null,2));console.log(results);
 }finally{await browser.close()}
})().catch(e=>{console.error(e);process.exitCode=1});

