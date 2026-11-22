const { chromium } = require('C:/Users/ehart/repos/certaindeath/node_modules/playwright');
(async()=>{const browser=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const page=await browser.newPage({viewport:{width:1320,height:960}});const errors=[];page.on('pageerror',e=>errors.push(e.message));
 try{
 for(const [i,key] of ['six-note-route'].entries()){
 await page.goto('http://192.168.1.129:8764/?case='+key);
 await page.waitForFunction(()=>document.querySelector('#motion')?.selectedOptions[0]?.text.toLowerCase().includes('rehearsal'));
 await page.waitForFunction(()=>typeof document.querySelector('#time').oninput==='function' && Number(document.querySelector('#time').max)>5); const expected=await page.evaluate(async key=>(await (await fetch('/data.json')).json()).cases.find(c=>c.key===key).measure.at(-1),key); const label=await page.locator('#motion').inputValue();const slider=page.locator('#time');await slider.evaluate(el=>{el.value=el.max;el.dispatchEvent(new Event('input'))});await page.waitForFunction(v=>document.querySelector('#liveMeasure').textContent===v,expected);
 console.log(key,label,await page.locator('#liveMeasure').innerText());
 if(key==='six-note-route'){await page.locator('.viewer').screenshot({path:'mission.png'});}
 await page.locator('#play').click();await page.waitForTimeout(250);await page.locator('#play').click();
 if(Number(await slider.inputValue())===0)throw Error('Playback did not advance');
 }
 if(errors.length)throw Error(errors.join('; '));
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});


