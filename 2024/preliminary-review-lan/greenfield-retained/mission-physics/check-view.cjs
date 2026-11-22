const {chromium}=require('C:/Users/ehart/repos/certaindeath/node_modules/playwright');
(async()=>{const b=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});try{
 const page=await b.newPage({viewport:{width:1440,height:1050}});page.on('pageerror',e=>console.error('PAGE',e.message));for(const key of ['shot','climb']){
 await page.goto('http://192.168.1.129:8764/?case='+key);await page.waitForFunction(()=>document.querySelector('#liveMeasure')?.textContent.length>0,null,{timeout:60000});await page.locator('#stage').scrollIntoViewIfNeeded();
 const start=await page.locator('#fraction').innerText();await page.locator('#play').click();await page.waitForTimeout(350);await page.locator('#play').click();if(start===await page.locator('#fraction').innerText())throw Error('Frozen playback');await page.locator('#time').evaluate((e,key)=>{e.value=key==='shot'?65:245;e.dispatchEvent(new Event('input'))},key);await page.waitForTimeout(300);
 console.log(key,await page.locator('#liveMeasure').innerText(),await page.locator('#motion').inputValue(),await page.locator('#time').getAttribute('max'));
 await page.locator('.viewer').screenshot({path:key+'-viewer.png'});
 if(await page.locator('#stage').evaluate(e=>e.querySelector('canvas').width)<600)throw Error('viewport');}
 }finally{await b.close()}})().catch(e=>{console.error(e);process.exitCode=1});

