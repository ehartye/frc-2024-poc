const {chromium}=require('C:/Users/ehart/repos/certaindeath/node_modules/playwright');
(async()=>{const b=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});const page=await b.newPage();const errors=[];page.on('pageerror',e=>errors.push(String(e)));try{
for(const [key,end] of [['split-96','6/6 scored'],['split-r02','0/1 scored']]){
 await page.goto('http://192.168.1.129:8764/?concept=split&case='+key);await page.waitForFunction(()=>typeof document.querySelector('#time').oninput==='function');
 await page.locator('#time').evaluate(x=>{x.value=x.max;x.dispatchEvent(new Event('input'))});await page.waitForFunction(v=>document.querySelector('#liveMeasure').textContent.includes(v),end);console.log(page.url(),await page.locator('#liveMeasure').innerText());
 if(!page.url().includes('concept=split'))throw Error('variant lost');}
 if(errors.length)throw Error(errors);
}finally{await b.close()}})().catch(e=>{console.error(e);process.exitCode=1});

