const {chromium}=require('C:/Users/ehart/repos/certaindeath/node_modules/playwright');
const fs=require('node:fs');
(async()=>{
 const b=await chromium.launch({channel:'msedge',headless:true,args:['--no-proxy-server','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 try{
 const p=await b.newPage();const errors=[],results=[];p.on('pageerror',e=>errors.push(e.message));
 for(const [key,expected] of [['counter-driven','2/2'],['counter-reversed','0/2']]){
    await p.goto('http://192.168.1.129:8764/?concept=guided&case='+key);
    await p.waitForFunction(()=>typeof document.querySelector('#time').oninput==='function',null,{timeout:60000});
    await p.locator('#time').evaluate(x=>{x.value=x.max;x.dispatchEvent(new Event('input'))});
    await p.waitForFunction(()=>Number(document.querySelector('#time').value)>0 && !document.querySelector('#liveMeasure').textContent.includes('0.00 s'));const message=await p.locator('#liveMeasure').innerText();
    if(!message.includes(expected)||!p.url().includes(key))throw Error(key+' '+message);
    results.push({key,url:p.url(),message});
 }
 if(errors.length)throw Error(errors.join());fs.writeFileSync('view-check.json',JSON.stringify(results,null,2));console.log(results);
 }finally{await b.close()}
})().catch(e=>{console.error(e);process.exitCode=1})

