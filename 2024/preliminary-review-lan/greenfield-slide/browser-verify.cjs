const {chromium}=require('C:/Users/ehart/repos/certaindeath/node_modules/playwright');
const fs=require('fs'),p=require('path');
(async()=>{const b=await chromium.launch({channel:'msedge',args:['--no-proxy-server','--use-angle=swiftshader','--enable-unsafe-swiftshader']});try{
 const page=await b.newPage({viewport:{width:1440,height:960}}),errors=[];page.on('pageerror',e=>errors.push(e.message));
 await page.addInitScript(()=>{let n=0;const proto=WebGLRenderingContext.prototype,send=proto.bufferData;proto.bufferData=function(...args){n++;return send.apply(this,args)};window.countUploads=()=>n});
 await page.goto('http://192.168.1.129:8764/');await page.waitForFunction(()=>document.querySelector('h1').textContent.includes('P2.2')&&document.querySelectorAll('#inventory tr[data-name]').length===92);
 await page.check('#labels');await page.locator('#names .tag').first().waitFor();const labels=await page.locator('#names .tag').count();await page.uncheck('#labels');const uploads=await page.evaluate(()=>window.countUploads());
 for(let i=0;i<8;i++)await page.locator('canvas').press('ArrowRight');await page.locator('canvas').press('Shift+ArrowDown');const after=await page.evaluate(()=>window.countUploads());
 await page.selectOption('#group','Subsystem');await page.selectOption('#subset','Pivot support');await page.selectOption('#motion','Battery withdrawal');await page.locator('#findings tr[data-finding]').first().waitFor();await page.selectOption('#set',{index:1});await page.selectOption('#set',{index:0});
 const reload=page.waitForNavigation({waitUntil:'load',timeout:10000});const css=p.resolve(__dirname,'../www/review.css');const now=new Date();fs.utimesSync(css,now,now);await reload;await page.waitForFunction(()=>document.querySelectorAll('#inventory tr[data-name]').length===92);
 await page.screenshot({path:p.resolve(__dirname,'current-view.png')});
 console.log(JSON.stringify({revision:await page.locator('h1').innerText(),labels,extraOrbitUploads:after-uploads,hotReload:true,errors}));if(errors.length||after!==uploads)process.exitCode=1;
}finally{await b.close()}})().catch(e=>{console.error(e);process.exit(1)});

