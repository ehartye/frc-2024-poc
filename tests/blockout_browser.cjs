const assert=require('node:assert/strict'),http=require('node:http'),fs=require('node:fs'),path=require('node:path');
const root=path.resolve(__dirname,'../tools/blockout');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
(async()=>{
 const server=http.createServer((req,res)=>{
  const file=path.resolve(root,'.'+decodeURIComponent(req.url.split('?')[0]==='/'?'/index.html':req.url.split('?')[0]));
  if(!file.startsWith(root+path.sep)){res.writeHead(403).end();return;}
  try{res.setHeader('Content-Type',({'.html':'text/html','.js':'text/javascript','.json':'application/json'})[path.extname(file)]||'text/plain');res.end(fs.readFileSync(file));}catch{res.writeHead(404).end();}
 });
 await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));let browser;
 try{
  browser=await chromium.launch({headless:true,...(process.env.CHROMIUM_EXECUTABLE?{executablePath:process.env.CHROMIUM_EXECUTABLE}:{}),args:['--enable-unsafe-swiftshader']});
  for(const viewport of [{width:1280,height:900},{width:390,height:844}]){
   const page=await browser.newPage({viewport}),errors=[],external=[];page.on('pageerror',e=>errors.push(e.message));
   await page.route('**/*',route=>{const u=new URL(route.request().url());if(u.hostname==='127.0.0.1')return route.continue();external.push(u.hostname);return route.abort();});
   await page.goto(`http://127.0.0.1:${server.address().port}/`);await page.waitForFunction(()=>window.blockoutReady&&window.rehearsalReady);
   const sample=await page.evaluate(()=>({near:blockoutOverlapTest({x:0,y:0,heading:0},{x:.5,y:0,heading:0}),far:blockoutOverlapTest({x:0,y:0,heading:0},{x:2,y:0,heading:0}),edge:blockoutOverlapTest({x:0,y:0,heading:0},{x:.86,y:0,heading:0}),total:rehearsalPlan.total,budget:rehearsalPlan.drive+rehearsalPlan.turn+rehearsalPlan.action,overflow:document.documentElement.scrollWidth>innerWidth,canvas:document.querySelector('canvas').width>0}));
   assert.equal(sample.near,true);assert.equal(sample.far,false);assert.equal(sample.edge,false);assert.ok(sample.total>0);assert.ok(Math.abs(sample.total-sample.budget)<1e-9);assert.equal(sample.overflow,false);assert.equal(sample.canvas,true);
   await page.locator('#rehearse').click();await page.waitForFunction(()=>rehearsalState.time>.1);await page.locator('#pauseRoute').click();const paused=await page.evaluate(()=>rehearsalState.time);await page.waitForTimeout(100);assert.equal(await page.evaluate(()=>rehearsalState.time),paused);
   await page.locator('#clearRoute').click();assert.equal(await page.evaluate(()=>rehearsalPlan.route.length),1);assert.equal(await page.evaluate(()=>rehearsalPlan.total),0);
   assert.deepEqual(errors,[]);assert.deepEqual(external,[]);await page.close();console.log(`Offline 3D/rehearsal checks passed at ${viewport.width}x${viewport.height}`);
  }
 }finally{if(browser)await browser.close();await new Promise(resolve=>server.close(resolve));}
})().catch(e=>{console.error(e.message);process.exitCode=1;});
