const {chromium}=require('%USERPROFILE%/repos/certaindeath/node_modules/playwright');
const fs=require('fs'),path=require('path');
(async()=>{const browser=await chromium.launch({channel:'msedge',args:['--no-proxy-server','--use-angle=swiftshader','--enable-unsafe-swiftshader']});try{
 const page=await browser.newPage({viewport:{width:1440,height:1000}}), errors=[];
 page.on('pageerror',e=>errors.push(e.message));await page.goto('http://192.168.1.129:8764/');
 await page.waitForFunction(()=>document.querySelector('h1')?.textContent.includes('P3')&&document.querySelectorAll('#inventory tr[data-name]').length===126);
 await page.check('#labels');
 async function check(){
   await page.waitForFunction(()=>document.querySelector('.callout-diagram polyline'));
   return page.evaluate(()=>{
     const rows=[...document.querySelectorAll('.callout-diagram g')].filter(g=>g.style.display!=='none').map(g=>{
       const p=g.querySelector('polyline').points, t=g.querySelector('text'),bb=t.getBoundingClientRect();
       if(p.length!==3||Math.abs(p[1].y-p[2].y)>.01||p[0].x>=p[1].x||p[1].x>=p[2].x)throw Error('Non-horizontal or reverse leader');
       return {name:g.dataset.part,row:p[0].y,tip:p[2].y,l:bb.left,r:bb.right};
     });
     if(rows.some((a,i)=>i&&(a.row-rows[i-1].row<23.9||a.tip-rows[i-1].tip<6.9)))throw Error('Overlapping layout');return rows;
   });
 }
 let first=await check();const names=new Set(first.map(r=>r.name));while(await page.locator('#labelNext').isEnabled()){await page.click('#labelNext');await page.waitForTimeout(90);for(const r of await check())names.add(r.name)}
 const cv=page.locator('#canvas');await cv.focus();await page.keyboard.press('ArrowRight');await page.keyboard.press('ArrowUp');await page.waitForTimeout(120);await check();
 const stop=page.locator('#inventory tr[data-name="Rear stop bracket · left"]');await stop.click();await page.waitForTimeout(150);await check();
 const inspected=await page.locator('#inspector h2').textContent();if(!inspected.includes('Rear stop bracket'))throw Error(inspected);
 await page.locator('#stage').scrollIntoViewIfNeeded();await page.locator('#stage').screenshot({path:path.join(__dirname,'retained-cassette-labels.png')});
 if(errors.length)throw Error(errors.join(';'));const report={visibleLabels:first.length,partsReachedByPaging:names.size,inspected,errorCount:errors.length};fs.writeFileSync(path.join(__dirname,'viewer-check.json'),JSON.stringify(report));console.log(report);
}finally{await browser.close()}})().catch(e=>{console.error(e);process.exitCode=1});

