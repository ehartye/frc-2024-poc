const {chromium}=require('%USERPROFILE%/repos/certaindeath/node_modules/playwright');
const fs=require('node:fs');
(async()=>{
 const b=await chromium.launch({channel:'msedge',headless:true,args:['--no-proxy-server','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const p=await b.newPage(),errors=[],rows=[];p.on('pageerror',e=>errors.push(e.message));
 try{
  for(const [search,key] of [['?concept=guided&case=guided-front','guided-front'],['?case=guided-rear','guided-rear'],['?case=guided-service','guided-service'],['','loaded-cycle-48'],['?concept=field&case=field-guide','field-guide']]){
   await p.goto('http://192.168.1.129:8764/'+search);await p.waitForFunction(()=>typeof document.querySelector('#time').oninput==='function',null,{timeout:60000});
   if(new URL(p.url()).searchParams.get('case')!==key)throw Error(p.url());
   await p.locator('#time').evaluate(el=>{el.value=el.max;el.dispatchEvent(new Event('input'))});
   if(['guided-front','guided-rear'].includes(key))await p.waitForFunction(()=>document.querySelector('#liveMeasure').textContent.includes('Power cut'));
   if(key==='guided-service')await p.waitForFunction(()=>document.querySelector('#liveMeasure').textContent.includes('370 / 370'));
   const end=await p.locator('#liveMeasure').innerText(),weights=await p.locator('#resources').innerText();
   if(['guided-front','guided-rear'].includes(key)&&(!end.includes('Power cut')||!weights.includes('55.963')||!weights.includes('10.9')))throw Error(end+' '+weights);
   if(key==='guided-front'){
    await p.locator('#inventory tr').filter({hasText:'Hook anchor compression tube'}).first().click();
    if(!(await p.locator('#inspector').innerText()).includes('forward eye'))throw Error('Inspector failed');
   }
   rows.push({url:p.url(),title:await p.locator('h1').innerText(),end});
  }
  if(errors.length)throw Error(errors.join('; '));fs.writeFileSync('verified-view.json',JSON.stringify(rows,null,2));console.log(rows)
 }finally{await b.close()}
})().catch(e=>{console.error(e);process.exitCode=1})


