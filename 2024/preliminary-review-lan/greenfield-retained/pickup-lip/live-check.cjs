const {chromium}=require('%USERPROFILE%/repos/certaindeath/node_modules/playwright');
(async()=>{const b=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});try{const p=await b.newPage({viewport:{width:1440,height:1020}});p.on('pageerror',e=>{throw e});
await p.goto('http://192.168.1.129:8764/?case=intake');await p.waitForFunction(()=>document.querySelector('#liveMeasure')?.textContent.includes('flap'));if(!(await p.locator('h1').innerText()).includes('pickup lip'))throw Error('Old issue');
for(const frame of [0,80,150,250]){await p.locator('#time').evaluate((el,f)=>{el.value=f;el.dispatchEvent(new Event('input'))},frame);await p.waitForTimeout(100);console.log(frame,await p.locator('#liveMeasure').innerText());if(frame===80){await p.locator('#stage').scrollIntoViewIfNeeded();await p.locator('.viewer').screenshot({path:'pickup-lip-flap.png'})}}
await p.locator('#play').click();await p.waitForTimeout(350);await p.locator('#play').click();if(await p.locator('#time').inputValue()==='0')throw Error('stalled');console.log(await p.locator('#motion').inputValue());
const positions=await p.evaluate(()=>['.viewer','#resources','#weightCard','#powerCard'].map(s=>[s,document.querySelector(s).getBoundingClientRect().top]));console.log(JSON.stringify(positions));if(positions[1][1]<=positions[0][1]||positions[1][1]>=positions[2][1])throw Error('misplaced metrics');
}finally{await b.close()}})().catch(e=>{console.error(e);process.exitCode=1});

