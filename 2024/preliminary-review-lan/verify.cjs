const {chromium}=require('%USERPROFILE%/repos/certaindeath/node_modules/playwright');
(async()=>{const browser=await chromium.launch({channel:'msedge',headless:true,args:['--no-proxy-server','--use-angle=swiftshader','--enable-unsafe-swiftshader']});try{
const p=await browser.newPage({viewport:{width:1400,height:950}});p.on('pageerror',e=>console.log('PAGE ERROR',e.message));
await p.goto('http://192.168.1.129:8764/');await p.locator('#inventory tr[data-name]').first().waitFor({timeout:15000});console.log(await p.locator('#validation').innerText());
await p.locator('#labels').check();console.log('labels',await p.locator('#names span').count());await p.locator('#labels').uncheck();
const c=p.locator('#canvas');await c.focus();await p.keyboard.press('ArrowLeft');await p.keyboard.press('Shift+ArrowDown');await p.keyboard.press('Home');
await p.locator('#play').click();await p.waitForTimeout(500);await p.locator('#play').click();console.log('stroke',await p.locator('#fraction').innerText());
await p.locator('#group').selectOption('Category');await p.locator('#subset').selectOption('electrical');console.log('electrical',await p.locator('#summary').innerText());
await p.locator('#group').selectOption('Custom subsystem');await p.locator('#inventory input').first().check();await p.locator('#subname').fill('Test assembly');await p.locator('#save').click();console.log('saved',await p.locator('#subset').inputValue());await p.locator('#group').selectOption('Whole robot');await p.locator('#time').fill('0');await p.screenshot({path:'review-desktop.png'});console.log('ok');
}finally{await browser.close()}})().catch(e=>{console.error(e);process.exitCode=1});
