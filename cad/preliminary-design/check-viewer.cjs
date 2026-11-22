// Run against the season site's preliminary-design/ URL with Playwright available to Node.
const { chromium } = require('playwright');
const assert = require('node:assert/strict');
const { createHash } = require('node:crypto');
const { readFileSync } = require('node:fs');
const root = new URL(process.argv[2]);
const baseline = process.argv[3] && readFileSync(process.argv[3],'utf8').trim().split(/\r?\n/).map(JSON.parse);
const sha = bytes => createHash('sha256').update(bytes).digest('hex');

async function check(page, query, release, motion) {
  const url = new URL(root);
  url.search = query;
  await page.goto(url.href);
  const frame = page.frameLocator('iframe');
  await frame.locator('#time').evaluate(el => new Promise((resolve, reject) => {
    const start = performance.now();
    const poll = () => el.oninput ? resolve() :
      performance.now() - start > 30000 ? reject(Error('Replay initialization timed out')) : setTimeout(poll, 50);
    poll();
  }));
  assert.equal(await page.locator('#release').count(), 0);
  assert.equal(new URL(await page.locator('#viewer').getAttribute('src'),root).pathname, new URL('releases/'+release+'/',root).pathname);
  assert.equal(new URL(await page.locator('#direct').getAttribute('href'), root).searchParams.get('case'), motion);
  const canvas = frame.locator('#canvas');
  const image = async () => sha(await canvas.screenshot());
  const initial = await image();
  await canvas.press('ArrowRight');
  assert.notEqual(await image(), initial, 'orbit must redraw geometry');
  await canvas.press('Shift+ArrowDown');
  const panned = await image();
  await canvas.press('+');
  assert.notEqual(await image(), panned, 'zoom must change projected size');
  await frame.locator('#reset').click();
  await frame.locator('#labels').check();
  await frame.locator('#labels').uncheck();
  await frame.locator('#floor').uncheck();
  await frame.locator('#floor').check();
  await frame.locator('#inventory tr[data-name]').first().click();
  assert.match(await frame.locator('#inspector').innerText(), /Why it is included/);
  await frame.getByText('Focus part', {exact:true}).click();
  await frame.getByText('Show assembly', {exact:true}).click();
  await frame.locator('#powerBars button').first().click();
  assert.ok(await frame.locator('#loads details[open]').count());
  await frame.locator('#group').selectOption({label:'Category'});
  await frame.locator('#subset').selectOption({index:1});
  await frame.locator('#group').selectOption({label:'Whole robot'});
  await frame.locator('#play').click();
  await frame.getByRole('button',{name:'Pause',exact:true}).click();
  await frame.locator('#time').evaluate(async el => {
    el.value = el.max;
    el.dispatchEvent(new Event('input'));
    await new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve)));
  });
  return { release, motion, initial, end:await frame.locator('#liveMeasure').innerText(),
    inventory:await frame.locator('#inventory tr[data-name]').count(),
    weight:sha(await frame.locator('#weightBody').innerText()), power:sha(await frame.locator('#powerBars').innerText()) };
}

(async () => {
  const browser = await chromium.launch({channel:process.env.BROWSER_CHANNEL || undefined,
    args:['--no-proxy-server','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
  try {
    const page = await browser.newPage({viewport:{width:1440,height:1000},deviceScaleFactor:1});
    const failures = [];
    page.on('pageerror', e => failures.push(e.message));
    page.on('response', response => { if (response.status() >= 400) failures.push(response.url()); });
    for (const motion of ['continuous-cycle-96','continuous-cycle-48','score-to-climb-front','score-to-climb-rear']) {
      const live = await check(page,motion==='continuous-cycle-96'?'':'?case='+motion,'003',motion);
      assert.ok(live.end.includes(motion.includes('climb')?'Power cut':'6/6 individual events'));
      if (baseline) {
        const prior = baseline.find(r=>r.release==='002' && r.motion===motion);
        for (const k of ['end','inventory','weight','power']) assert.equal(live[k],prior[k],motion+' '+k);
      }
      console.log(JSON.stringify(live));
    }
    for (const [old,motion] of [['001','loaded-cycle-48'],['002','continuous-cycle-48'],['003','continuous-cycle-48']]) {
      await page.goto(new URL('?release='+old+'&case='+motion,root).href);
      await page.waitForURL(u=>!u.searchParams.has('release') && u.searchParams.get('case')==='continuous-cycle-48');
      assert.match(await page.locator('#viewer').getAttribute('src'), /releases\/003\//);
    }
    await page.goto(root.href);
    const scene = page.frameLocator('#viewer');
    await scene.locator('#time').evaluate(el => new Promise(resolve => {
      function ready() { if (el.oninput) resolve(); else setTimeout(ready,50); } ready();
    }));
    for (const width of [1344,390]) {
      await page.setViewportSize({width,height:900});
      for (const fraction of [0,.33,.67,1]) {
        const edge = await scene.locator('#time').evaluate(async (el,fraction) => {
          el.value = Math.round(fraction * Number(el.max));
          el.dispatchEvent(new Event('input'));
          await new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve)));
          const [x,y] = current().positions[Number(el.value)];
          return Math.max(...data.corners.flatMap(([px,py,pz]) =>
            lastProjection([px-x,py-y,pz]).slice(0,2).map(Math.abs)));
        },fraction);
        assert.ok(edge < 1,'Speaker opening is clipped at width '+width+' and fraction '+fraction);
      }
    }
    await scene.locator('#fieldStatus[data-fixture="speaker"][data-ready="true"]').waitFor({timeout:180000});
    const counts = await scene.locator('#canvas').evaluate(() =>
      Object.fromEntries(['speakerPanel','speakerRamp','sensors'].map(k => [k,sceneGPU.fixtures[k].count/3])));
    assert.ok(counts.speakerPanel>100000 && counts.speakerRamp>5000 && counts.sensors>100,
      'Detailed CAD shell, ramp and sensor meshes must be installed');
    const shellImage = sha(await scene.locator('#canvas').screenshot());
    await scene.locator('#cutaway').check();
    assert.notEqual(sha(await scene.locator('#canvas').screenshot()),shellImage,'Speaker cutaway must reveal the interior');
    for (const approach of ['front','rear']) {
      const motion='score-to-climb-'+approach;
      await scene.locator('#motion').evaluate((select,key)=>{
        select.selectedIndex=data.cases.findIndex(c=>c.key===key);
        select.dispatchEvent(new Event('change'));
      },motion);
      await scene.locator('#fieldStatus[data-fixture="stage"][data-ready="true"]').waitFor({timeout:180000});
      const poses=await scene.locator('#time').evaluate(el=>{
        const names=['stageSteel','anchors','workingChain','otherChain'];
        const fixture=sceneGPU.fixtures,c=current(),poses=[];
        const originalDraw=gl.drawArrays;
        try{
          for(const t of [0,Math.round(Number(el.max)/2),Number(el.max)]){
            el.value=t;
            const draws={};
            gl.drawArrays=function(...args){
              const buffer=this.getParameter(this.ARRAY_BUFFER_BINDING);
              const name=names.find(n=>fixture[n].buffer===buffer);
              if(name)draws[name]={triangles:args[2]/3,bent:!!this.getUniform(program,locationMap.bend)};
              return originalDraw.apply(this,args);
            };
            render();
            const offset=gl.getUniform(program,gl.getUniformLocation(program,'offsets[12]'));
            const expected=c.chain[t][12][2]-c.chain[0][12][2];
            poses.push({draws,dz:offset[2],expected});
          }
        }finally{gl.drawArrays=originalDraw}
        return poses;
      });
      for(const {draws,dz,expected} of poses){
        assert.deepEqual(Object.keys(draws).sort(),['anchors','otherChain','stageSteel','workingChain']);
        assert.ok(draws.stageSteel.triangles>100000 && draws.anchors.triangles>1000 && draws.workingChain.triangles>50000);
        assert.equal(draws.otherChain.triangles,2*draws.workingChain.triangles);
        assert.deepEqual(Object.keys(draws).filter(k=>draws[k].bent),['workingChain']);
        assert.ok(Math.abs(dz-expected)<.001,'CAD chain follows the recorded midpoint displacement');
      }
      assert.equal(poses[0].dz,0);
      assert.ok(poses.at(-1).dz < -45,'Loaded chain sags at power cut');
    }
    await page.goto(new URL('?release=missing',root).href);
    await page.locator('[role=alert]').waitFor({state:'visible'});
    assert.deepEqual(failures,[]);
  } finally {await browser.close();}
})().catch(e => {console.error(e);process.exitCode=1;});
