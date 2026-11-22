// Run against the season site's preliminary-design/ URL with Playwright available to Node.
const { chromium } = require('playwright');
const assert = require('node:assert/strict');
const { createHash } = require('node:crypto');
const root = new URL(process.argv[2]);
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
  assert.equal(await page.locator('#release').inputValue(), release);
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
    for (const [query, release, motion] of [
      ['', '002','continuous-cycle-96'], ['?release=001','001','loaded-cycle-48'],
      ['?release=001&case=observed-reset-96','001','observed-reset-96'],
      ...['continuous-cycle-48','score-to-climb-front','score-to-climb-rear'].map(m => ['?release=002&case='+m,'002',m])
    ]) console.log(JSON.stringify(await check(page,query,release,motion)));
    await page.goto(new URL('?release=missing',root).href);
    await page.locator('[role=alert]').waitFor({state:'visible'});
    await page.selectOption('#release','001');
    await page.waitForURL(url => url.searchParams.get('release')==='001');
    assert.deepEqual(failures,[]);
  } finally {await browser.close();}
})().catch(e => {console.error(e);process.exitCode=1;});
