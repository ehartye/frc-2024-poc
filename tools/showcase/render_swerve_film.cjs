// Render the published glTF at recorded timestamps. Requires playwright-core and FFmpeg with VP9.
const {chromium} = require("playwright-core");
const {spawn} = require("node:child_process");
const {once} = require("node:events");
const fs = require("node:fs");
const path = require("node:path");
const http = require("node:http");

async function render(docs, browserPath, ffmpeg, output) {
  const receipt = JSON.parse(fs.readFileSync(path.join(docs,"media/swerve-r7.json")));
  const assets = {"/viewer.js":"vendor/model-viewer-4.1.0.min.js","/robot.glb":"media/swerve-r7.glb"};
  const html = `<!doctype html><html><head><meta charset="utf-8"><style>
  *{box-sizing:border-box}body{margin:0;color:white;background:#162f60;font:17px "Segoe UI",sans-serif}
  model-viewer{width:100vw;height:100vh;background:radial-gradient(ellipse,#345789,#122950)}
  header,footer{position:absolute;left:22px;right:22px;pointer-events:none}header{top:16px;font-weight:650}
  footer{bottom:14px;display:flex;justify-content:space-between;background:#132a54ec;padding:9px 12px;border-left:4px solid #f7bd54}
  #time{font-variant-numeric:tabular-nums;color:#f7cf80}</style></head><body>
  <model-viewer src="/robot.glb" camera-orbit="42deg 68deg auto" max-camera-orbit="auto auto 7m" shadow-intensity=".75" exposure="1.15" interaction-prompt="none"></model-viewer>
  <header>1780 &nbsp;/&nbsp; R7 display · D1 simulated pickup and shot</header><footer><span id="state"></span><span id="time"></span></footer>
  <script type="module" src="/viewer.js"></script></body></html>`;
  const server = http.createServer((req,res)=>{
    if (req.url==="/") {res.setHeader("Content-Type","text/html");res.end(html);return;}
    if (!assets[req.url]) {res.writeHead(404);res.end();return;}
    res.setHeader("Content-Type",req.url==="/viewer.js"?"application/javascript":"model/gltf-binary");
    fs.createReadStream(path.join(docs,assets[req.url])).pipe(res);
  });
  await new Promise(resolve=>server.listen(0,"127.0.0.1",resolve));
  let browser, encoder;
  try {
    browser=await chromium.launch({headless:true,executablePath:browserPath,args:["--use-angle=swiftshader","--enable-unsafe-swiftshader"]});
    const page=await browser.newPage({viewport:{width:800,height:520}});
    await page.goto("http://127.0.0.1:"+server.address().port);
    await page.waitForFunction(()=>document.querySelector("model-viewer").loaded);
    const fps=20, slowAt=receipt.releaseS-.50, slowRate=.125;
    const slowStart=slowAt-receipt.sourceStartS;
    const openingFrameTime=slowStart+(receipt.openingS-slowAt)/slowRate;
    const duration=openingFrameTime+receipt.simultaneousSensorS-receipt.openingS+receipt.film.holdS;
    encoder=spawn(ffmpeg,["-hide_banner","-loglevel","error","-f","image2pipe","-framerate",String(fps),"-c:v","mjpeg","-i","pipe:0",
      "-c:v","libvpx-vp9","-threads","1","-b:v","900k","-pix_fmt","yuv420p","-y",output],{stdio:["pipe","ignore","inherit"]});
    const completed=once(encoder,"close");
    const frames=Math.ceil(duration*fps)+1;
    for(let i=0;i<frames;i++){
      const t=i/fps;
      const slowed=t>=slowStart && t<openingFrameTime;
      const match=t<slowStart ? receipt.sourceStartS+t : slowed ? slowAt+(t-slowStart)*slowRate :
        Math.min(receipt.simultaneousSensorS,receipt.openingS+t-openingFrameTime);
      await page.evaluate(async ({match,slowed,receipt})=>{
        const v=document.querySelector("model-viewer");
        v.animationName=receipt.clip;
        v.currentTime=Math.min(v.duration-.00001,Math.max(0,match-receipt.sourceStartS));
        if (match>=receipt.openingS) {
          const u=Math.min(1,(match-receipt.openingS)/(receipt.simultaneousSensorS-receipt.openingS));
          v.cameraTarget=receipt.openingCenter.map((x,i)=>(x+(receipt.counterCenter[i]-x)*u)+"m").join(" ");
          v.cameraOrbit="12deg 77deg "+(3.8-2.4*u)+"m";
        } else if(slowed) {
          v.cameraTarget=".61m 1.09m 0m";
          v.cameraOrbit="12deg 84deg 5.85m";
        } else {
          v.cameraTarget="0m .55m 0m";
          v.cameraOrbit="42deg 68deg 3.45m";
        }
        v.jumpCameraToGoal();
        const atCounter=match>=receipt.simultaneousSensorS;
        document.querySelector("#state").textContent=atCounter?"Both modeled Speaker sensor pairs blocked":
          match>=receipt.openingS?"Opening cleared · NOTE inside Speaker":match>=receipt.releaseS?"NOTE in flight":
          slowed?"Feeding the launcher":match>=receipt.seatedS?"Lift, turn and carry":match>=receipt.firstBilateralS?"Intake seats the NOTE":"Drive over floor NOTE";
        document.querySelector("#time").textContent=(atCounter?"Event held":slowed?"1/8 speed":"Real time")+"   |   "+match.toFixed(3)+" s";
        await new Promise(resolve=>requestAnimationFrame(()=>requestAnimationFrame(resolve)));
      },{match,slowed,receipt});
      const frame=await page.screenshot({type:"jpeg",quality:87});
      if(!encoder.stdin.write(frame)) await once(encoder.stdin,"drain");
    }
    encoder.stdin.end();
    if ((await completed)[0]!==0) throw Error("Video encoding failed");
    console.log(frames+" frames of D1 motion at "+fps+" fps");
  } finally {
    if(encoder && encoder.exitCode===null) encoder.kill();
    if(browser) await browser.close();
    server.close();
  }
}
render(...process.argv.slice(2)).catch(err=>{console.error(err);process.exitCode=1});
