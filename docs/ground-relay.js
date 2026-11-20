const el = id => document.getElementById(id);
const policy = el('policy'), condition = el('condition'), slider = el('time');
const canvas = document.querySelector('canvas'), ctx = canvas.getContext('2d');
const names = {clear:'Receiver ready',late:'110 s receiver delay',misaligned:'Second NOTE offset',
  'late-offset':'Held out · 112 s and offset','slow-receiver':'Held out · 120 s delay',
  preposition:'Preposition only','exposed-store':'Exposed storage route','direct-dispatch':'Direct dispatch',
  empty:'Empty supply',dropOutside:'Release outside pad',zeroRetention:'No retention force',
  sweptAway:'NOTE displaced by traffic',unreserved:'No lane reservation',oneInch:'One-inch clearance'};
let data, run, anim = 0;
function stop(){cancelAnimationFrame(anim);anim=0;el('play').textContent='Play';}
function chooseCase(){
  stop(); run=data.cases[policy.value+'/'+condition.value];slider.value=150;draw();
}
function populate(){
  const keys=Object.keys(data.cases).filter(k=>k.startsWith(policy.value+'/')).map(k=>k.split('/')[1]);
  condition.replaceChildren(...keys.map(k=>new Option(names[k],k)));
  condition.value=keys.includes('slow-receiver')?'slow-receiver':keys.includes('direct-dispatch')?'direct-dispatch':keys[0];
  chooseCase();
}
function point(rows,t){
  let i=0;while(i+1<rows.length && rows[i+1][0]<=t)i++;
  const a=rows[i], b=rows[i+1]||a, u=b===a?0:(t-a[0])/(b[0]-a[0]);
  return [1,2].map(j=>a[j]+u*(b[j]-a[j]));
}
function draw(){
  const t=+slider.value, i=Math.min(run.samples.length-1,Math.floor((t-15)*2));
  const s=run.samples[i], next=run.samples[i+1]||s;
  const u=next===s?0:(t-s[0])/(next[0]-s[0]);
  const pos=Object.fromEntries(['B3','B2'].map((name,k)=>[name,[1+2*k,2+2*k].map(j=>s[j]+u*(next[j]-s[j]))]));
  for(const [name,rows] of Object.entries(data.peers))pos[name]=point(rows,(t-15)/data.dt);
  const events=run.history.filter(e=>e.t<=t+1e-7);
  const scale=canvas.width/data.field[0], y=v=>canvas.height-v*scale;
  ctx.fillStyle='#16333c';ctx.fillRect(0,0,canvas.width,canvas.height);
  ctx.strokeStyle='#34525b';ctx.lineWidth=1;
  for(let x=0;x<data.field[0];x+=36){ctx.beginPath();ctx.moveTo(x*scale,0);ctx.lineTo(x*scale,canvas.height);ctx.stroke();}
  for(let v=0;v<data.field[1];v+=36){ctx.beginPath();ctx.moveTo(0,y(v));ctx.lineTo(canvas.width,y(v));ctx.stroke();}
  for(const stage of data.protectedStages){
    ctx.beginPath();stage.forEach(([x,v],j)=>j?ctx.lineTo(x*scale,y(v)):ctx.moveTo(x*scale,y(v)));
    ctx.closePath();ctx.fillStyle='#456169';ctx.fill();ctx.strokeStyle='#8ca1a6';ctx.stroke();
  }
  ctx.setLineDash([6,4]);ctx.strokeStyle='#f3e1a0';ctx.lineWidth=2;
  for(const [index,[x,v]] of data.pads.entries()){
    if(policy.value==='serial' && index>0)continue;
    ctx.beginPath();ctx.arc(x*scale,y(v),10*scale,0,2*Math.PI);ctx.stroke();
  }
  ctx.setLineDash([]);
  for(const [name,[x,v]] of Object.entries(pos)){
    const bounds=name==='B3'?data.box:[-17,-17,17,17];
    ctx.fillStyle=name==='B3'?'#39ccb5':name==='B2'?'#a6d5ec':name[0]==='R'?'#ad706d':'#638aa2';
    ctx.fillRect((x+bounds[0])*scale,y(v+bounds[3]),(bounds[2]-bounds[0])*scale,(bounds[3]-bounds[1])*scale);
    ctx.fillStyle='#10252c';ctx.textAlign='center';ctx.font='bold 18px system-ui';ctx.fillText(name,x*scale,y(v)+6);
  }
  for(const [item,start] of Object.entries(run.initialPositions)){
    let xy=start;
    for(const e of events.filter(e=>e.piece===item)){
      if(e.kind==='acquired'||e.kind==='received')xy=[pos[e.actor][0]+data.port,pos[e.actor][1]];
      if(e.xy)xy=e.xy;
      if(['lost','empty after traffic sweep','outside search band'].includes(e.kind))xy=null;
    }
    if(xy){ctx.beginPath();ctx.arc(xy[0]*scale,y(xy[1]),data.ringRadius*scale,0,2*Math.PI);
      ctx.strokeStyle='#ffab4b';ctx.lineWidth=4;ctx.stroke();}
  }
  el('clock').textContent=t.toFixed(1)+' s';
  for(const [id,kind] of [['deposits','floor deposit'],['received','received'],['stored','downstream floor']])
    el(id).textContent=events.filter(e=>e.kind===kind).length;
  const actorEvents=events.filter(e=>e.kind!=='peer crossed piece projection');
  const last=actorEvents.at(-1);
  el('state').textContent=t===150?
    (run.cutoffReason==='clock'?'At the horn.':run.cutoffReason+'.')+' Blue 3 '+(run.reset?'reset.':'still in cycle.')+' Blue 2 '+(run.partnerReset?'ready.':'still in cycle.'):
    (last?last.actor+' · '+last.kind:'TELEOP begins');
  el('events').replaceChildren(...events.slice().reverse().map(e=>{
    const li=document.createElement('li');
    li.textContent=e.t.toFixed(2)+' s · '+e.actor+' · '+e.kind+(e.piece?' · '+e.piece:'')+(e.detail?' · '+e.detail:'');
    return li;
  }));
  if(!anim)el('play').textContent=t>=150?'Replay':'Play';
}
el('play').onclick=()=>{
  if(anim){stop();return;}
  if(+slider.value>=150)slider.value=15;
  let previous=performance.now();el('play').textContent='Pause';
  function step(now){
    slider.value=Math.min(150,+slider.value+(now-previous)*.006);previous=now;draw();
    if(+slider.value>=150){stop();el('play').textContent='Replay';return;}
    anim=requestAnimationFrame(step);
  }
  anim=requestAnimationFrame(step);
};
slider.oninput=()=>{stop();draw();};
policy.onchange=populate;condition.onchange=chooseCase;
try{
  const res=await fetch('ground-relay.json');if(!res.ok)throw Error(res.status);
  data=await res.json();populate();
}catch(e){el('state').textContent='The saved relay observations could not be loaded.';el('play').disabled=true;}
