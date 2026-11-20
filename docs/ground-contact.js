const el = id => document.getElementById(id);
const poses = {
  seat:[241.5,0,'Both tread surfaces intersect the nominal foam ring by about 1.38 mm. The closest backing belt has only 0.11 mm of modeled clearance.'],
  external:[741.812,0,'At the original external dock, the NOTE clears the entire parked envelope by at least 25.4 mm. Neither tread can acquire it there.'],
  offset:[241.5,3,'A 3 mm lateral error separates one tread and intersects the opposite front pulley by about 1.8 mm along the insertion stroke.'],
  raised:[505,0,'With the NOTE raised 8 mm, withdrawal crosses the front cover by about 0.8 mm. The revised sequence restores floor support before this part of the stroke.']
};
const labels={clear:'Receiver ready',late:'110 s receiver delay',misaligned:'3.5 in source offset','late-offset':'112 s delay and 3 in offset','slow-receiver':'120 s receiver delay',
  directDispatch:'120 s delay · earlier receiver staging',externalDock:'Original external dock',offset3mm:'3 mm residual alignment error',raisedWithdrawal:'Withdraw while raised',braked:'Withdraw with braked treads',empty:'Empty supply',noGrip:'No holding force',sweptAway:'First NOTE displaced by traffic',unreserved:'No lane reservation',rectangularReservation:'Reserve the whole approach rectangle'};
const canvas=document.querySelector('canvas'),ctx=canvas.getContext('2d');
let data;
function draw(){
  const [x,y,caption]=poses[el('pose').value];el('caption').textContent=caption;
  const sx=v=>v+30, sy=v=>232-v;
  ctx.fillStyle='#16333c';ctx.fillRect(0,0,1000,465);ctx.lineWidth=1;
  for(let i=0;i<1000;i+=100){
    ctx.strokeStyle='#35515a';ctx.beginPath();ctx.moveTo(sx(i),0);ctx.lineTo(sx(i),465);ctx.stroke();
  }
  for(const [name,segments] of Object.entries(data.section)){
    ctx.strokeStyle=['534','535'].includes(name)?'#47e0c6':'#93a6ac';ctx.lineWidth=['534','535'].includes(name)?3:1.5;
    ctx.beginPath();for(const [a,b,c,d] of segments){ctx.moveTo(sx(a),sy(b));ctx.lineTo(sx(c),sy(d));}ctx.stroke();
  }
  ctx.strokeStyle='#d9e2d7';ctx.setLineDash([7,6]);ctx.beginPath();ctx.moveTo(sx(538.512),0);ctx.lineTo(sx(538.512),465);ctx.stroke();ctx.setLineDash([]);
  ctx.fillStyle='#e2e9e6';ctx.font='17px system-ui';ctx.fillText('Front cover projection',sx(538.512)+8,28);
  ctx.beginPath();ctx.arc(sx(x),sy(y),152.4,0,2*Math.PI);ctx.strokeStyle='rgba(255,164,65,.64)';ctx.lineWidth=50.8;ctx.stroke();
  ctx.strokeStyle='#ffc786';ctx.lineWidth=1.5;
  for(const radius of [127,177.8]){ctx.beginPath();ctx.arc(sx(x),sy(y),radius,0,2*Math.PI);ctx.stroke();}
  ctx.fillStyle='#f3f5ee';ctx.fillText('Treads',sx(10),sy(190));ctx.fillText('Robot +X →',20,450);
}
function show(){
  const r=data.results[el('run').value];
  for(const key of ['deposits','received','stored'])el(key).textContent=r[key].length;
  const owners=Object.entries(r.custody).map(([id,owner])=>id+' · '+owner).join(' / ');
  el('state').textContent=(r.stop==='horn'?'At the 150 s horn.':'Stopped. '+r.stop+'.')+' '+(owners||'No supplied NOTES.');
  const rows=r.history.map(e=>{
    const tr=document.createElement('tr');
    for(const value of [e.timeS.toFixed(2)+' s',e.actor,e.reason||e.action,e.piece||'—']){
      const cell=document.createElement('td');cell.textContent=value;tr.append(cell);
    }return tr;
  });
  document.querySelector('tbody').replaceChildren(...rows);
}
try{
  const response=await fetch('ground-contact.json');if(!response.ok)throw Error('Missing evidence');
  data=await response.json();
  for(const id of Object.keys(data.results))el('run').add(new Option(labels[id.split('/').at(-1)],id));
  el('run').value='slow-receiver';el('run').onchange=show;el('pose').onchange=draw;draw();show();
}catch(error){el('state').textContent='The registered contact evidence could not be loaded.';}
