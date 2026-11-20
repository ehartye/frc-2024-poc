const el = id => document.getElementById(id);
async function get(file) {
  const response = await fetch(file);
  if (!response.ok) throw Error(file);
  return response.json();
}
const [data,cad] = await Promise.all([get('passive-entry.json'),get('passive-cassette.json')]);
function renderCassette() {
  const canvas=el('cad'), ctx=canvas.getContext('2d'), yaw=Number(el('angle').value)*Math.PI/180;
  const faces=[];
  const palette={'molded tread':[218,155,65],carrier:[123,173,178],'welded spine':[186,202,208]};
  for(const group of cad.groups){
    const color=palette[group.name]??[214,221,221];
    const points=group.v.map(([x,y,z])=>{
      const u=(x-389)*Math.cos(yaw)+(y-198)*Math.sin(yaw);
      const v=-(x-389)*Math.sin(yaw)+(y-198)*Math.cos(yaw);
      return [u,-.85*(z-21)+.53*v,.85*v+.53*(z-21)];
    });
    for(let i=0;i<group.f.length;i+=3){
      const [a,b,c]=group.f.slice(i,i+3).map(j=>points[j]);
      const ab=b.map((x,j)=>x-a[j]),ac=c.map((x,j)=>x-a[j]);
      const normal=[ab[1]*ac[2]-ab[2]*ac[1],ab[2]*ac[0]-ab[0]*ac[2],ab[0]*ac[1]-ab[1]*ac[0]];
      const length=Math.hypot(...normal)||1;
      const shade=.68+.32*Math.abs((-.28*normal[0]-.68*normal[1]+.67*normal[2])/length);
      faces.push({points:[a,b,c],depth:(a[2]+b[2]+c[2])/3,color:'rgb('+color.map(x=>Math.round(x*shade)).join(',')+')'});
    }
  }
  let xmin=Infinity,xmax=-Infinity,ymin=Infinity,ymax=-Infinity;
  for(const f of faces)for(const [x,y] of f.points){xmin=Math.min(xmin,x);xmax=Math.max(xmax,x);ymin=Math.min(ymin,y);ymax=Math.max(ymax,y);}
  const zoom=Math.min((canvas.width-70)/(xmax-xmin),(canvas.height-34)/(ymax-ymin));
  ctx.clearRect(0,0,canvas.width,canvas.height);
  for(const f of faces.sort((a,b)=>a.depth-b.depth)){
    ctx.beginPath();
    f.points.forEach(([x,y],i)=>ctx[i?'lineTo':'moveTo'](canvas.width/2+(x-(xmin+xmax)/2)*zoom,canvas.height/2+(y-(ymin+ymax)/2)*zoom));
    ctx.closePath();ctx.fillStyle=f.color;ctx.fill();
  }
}
function drawBench(){
  const frames=data.bench[el('offset').value].trace;
  const [t,x,y,,lift,bottom,load,,phase]=frames[Number(el('time').value)];
  const ctx=el('plan').getContext('2d'), X=v=>86+(v-245)*1.08,Y=v=>220-v*.96;
  ctx.clearRect(0,0,940,440);
  ctx.save();ctx.strokeStyle='#3d6268';ctx.lineWidth=1;ctx.setLineDash([7,7]);
  ctx.beginPath();ctx.moveTo(32,Y(0));ctx.lineTo(918,Y(0));ctx.stroke();ctx.restore();
  for(const side of [-1,1]) {
    ctx.lineJoin='round';ctx.lineCap='round';ctx.strokeStyle='#90ccc0';ctx.lineWidth=13.5;ctx.beginPath();
    data.design.railPathMm.forEach(([a,b],i)=>ctx[i?'lineTo':'moveTo'](X(a),Y(b*side)));ctx.stroke();
  }
  ctx.beginPath();ctx.ellipse(X(x),Y(y),152.4*1.08,152.4*.96,0,0,2*Math.PI);
  ctx.lineWidth=50.8;ctx.strokeStyle='rgba(245,163,65,.87)';ctx.stroke();
  ctx.fillStyle='#d6ede8';ctx.beginPath();ctx.arc(X(x),Y(y),3,0,2*Math.PI);ctx.fill();
  el('phase').textContent=t.toFixed(2)+' s · '+phase+' · lift '+lift.toFixed(1)+' mm';
  el('y').textContent=y.toFixed(2)+' mm';el('bottom').textContent=bottom.toFixed(2)+' mm';el('force').textContent=load.toFixed(2)+' N';
}
function chooseOffset(initialize=false){
  const frames=data.bench[el('offset').value].trace;el('time').max=frames.length-1;
  if(initialize)el('time').value=frames.findIndex(row=>row[8]==='hold');
  drawBench();
}
const labels={clear:'Ready receiver',late:'Late receiver',misaligned:'Offset source','late-offset':'Late receiver and offset source','slow-receiver':'Slow receiver',directDispatch:'Direct dispatch control'};
for(const key of Object.keys(data.runs))el('case').add(new Option(labels[key],key));
function showCustody(){
  const r=data.runs[el('case').value];
  el('custody').replaceChildren(...Object.entries(r.pieces).map(([id,p])=>{
    const node=document.createElement('p'),name=document.createElement('strong');name.textContent=id;
    node.append(name,document.createElement('br'),'Blue 3 settled at '+p.timeS.toFixed(2)+' s. At horn: '+r.finalCustody[id]+'.');return node;
  }));
  el('reset').textContent='Blue 3 reset: '+(r.carrierReset?'yes':'no')+'. Blue 2 reset and ready: '+(r.receiverReady?'yes':'no')+'.';
}
el('case').addEventListener('change',showCustody);el('offset').addEventListener('change',()=>chooseOffset());
el('time').addEventListener('input',drawBench);el('angle').addEventListener('input',renderCassette);
renderCassette();chooseOffset(true);showCustody();
