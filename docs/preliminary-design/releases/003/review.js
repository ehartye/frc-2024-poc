function weightLedger(){
 const root=$('weightBody');root.replaceChildren();
 const limit=56.69904625,low=data.inspection[0],high=data.inspection[1];
 const heading=text('p','Inspection allowance '+limit.toFixed(2)+' kg. Upper estimate '+high.toFixed(2)+' kg, with '+signed(limit-high,'kg',2)+' headroom.');
 root.append(heading);
 const ruler=document.createElement('div');ruler.className='weight-ruler';
 const width=Math.max(1,Math.abs(low-limit),Math.abs(high-limit))*1.08,start=limit-width,end=limit+width;const band=document.createElement('i');band.style.left=((low-start)/(end-start)*100)+'%';band.style.width=((high-low)/(end-start)*100)+'%';ruler.append(band,text('b','Limit '+limit.toFixed(3)+' kg'));root.append(ruler);
 const [tab,body]=table(['Inspection allocation','Low, kg','High, kg']);for(const row of data.mass.filter(r=>r.kg[1]>0)){
 const tr=body.insertRow();tr.insertCell().textContent=row.id;for(const v of row.kg)tr.insertCell().textContent=v.toFixed(3)}
 const total=body.insertRow();total.className='grand-total';total.insertCell().textContent='Inspection total';for(const v of data.inspection)total.insertCell().textContent=v.toFixed(3);root.append(tab);
 const details=document.createElement('details');details.append(text('summary','Running configuration · '+range(data.running,2)+' kg'));details.append(text('p','Add the battery, one game piece and the finished bumper scenario to the inspection estimate.'));
 const [ta,bd]=table(['Additional article','Low, kg','High, kg']);for(const r of data.details.filter(r=>['battery','one NOTE','finished bumper scenario'].includes(r.name))){const tr=bd.insertRow();tr.insertCell().textContent=r.name;for(const v of r.kg)tr.insertCell().textContent=v.toFixed(3)}details.append(ta);root.append(details)
}
function drawPowerBars(){
 const e=duty(),holder=$('powerBars');holder.replaceChildren();const max=e.batterySideA[1];
 for(const [i,line] of e.lines.entries()){
 const b=document.createElement('button');b.className='load-bar';b.setAttribute('aria-label','Inspect '+line.name);
 const caption=text('span',line.name),val=text('strong',line.amps[1].toFixed(1)+' A'),track=document.createElement('i'),fill=document.createElement('i');fill.style.width=Math.max(.5,100*line.amps[1]/Math.max(1,max))+'%';fill.style.background=['#2f66b0','#328a8a','#dfac29'][i%3];track.append(fill);b.append(caption,val,track);
 b.onclick=()=>{const target=$('loads').children[i+1];target.open=true;target.scrollIntoView({block:'nearest'});isolated=null;marked=line.members.slice();selected=null;draw()};holder.append(b)
 }
}

/* Selected-duty, battery-side load ledger and role-based commissioning checks. */
function signed(v,unit,digits){return (v>=0?'+':'−')+Math.abs(v).toFixed(digits)+' '+unit}
function resourcePanel(){
 const e=duty(),limit=56.69904625,worst=data.inspection[1],voc=e.openCircuitV??(e.loadedBusV[0]+e.resistance*e.batterySideA[1]),r=e.resistance,minimum=e.requiredMinV,available=(voc-minimum)/r,imax=e.batterySideA[1];
 const pAt=i=>i*(voc-r*i),peak=Math.max(pAt(e.batterySideA[0]),pAt(imax),e.batterySideA[0]<=voc/(2*r)&&imax>=voc/(2*r)?pAt(voc/(2*r)):0);
 weightLedger();drawPowerBars();const total=$('resources');total.replaceChildren();
 function meter(label,value,message,used,capacity,negative){const block=document.createElement('section');block.className='budget-meter';block.append(text('span',label),text('b',value));const bar=document.createElement('progress');bar.max=Math.max(used,capacity)*1.04;bar.value=used;bar.setAttribute('aria-label',label);block.classList.toggle('over',negative);block.append(bar,text('small',message));const link=text('a',label==='Inspection weight'?'Weight ledger ↓':'Power ledger ↓');link.href=label==='Inspection weight'?'#weightCard':'#powerCard';block.append(link);total.append(block)}
 meter('Inspection weight',data.inspection[0].toFixed(3)+'–'+worst.toFixed(3)+' kg','Limit '+limit.toFixed(3)+' kg · headroom '+signed(limit-worst,'kg',3)+' high / '+signed(limit-data.inspection[0],'kg',3)+' low',worst,limit,worst>limit);
 meter('Battery voltage under load',e.loadedBusV[0].toFixed(3)+' V minimum','Requirement '+minimum.toFixed(2)+' V · headroom '+signed(e.loadedBusV[0]-minimum,'V',3),voc-e.loadedBusV[0],voc-minimum,e.loadedBusV[0]<minimum);
 meter('Battery-side demand',imax.toFixed(1)+' A · '+(peak/1000).toFixed(2)+' kW','Pack budget at '+minimum.toFixed(1)+' V: '+available.toFixed(1)+' A / '+(minimum*available/1000).toFixed(2)+' kW',imax,available,imax>available);
 const mount=$('loads');mount.replaceChildren(text('p',e.case+' · '+voc.toFixed(1)+' V open circuit · '+(1000*r).toFixed(1)+' mΩ battery model. Select a load to highlight its components.'));
 for(const line of e.lines){
  const d=document.createElement('details');const s=document.createElement('summary');s.textContent=line.name+'　 '+range(line.amps,1)+' A';d.append(s);
  const minW=line.amps[0]*e.loadedBusV[0],maxW=line.amps[1]*e.loadedBusV[1];d.append(text('p','Battery input envelope '+Math.floor(minW)+'–'+Math.ceil(maxW)+' W · pack sag contribution '+range(line.amps.map(i=>r*i),3)+' V. Power bounds combine voltage and current extrema.'));
  const group=text('button','Locate this load');group.onclick=()=>{isolated=null;selected=null;marked=line.members.slice();draw();$('stage').scrollIntoView({block:'nearest'});};d.append(group);
  const members=document.createElement('div');members.className='member-buttons';line.members.forEach(n=>{if(data.bodies[n]){const b=text('button',n);b.onclick=()=>{isolated=null;inspect(n);$('stage').scrollIntoView({block:'nearest'})};members.append(b)}});d.append(members);d.append(text('p',line.members.length>1?'Budgeted as a shared load. Individual current split is unavailable.':'Assigned to this component in the duty model.'));mount.append(d)
 }
 const v=$('required');v.replaceChildren();const names=new Set(entries.map(([n])=>n));const motors=Array.from(names).filter(n=>/^Swerve (drive|steer) motor/.test(n));
 const expected=data.required||[];
 let missing=0;for(const [role,list] of expected){const absent=list.filter(n=>!names.has(n));missing+=absent.length;const row=document.createElement('div');row.className='role-row '+(absent.length?'warn':'');row.append(text('strong',absent.length?'Missing':'Present'),text('span',role+'　 '+(list.length-absent.length)+'/'+list.length));if(absent.length)row.append(text('small',absent.join(', ')));v.append(row)}
 const warns=$('warningText');warns.textContent=(missing?missing+' missing required representations. ':'Required component counts present. ')+(worst>limit?'High estimate exceeds inspection weight. ':'')+(e.loadedBusV[0]<minimum?'Selected duty falls below voltage requirement. ':'')+(data.focus||'');
}


'use strict';
const $=id=>document.getElementById(id);
const text=(tag,value)=>Object.assign(document.createElement(tag),{textContent:value});
const options=(sel,values)=>sel.replaceChildren(...values.map(x=>new Option(x,x)));
const range=(r,n=2)=>r.map(v=>v.toFixed(n)).join('–');
const sum=(rows,key)=>rows.reduce((s,r)=>s.map((v,i)=>v+r[key][i]),[0,0]);
const flat=(m,p)=>[0,4,8].map(k=>m[k]*p[0]+m[k+1]*p[1]+m[k+2]*p[2]+m[k+3]);
const identity=[1,0,0,0,0,1,0,0,0,0,1,0];
let projected=[],selected=null;
let data,entries,custom,selNames=new Set(),picked=new Set(),marked=[],timer,cam,gl,program,buffer,position,color;
const rgb=s=>[1,3,5].map(i=>parseInt(s.slice(i,i+2),16)/255);
const current=()=>data.cases[$('motion').selectedIndex];
const duty=()=>data.power[$('duty').selectedIndex];
const key=()=> 'agent-frc/preliminary/groups/'+data.step;
function readGroups(){try{return JSON.parse(localStorage.getItem(key())||'{}')}catch{return {}}}
let isolated=null;
function meshName(c,name){return c.fieldNotes?.[name]?'Chambered NOTE':name}
function displayedEntries(c){
 if(!c.fieldNotes)return entries;
 const mesh=data.bodies['Chambered NOTE'];
 return entries.filter(([name])=>name!=='Chambered NOTE').concat(Object.keys(c.fieldNotes).map(name=>[name,mesh]));
}
function poseAt(c,n,t){return c.fieldNotes?.[n]?.[t]||c.series?.[c.rig?.[n]]?.[t]||c.frames[n]?.[t]||identity}
function interactionFit(){
 const k=current().fixture||current().key;if(k!=='shot'&&k!=='climb'){reset();return}
 if(k==='shot'){
   cam={az:-.76,el:.12,x:0,y:0,center:[0,0,0],zoom:1,trackSpeaker:true};isolated=null;draw();return
 }
 cam={az:-.83,el:.30,x:0,y:0,center:[0,0,0],zoom:1,trackStage:true};isolated=null;draw();return
}

// Orthographically fit both the station and the moving robot. Only the shot's
// interaction view tracks this box; orbit, user zoom and pan remain available.
function fieldFraming(xy,az,el,width,height,stage){
 const u=[-Math.sin(az),Math.cos(az),0];
 const up=[-Math.sin(el)*Math.cos(az),-Math.sin(el)*Math.sin(az),Math.cos(el)];
 let left=Infinity,right=-Infinity,bottom=Infinity,top=-Infinity;
 for(const [lo,hi] of [
   stage?[[3073-xy[0],2398-xy[1],-2],[6030-xy[0],5812-xy[1],2243]]:[[-1519-xy[0],3928-xy[1],0],[920-xy[0],7164-xy[1],2505]],
   [[-780,-780,0],[780,780,1400]]
 ]){
   for(const x of [lo[0],hi[0]])for(const y of [lo[1],hi[1]])for(const z of [lo[2],hi[2]]){
     const h=x*u[0]+y*u[1],v=x*up[0]+y*up[1]+z*up[2];
     left=Math.min(left,h);right=Math.max(right,h);bottom=Math.min(bottom,v);top=Math.max(top,v)
   }
 }
 return {center:u.map((k,i)=>k*(right+left)/2+up[i]*(top+bottom)/2),
   scale:.91*Math.min(width/(right-left),height/(top-bottom))}
}
function reset(){
 isolated=null;labelPage=0;
 const c=$('canvas'),w=c.clientWidth||1000,h=c.clientHeight||600,rail=labelRail(w);
 const az=-.88,el=.43,ca=Math.cos(az),sa=Math.sin(az),ce=Math.cos(el),se=Math.sin(el);
 const u=[-sa,ca,0],v=[-se*ca,-se*sa,ce],bounds=[Infinity,-Infinity,Infinity,-Infinity];
 const cs=current(),frame=+$('time').value;
 for(const [n,b] of entries){if(cs.removed.includes(n))continue;const m=poseAt(cs,n,frame);
  for(const vertex of b.v){const p=flat(m,vertex),x=u.reduce((s,a,i)=>s+a*p[i],0),y=v.reduce((s,a,i)=>s+a*p[i],0);
   bounds[0]=Math.min(bounds[0],x);bounds[1]=Math.max(bounds[1],x);bounds[2]=Math.min(bounds[2],y);bounds[3]=Math.max(bounds[3],y)}}
 const midU=(bounds[0]+bounds[1])/2,midV=(bounds[2]+bounds[3])/2,depth=[ca*ce,sa*ce,se];
 const center=u.map((q,i)=>midU*q+midV*v[i]+(se*475)*depth[i]);
 const scale=.88*Math.min((w-rail-24)/(bounds[1]-bounds[0]),h/(bounds[3]-bounds[2]));
 cam={az,el,x:0,y:0,center,zoom:480*scale/Math.min(w,h)};draw()
}
let updateFieldFixture;
let sceneGPU,paintPending=false,locationMap={},lastProjection;
function initGL(){
 const c=$('canvas');gl=c.getContext('webgl',{antialias:true,alpha:false});
 if(!gl)throw Error('Enable WebGL to view the assembly.');
 const vertex='attribute vec3 point;attribute vec3 normal;uniform mat4 pose;uniform mat4 camera;uniform vec3 tint;uniform bool bend;uniform vec3 offsets[25];varying vec3 shade;void main(){vec3 pt=point;if(bend){float f=clamp((pt.y-2893.751)/100.88075,0.0,23.9999);int ix=int(f);pt+=mix(offsets[ix],offsets[ix+1],fract(f));}gl_Position=camera*pose*vec4(pt,1.0);vec3 n=mat3(pose)*normal;float light=.83+.28*dot(n,vec3(.32,-.42,.84));shade=tint*light+vec3(.035);}';
 const fragment='precision mediump float;varying vec3 shade;void main(){gl_FragColor=vec4(shade,1.0);}';
 program=gl.createProgram();for(const [type,src] of [[gl.VERTEX_SHADER,vertex],[gl.FRAGMENT_SHADER,fragment]]){
  const shader=gl.createShader(type);gl.shaderSource(shader,src);gl.compileShader(shader);if(!gl.getShaderParameter(shader,gl.COMPILE_STATUS))throw Error(gl.getShaderInfoLog(shader));gl.attachShader(program,shader)}
 gl.linkProgram(program);if(!gl.getProgramParameter(program,gl.LINK_STATUS))throw Error(gl.getProgramInfoLog(program));gl.useProgram(program);
 for(const key of ['camera','pose','tint','bend','offsets[0]'])locationMap[key]=gl.getUniformLocation(program,key);
 position=gl.getAttribLocation(program,'point');color=gl.getAttribLocation(program,'normal');gl.enableVertexAttribArray(position);gl.enableVertexAttribArray(color);gl.enable(gl.DEPTH_TEST);
 function upload(values,mode){const buffer=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,buffer);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(values),gl.STATIC_DRAW);return {buffer,count:values.length/6,mode}}
 function triangles(s){const out=[];for(const f of s.f){const [a,b,c]=f.map(i=>s.v[i]);const u=b.map((x,i)=>x-a[i]),v=c.map((x,i)=>x-a[i]);const n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]],l=Math.hypot(...n)||1;for(const p of [a,b,c])out.push(...p,n[0]/l,n[1]/l,n[2]/l)}return upload(out,gl.TRIANGLES)}
 const mesh=new Map(entries.map(([n,b])=>[n,triangles(b)]));const lineData=[];const add=(a,b)=>lineData.push(...a,0,0,1,...b,0,0,1);
 const size=[16540,8210];for(let x=0;x<=size[0];x+=250)add([x,0,0],[x,size[1],0]);for(let y=0;y<=size[1];y+=250)add([0,y,0],[size[0],y,0]);
 const grid=upload(lineData,gl.LINES);lineData.length=0;data.corners.forEach((p,i)=>add(p,data.corners[(i+1)%4]));
 const fixtures={};
 function unpack(str,Type){const raw=atob(str),b=new Uint8Array(raw.length);for(let i=0;i<raw.length;i++)b[i]=raw.charCodeAt(i);return new Type(b.buffer)}
 for(const [name,packed] of Object.entries(data.fixtureMeshes)){
   const v=unpack(packed.v,Float32Array),f=unpack(packed.f,Uint32Array),out=new Float32Array(f.length*6);
   for(let i=0;i<f.length;i+=3){const a=3*f[i],b=3*f[i+1],c=3*f[i+2];const x=v[b]-v[a],y=v[b+1]-v[a+1],z=v[b+2]-v[a+2],u=v[c]-v[a],w=v[c+1]-v[a+1],h=v[c+2]-v[a+2];const nx=y*h-z*w,ny=z*u-x*h,nz=x*w-y*u,L=Math.hypot(nx,ny,nz)||1;
    for(let j=0;j<3;j++){const k=3*f[i+j],at=6*(i+j);out[at]=v[k];out[at+1]=v[k+1];out[at+2]=v[k+2];out[at+3]=nx/L;out[at+4]=ny/L;out[at+5]=nz/L}}
   fixtures[name]=upload(out,gl.TRIANGLES)
 }
 sceneGPU={fixtures,mesh,grid,aperture:upload(lineData,gl.LINES),ground:triangles({v:[[0,0,-.1],[size[0],0,-.1],[size[0],size[1],-.1],[0,size[1],-.1]],f:[[0,1,2],[0,2,3]]})};
 const state=$('fieldStatus'),retry=$('fieldRetry'),loaded=new Set();
 let activeWorker=null, requested=null;
 function update(force=false){
   const kind=current().fixture||current().key;
   const fixture=kind==='shot'?'speaker':kind==='climb'||kind==='park'?'stage':null;
   $('fieldSource').hidden=!fixture;
   if(fixture===requested&&!force)return;
   if(activeWorker){activeWorker.terminate();activeWorker=null}
   requested=fixture;retry.hidden=true;
   if(!fixture)return;
   state.dataset.fixture=fixture;
   const ready=()=>{state.dataset.ready='true';state.textContent=fixture==='stage'?'Stage, anchors and chains · FIRST 2024 field CAD':'Grey speaker · FIRST 2024 field CAD'};
   if(loaded.has(fixture)){ready();return}
   state.dataset.ready='loading';state.textContent='Loading '+fixture+' from FIRST field CAD…';
   const worker=new Worker('./field-cad-worker.js');activeWorker=worker;
   const finish=()=>{worker.terminate();activeWorker=null};
   function failed(message){if(activeWorker!==worker)return;state.dataset.ready='error';state.textContent='Field CAD unavailable · showing reference geometry. '+message;retry.hidden=false;finish()}
   worker.onerror=event=>failed(event.message||'Reader failed to start.');
   worker.onmessage=({data:result})=>{
     if(activeWorker!==worker)return;
     if(result.status){state.textContent=result.status;return}
     if(result.error){failed(result.error);return}
     for(const [name,buffer] of Object.entries(result.buffers)){
       const previous=fixtures[name];fixtures[name]=upload(new Float32Array(buffer),gl.TRIANGLES);
       gl.deleteBuffer(previous.buffer);
     }
     loaded.add(fixture);ready();finish();draw()
   };
   worker.postMessage(fixture);
 }
 updateFieldFixture=update;retry.onclick=()=>update(true);
}
function modelMatrix(m){return [m[0],m[4],m[8],0,m[1],m[5],m[9],0,m[2],m[6],m[10],0,m[3],m[7],m[11],1]}
function draw(){if(!cam||paintPending)return;paintPending=true;requestAnimationFrame(render)}
// Place part names in one margin. The two vertical rails form an ordered fan:
// both ends have the same Y order, then leaders run horizontally to material.
let labelPage=0, followLabel=null;
const calloutNodes=[];
function labelRail(width){return Math.min(212,Math.max(92,width*.235))}
function drawLeaders(labels,width,height,rail){
 const layer=$('names'),nav=$('labelPages'),all=$('labels').checked;
 if(!labels.length){layer.hidden=true;nav.hidden=true;return}
 layer.hidden=false;
 let svg=layer.firstElementChild;
 const ns='http://www.w3.org/2000/svg';
 function element(kind){return document.createElementNS(ns,kind)}
 if(!svg){svg=element('svg');svg.classList.add('callout-diagram');svg.setAttribute('aria-label','Component callouts');layer.append(svg)}
 svg.setAttribute('viewBox','0 0 '+width+' '+height);
 const rowHeight=24,capacity=Math.min(13,Math.max(1,Math.floor((height-36)/rowHeight))),pages=[];
 for(const point of labels){
  let page=pages.find(items=>items.length<capacity&&items.every(other=>Math.abs(other.y-point.y)>=7));
  if(!page){page=[];pages.push(page)}page.push(point);
 }
 if(followLabel){const i=pages.findIndex(page=>page.some(p=>p.name===followLabel));if(i>=0)labelPage=i;followLabel=null}
 labelPage=Math.min(labelPage,pages.length-1);
 const page=pages[labelPage].sort((a,b)=>a.y-b.y);
 nav.hidden=!all||pages.length<2;$('labelPageCount').textContent=(labelPage+1)+' / '+pages.length;
 $('labelPrev').disabled=labelPage===0;$('labelNext').disabled=labelPage===pages.length-1;
 // Clamp each label between the previous row and the room needed by later rows.
 let previous=-rowHeight;
 for(let i=0;i<page.length;i++){
  const p=page[i],row=Math.max(17,previous+rowHeight,Math.min(p.y,height-17-(page.length-i-1)*rowHeight));previous=row;
  let node=calloutNodes[i];
  if(!node){
   const g=element('g'),line=element('polyline'),dot=element('circle'),label=element('text'),title=element('title');
   dot.setAttribute('r','2.7');label.classList.add('tag');label.setAttribute('text-anchor','end');label.setAttribute('dominant-baseline','middle');
   g.append(line,dot,label,title);svg.append(g);node={g,line,dot,label,title};calloutNodes.push(node);
   label.addEventListener('click',()=>inspect(g.dataset.part));
  }
  node.g.style.display='';node.g.dataset.part=p.name;
  node.g.classList.toggle('chosen',p.name===selected||marked.includes(p.name));
  const start=rail-12,turn=rail+14;
  node.line.setAttribute('points',[[start,row],[turn,p.y],[p.x,p.y]].map(q=>q.join(',')).join(' '));
  node.dot.setAttribute('cx',p.x);node.dot.setAttribute('cy',p.y);
  node.label.setAttribute('x',start-5);node.label.setAttribute('y',row);
  node.label.textContent=p.name;node.title.textContent=p.name;
  node.label.removeAttribute('textLength');
  const max=rail-24;
  if(node.label.getComputedTextLength()>max){node.label.setAttribute('textLength',max);node.label.setAttribute('lengthAdjust','spacingAndGlyphs')}
  else node.label.removeAttribute('textLength');
 }
 for(const node of calloutNodes.slice(page.length))node.g.style.display='none';
}


function render(){
 paintPending=false;if(!sceneGPU)return;
 const cs=current(),t=+$('time').value,canvas=$('canvas'),width=canvas.clientWidth,height=canvas.clientHeight,dpr=Math.min(devicePixelRatio||1,2);
 const pw=Math.round(width*dpr),ph=Math.round(height*dpr);
 if(canvas.width!==pw||canvas.height!==ph){canvas.width=pw;canvas.height=ph;gl.viewport(0,0,pw,ph)}
 gl.clearColor(.058,.114,.207,1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
 const rail=$('labels').checked?labelRail(width):0;
 const tracking=(cam.trackSpeaker||cam.trackStage)&&!isolated?fieldFraming(cs.positions[t],cam.az,cam.el,width-rail,height,cam.trackStage):null;
 const center=tracking?tracking.center:cam.center;
 const ca=Math.cos(cam.az),sa=Math.sin(cam.az),ce=Math.cos(cam.el),se=Math.sin(cam.el),sc=cam.zoom*(tracking?tracking.scale:Math.min(width-rail,height)/480);
 const u=[-sa,ca,0].map(v=>v*2*sc/width),v=[-se*ca,-se*sa,ce].map(v=>v*2*sc/height),z=[ce*ca,ce*sa,se].map(v=>-v/45000);
 const dot=(a,b)=>a.reduce((s,x,i)=>s+x*b[i],0);
 const off=[(2*cam.x+rail)/width-dot(u,center),-2*cam.y/height-dot(v,center),-dot(z,center)];
 lastProjection=p=>[dot(u,p)+off[0],dot(v,p)+off[1],dot(z,p)+off[2]];
 gl.uniformMatrix4fv(locationMap.camera,false,[u[0],v[0],z[0],0,u[1],v[1],z[1],0,u[2],v[2],z[2],0,...off,1]);
 function paint(item,mat,col){gl.bindBuffer(gl.ARRAY_BUFFER,item.buffer);gl.vertexAttribPointer(position,3,gl.FLOAT,false,24,0);gl.vertexAttribPointer(color,3,gl.FLOAT,false,24,12);gl.uniformMatrix4fv(locationMap.pose,false,modelMatrix(mat));gl.uniform3fv(locationMap.tint,col);gl.drawArrays(item.mode,0,item.count)}
 const xy=cs.positions[t],field=[1,0,0,-xy[0],0,1,0,-xy[1],0,0,1,0];if($('floor').checked){if(se>0)paint(sceneGPU.ground,field,[.1,.17,.25]);paint(sceneGPU.grid,field,[.16,.25,.34])}if(!isolated&&(cs.fixture||cs.key)==='shot'){
   if(!$('cutaway').checked)paint(sceneGPU.fixtures.speakerPanel,field,[.40,.55,.69]);
   paint(sceneGPU.fixtures.speakerRamp,field,[.49,.66,.77]);
   paint(sceneGPU.fixtures.sensors,field,[1,.55,.12]);paint(sceneGPU.aperture,field,[.36,.94,1]);
 }
 if(!isolated&&cs.fixture==='park'){
  for(const [name,tint] of [['stageSteel',[.44,.57,.70]],['anchors',[.83,.77,.47]],['otherChain',[.76,.81,.85]],['workingChain',[1,.76,.24]]])paint(sceneGPU.fixtures[name],field,tint);
 }
 if(!isolated&&(cs.fixture||cs.key)==='climb'){
  for(const [n,col] of [['stageSteel',[.44,.57,.70]],['anchors',[.83,.77,.47]],['otherChain',[.76,.81,.85]]])paint(sceneGPU.fixtures[n],field,col);
  const now=cs.chain[t],rest=cs.chain[0],delta=new Float32Array(75);
  for(let j=0;j<25;j++)for(let k=0;k<3;k++)delta[j*3+k]=now[j][k]-rest[j][k];
  gl.uniform1i(locationMap.bend,1);gl.uniform3fv(locationMap['offsets[0]'],delta);paint(sceneGPU.fixtures.workingChain,field,[1,.76,.24]);gl.uniform1i(locationMap.bend,0)
 }
 const labels=[];for(const [name,b] of displayedEntries(cs)){if(cs.removed.includes(name)||(isolated&&isolated!==name))continue;
  const m=poseAt(cs,name,t);
  let col=b.tint;if(marked.includes(name))col=[1,.34,.14];else if(!selNames.has(meshName(cs,name)))col=b.dim;if(name===selected)col=[1,.79,.27];
  paint(sceneGPU.mesh.get(meshName(cs,name)),m,col);
  if(rail&&($('labels').checked||marked.includes(name)||name===selected)){
   const point=lastProjection(flat(m,b.leaderAnchor)),x=(point[0]+1)*width/2,y=(1-point[1])*height/2;
   if(x>rail+17&&x<width-4&&y>4&&y<height-4)labels.push({name,x,y});
  }
 }
 drawLeaders(labels,width,height,rail);$('fraction').textContent=cs.time?cs.time[t].toFixed(2)+' s':Math.round(t/24*100)+'%';$('liveMeasure').textContent=cs.measure?.[t]||'Parked assembly';
}
function candidatesForPick(){
 const cs=current(),t=+$('time').value;
 return displayedEntries(cs).filter(([n])=>!cs.removed.includes(n)&&(!isolated||n===isolated)).map(([name,b])=>{
  const m=poseAt(cs,name,t);
  return {name,points:b.v.map(p=>lastProjection(flat(m,p))),faces:b.f}
 })
}


function shape(b){return b}
function modelEvidence(){const all=entries.map(([,b])=>b);$('validation').textContent=all.filter(b=>b.closed).length+'/'+all.length+' closed meshes · '+(all.length*(all.length-1)/2).toLocaleString()+' parked pairs screened, '+data.static.length+' positive, '+data.unresolved.length+' unresolved. Two parked overlaps correspond to the designed compliant tread squeeze. Motion evidence appears under each replay. '+data.focus}
function inspect(n){followLabel=n;selected=n;marked=[];const b=data.bodies[meshName(current(),n)],s=shape(b),box=$('inspector');box.replaceChildren();box.append(text('span',current().fieldNotes?.[n]?'FIELD PIECE':'COMPONENT INSPECTOR'),text('h2',n),text('b',b.category+' / '+b.group),text('p',b.material),text('p',(s.volume/1000).toFixed(1)+' cm³ · '+(s.basis||'Placed planning envelope')));for(const r of data.details.filter(r=>r.members.length===1&&r.members[0]===n))box.append(text('p',range(r.kg,3)+' kg · '+r.name));for(const [label,key] of [['Purpose','purpose'],['Why it is included','rationale']])box.append(text('h3',label),text('p',b[key]||'Missing from this review'));const focus=text('button','Focus part'),restore=text('button','Show assembly');focus.onclick=()=>{const m=poseAt(current(),n,+$('time').value);cam.center=flat(m,b.anchor);const extent=Math.max(...[0,1,2].map(i=>Math.max(...s.v.map(p=>p[i]))-Math.min(...s.v.map(p=>p[i]))));cam.zoom=Math.min(6,300/Math.max(extent,1));cam.x=cam.y=0;isolated=n;draw()};restore.onclick=reset;if(data.support[n])box.append(text('p',data.support[n]));box.append(focus,restore);draw()}
function pick(e){const r=$('canvas').getBoundingClientRect(),x=2*(e.clientX-r.left)/r.width-1,y=1-2*(e.clientY-r.top)/r.height;let best=Infinity,hit=null;for(const obj of candidatesForPick())for(const [i,j,k] of obj.faces){const [a,b,c]=[obj.points[i],obj.points[j],obj.points[k]];const det=(b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1]);if(Math.abs(det)<1e-14)continue;const u=((b[1]-c[1])*(x-c[0])+(c[0]-b[0])*(y-c[1]))/det,v=((c[1]-a[1])*(x-c[0])+(a[0]-c[0])*(y-c[1]))/det,h=1-u-v;if(Math.min(u,v,h)<0)continue;const z=u*a[2]+v*b[2]+h*c[2];if(z<best){best=z;hit=obj.name}}if(hit)inspect(hit)}
function group(){
 const by=$('group').value,sub=$('subset'),opts=by==='Category'?[...new Set(entries.map(([,b])=>b.category))]:by==='Subsystem'?[...new Set(entries.map(([,b])=>b.group))]:Object.keys(custom);
 const old=sub.value;options(sub,opts);if(opts.includes(old))sub.value=old;sub.hidden=by==='Whole robot'||!opts.length;$('editor').hidden=by!=='Custom subsystem';
 if(by==='Custom subsystem')picked=new Set(custom[sub.value]||[]);
 select()
}
function select(){
 const by=$('group').value,val=$('subset').value;
 selNames=new Set(entries.filter(([n,b])=>(by==='Whole robot'||(by==='Category'?val===b.category:by==='Subsystem'?val===b.group:picked.has(n)))).map(([n])=>n));
 budget();inventory();draw()
}
function partition(lines,whole){
 const included=[],shared=[];
 for(const line of lines){const n=line.members.filter(v=>selNames.has(v)).length;if(whole||n&&n===line.members.length)included.push(line);else if(n)shared.push(line)}
 return {included,shared}
}
function budget(){
 resourcePanel();
 const whole=$('group').value==='Whole robot',e=duty(),m=partition(data.mass,whole),p=partition(e.lines,whole),detail=partition(data.details,false),amps=sum(p.included,'amps'),kg=sum(m.included,'kg'),volume=entries.filter(([n])=>selNames.has(n)).reduce((s,[,b])=>s+shape(b).volume,0)/1e6;
 const values=[['Component envelopes',selNames.size+' selected · '+volume.toFixed(2)+' L'],['Inspection mass allocated',m.shared.length&&!m.included.length?'Shared estimate · see detail':range(kg,3)+' kg'+(m.shared.length?' + shared mass':'')],...(!whole&&detail.included.length?[['Identified component estimates',range(sum(detail.included,'kg'),3)+' kg · partial']]:[]),['Attributed battery current',range(amps,1)+' A'+(p.shared.length?' + shared load':'')],['Attributed pack sag',range(amps.map(a=>a*e.resistance),2)+' V'+(p.shared.length?' + shared load':'')],['Whole-robot loaded bus',range(e.loadedBusV,2)+' V · '+e.requiredMinV+' V required']];
 const div=$('summary');div.replaceChildren();for(const [k,v] of values){const row=text('p',k);row.append(text('b',v));div.append(row)}
 const lines=$('lines');lines.replaceChildren();for(const [title,arr,unit,field] of [['Included inspection',m.included,'kg','kg'],['Shared inspection · unapportioned',m.shared,'kg','kg'],['Component estimates · separate basis',detail.included,'kg','kg'],['Included current',p.included,'A','amps'],['Shared current · unapportioned',p.shared,'A','amps']]){
  if(!arr.length)continue;lines.append(text('strong',title));
  for(const v of arr)lines.append(text('p',(v.id||v.name)+' · '+range(v[field],3)+' '+unit))
 }
 lines.append(text('p','The whole-robot bus includes all loads. Ranges from independent extrema can be conservative. Running items and slide estimates are shown separately from inspection allocations.'));
}
function table(headers){const el=document.createElement('table'),head=el.createTHead().insertRow();headers.forEach(h=>head.append(text('th',h)));return [el,el.createTBody()]}
function inventory(){
 const edit=$('group').value==='Custom subsystem',[tab,body]=table([edit?'Use':'Category','Component / subsystem','Preliminary material','Volume']);
 for(const [n,b] of entries){if(!edit&&!selNames.has(n))continue;
  const r=body.insertRow();r.dataset.name=n;const first=r.insertCell();
  if(edit){const ck=document.createElement('input');ck.type='checkbox';ck.checked=picked.has(n);ck.setAttribute('aria-label','Include '+n);ck.onclick=ev=>{ev.stopPropagation();ck.checked?picked.add(n):picked.delete(n);selNames=new Set(picked);budget();draw()};first.append(ck)}
  else{const s=document.createElement('i');s.className='swatch';s.style.background=data.palette[b.category];first.append(s,b.category)}
  const cell=r.insertCell();cell.append(text('b',n),document.createElement('br'),b.group);r.insertCell().textContent=b.material;r.insertCell().textContent=(shape(b).volume/1000).toFixed(1)+' cm³';r.tabIndex=0;r.onclick=()=>inspect(n);r.onkeydown=e=>{if(e.key==="Enter")inspect(n)}
 }
 $('inventory').replaceChildren(tab);$('inventoryScope').textContent=edit?'Check members, give the subsystem a name and save.':'Each row is one placed preliminary envelope. Select a row to locate it.';
}
function evidence(){
 const parked=$('set').selectedIndex===1,cs=current(),rows=parked?data.static.map(x=>({pair:x.pair,status:'overlap',witness:{volumeMm3:x.volume}})):cs.results;
 const counts={};rows.forEach(r=>counts[r.status]=(counts[r.status]||0)+1);
 $('count').textContent=parked?'Base STEP · '+data.narrow+' bounding-box candidates measured. '+rows.length+' positive-volume pairs.':cs.id+' · '+Object.entries(counts).map(([k,v])=>v+' '+k).join(' / ')+(cs.gap==null?'':' · clearance '+cs.gap+' mm');
 const [tab,body]=table(['Pair','Result','Witness']);for(const f of rows){
  if(!parked&&$('filter').value==='adverse'&&!['fail','open'].includes(f.status))continue;
  const r=body.insertRow();r.dataset.finding='1';r.insertCell().textContent=f.pair.join(' / ');r.insertCell().textContent=f.status;const w=f.witness||{};r.insertCell().textContent=w.reason|| (Array.isArray(w)?'Open interval '+w.map(v=>Math.round(v*100)+'%').join('–'):w.volumeMm3>0?w.volumeMm3.toFixed(2)+' mm³':Number.isFinite(w.gapMm)?w.gapMm.toFixed(2)+' mm':'Unresolved');
  r.onclick=()=>{stop();marked=f.pair;if(parked){$('motion').selectedIndex=0;$('time').value=0}else if(Number.isFinite(w.t))$('time').value=Math.round(w.t*24);$('witness').textContent=f.pair.join(' / ')+(Number.isFinite(w.t)?' · recorded stroke '+(100*w.t).toFixed(2)+'% · nearest displayed frame':' · parked or interval result');draw()}
 }
 $('findings').replaceChildren(tab)
}
function stop(){clearInterval(timer);timer=null;$('play').textContent='Play'}
async function init(){
 const response=await fetch('./scene.json.gz');if(!response.ok)throw Error('Review data unavailable');const bytes=await response.arrayBuffer();const packed=new Uint8Array(bytes);data=await (packed[0]===31&&packed[1]===139?new Response(new Blob([bytes]).stream().pipeThrough(new DecompressionStream('gzip'))):new Response(bytes)).json();
 const query=new URLSearchParams(location.search), requested=query.get('case');
 const guided=data.concepts?.guided,field=data.concepts?.field;
 const conceptKey=query.get('concept')||(!requested||guided?.inherit.includes(requested)||guided?.cases.some(c=>c.key===requested)?'guided':field?.cases.some(c=>c.key===requested)?'field':null);
 let variant=data.concepts?.[conceptKey];
 if(conceptKey==='guided' && guided && field){
   const carry=Object.fromEntries(Object.keys(guided.add).map(n=>[n,'arm']));
   const inherited=guided.inherit.map(key=>{
     const c=field.cases.find(item=>item.key===key);
     const scope=key==='field-park'?' Added hoist stock clears the PARK approach.':' Earlier free-piece trial. Hoist stock was checked against the recorded piece path; updated dynamic scoring remains open.';
     return {...c,rig:{...carry,...c.rig},basis:c.basis+scope};
   });
   variant={...field,...guided,add:{...field.add,...guided.add},cases:[...inherited,...guided.cases].map(c=>c.key==='guided-service'?c:{...c,removed:[...new Set([...c.removed,...(guided.fixtureOnlyNames||[])])]})};
 }
 const slideService=data.slideService;
 const activeCase=requested||variant?.defaultCase;
 const sceneGroup=k=>k===slideService?.case.key?'service':slideService?.currentKeys?.includes(k)?'operating':'historical';
 if(conceptKey==='guided' && slideService){
   variant={...variant,cases:[...variant.cases,slideService.case]};
   if(sceneGroup(activeCase)!=='historical'){
     const gross=slideService.mass.kg, combine=a=>a.map((v,i)=>v+gross[i]);
     const originalMass=variant.mass.map(row=>({...row,members:row.members.filter(n=>!slideService.mass.members.includes(n))}));
     if(sceneGroup(activeCase)==='operating'){
       const members=Object.fromEntries(Object.keys(slideService.add).map(n=>[n,'frame']));
       variant={...variant,add:{...variant.add,...slideService.add},mass:[...originalMass,slideService.mass],
         cases:variant.cases.map(c=>slideService.currentKeys.includes(c.key)?{...c,rig:{...c.rig,...members}}:c),
         power:[slideService.currentPower,...variant.power],inspection:combine(variant.inspection),running:combine(variant.running),
         status:'Preliminary design held · scoring repeatability',
         validationText:'The complete service package is charged to the chassis mass budget. The free-piece screen covers twelve nominal pickups and shots. The motor can be removed with the cassette locked at its service angle.',
         notes:['Six individual events at both nominal resolutions. Four of five starts produced six individual events with the mission-length observer. Fifteen starts remain untested.',
                'The inspection estimate includes 6 kg design reserve, the cable take-up allowance and the gross slide-motor service package. Grip, loaded tracking and thermal duty remain provisional.']};
     }elsevariant={...variant,add:{...variant.add,...slideService.add},mass:[...originalMass,slideService.mass],
       inspection:combine(variant.inspection),running:combine(variant.running),issue:'Guided cassette · slide motor service',
       step:slideService.step,focus:slideService.case.basis,validationText:'Service-specific installed package. Connector envelopes and the real wire exits are included. See the checks for the fully supported maintenance position.'};
   }
 }
 if(variant){const bodies={...data.bodies};for(const n of variant.remove)delete bodies[n];Object.assign(bodies,variant.add);
 data={...data,...variant,bodies};}

 entries=Object.entries(data.bodies);custom=readGroups();
 for(const [n,b] of entries){b.tint=rgb(data.palette[b.category]);b.dim=b.tint.map((x,i)=>.20*x+.80*[.64,.73,.83][i]);b.anchor=[0,1,2].map(i=>b.v.reduce((s,p)=>s+p[i],0)/b.v.length);let best=Infinity;for(const p of b.v){const d=p.reduce((sum,x,i)=>sum+(x-b.anchor[i])**2,0);if(d<best){best=d;b.leaderAnchor=p}}}
 options($('motion'),data.cases.map(c=>c.id));const deep=new URLSearchParams(location.search).get('case')||variant?.defaultCase;const ix=data.cases.findIndex(c=>c.key===deep);if(ix>=0)$('motion').selectedIndex=ix;
 options($('duty'),data.power.map(e=>e.case));$('duty').selectedIndex=0;
 $('state').textContent=data.status;document.querySelector('.title p').textContent='2024　/　Preliminary design';document.querySelector('h1').textContent=data.issue;const parts=entries.length;
 const maxError=Math.max(...entries.map(([,b])=>b.error));
 const metrics=[['Inspection estimate',range(data.inspection,3)+' kg'],['Running estimate',range(data.running,2)+' kg'],['Package envelope sum',(entries.reduce((s,[,b])=>s+shape(b).volume,0)/1e6).toFixed(1)+' L'],['Movement cases',data.cases.length+' modeled']];
 for(const [label,value] of metrics){const box=document.createElement('div');box.append(text('span',label),text('b',value));$('metrics').append(box)}
 for(const role of new Set(entries.map(([,b])=>b.category))){const span=document.createElement('span'),i=document.createElement('i');i.className='swatch';i.style.background=data.palette[role];span.append(i,role);$('legend').append(span)}
 $('validation').textContent=entries.filter(([,b])=>b.closed).length+'/'+parts+' closed meshes · max absolute volume difference '+maxError.toPrecision(3)+' mm³. '+parts*(parts-1)/2+' parked pairs screened, '+data.static.length+' positive, '+data.unresolved.length+' unresolved. Motion evidence is scoped to each case.';
 if(variant?.validationText)$('validation').textContent=variant.validationText;
 const notes=data.notes||[];const n=data.narrativeReview;if(n){$('narrative').replaceChildren(text('p',n.source.file+' · '+n.source.sha256.slice(0,12)),...n.objectives.map(row=>text('p',row.objective+' — '+row.disposition+'. '+row.basis)))}
 $('notes').replaceChildren(...notes.map(v=>text('p',v)));initGL();group();reset();evidence();modelEvidence();
 $('duty').onchange=budget;$('group').onchange=group;$('subset').onchange=()=>{if($('group').value==='Custom subsystem')picked=new Set(custom[$('subset').value]||[]);select()};
 $('save').onclick=()=>{const n=$('subname').value.trim();if(!n||!picked.size){$('inventoryScope').textContent='Enter a subsystem name and select at least one component.';return}custom[n]=[...picked];localStorage.setItem(key(),JSON.stringify(custom));group();$('subset').value=n;$('subset').onchange()};
 function activate(){
   stop();const c=current();$('time').max=c.positions.length-1;$('time').value=0;marked=[];selected=null;
   const link=new URL(location.href);if(variant)link.searchParams.set('concept',conceptKey);else link.searchParams.delete('concept');link.searchParams.set('case',c.key);history.replaceState({},'',link);if(parent!==window)parent.postMessage({type:'review-case',case:c.key},location.origin);
   const match=data.power.find(e=>e.stage===c.id&&e.batteryCondition==='weak');if(match){$('duty').value=match.case;budget()}
   $('cutaway').disabled=(c.fixture||c.key)!=='shot';$('basis').textContent=c.basis;$('witness').textContent='';evidence();interactionFit();updateFieldFixture();
 }
 $('motion').onchange=()=>{const next=current().key;if(slideService && sceneGroup(activeCase)!==sceneGroup(next)){const u=new URL(location.href);u.searchParams.set('concept','guided');u.searchParams.set('case',next);location.assign(u.href);return;}activate();};$('time').oninput=()=>{stop();draw()};
 $('play').onclick=()=>{if(timer)return stop();const c=current(),times=c.time||c.positions.map((_,i)=>i*.1);if(+$('time').value===times.length-1)$('time').value=0;const start=performance.now(),offset=times[+$('time').value],rate=+$('speed').value;$('play').textContent='Pause';timer=setInterval(()=>{const elapsed=offset+(performance.now()-start)*rate/1000;let t=+$('time').value;while(t<times.length-1&&times[t+1]<=elapsed)t++;$('time').value=t;draw();if(t===times.length-1)stop()},16)};
 $('speed').onchange=stop;$('cutaway').onchange=draw;activate();
 $('labelPrev').onclick=()=>{labelPage=Math.max(0,labelPage-1);draw()};$('labelNext').onclick=()=>{labelPage+=1;draw()};
 for(const id of ['labels','floor'])$(id).onchange=draw;

 $('expand').onclick=async()=>{if(document.fullscreenElement){await document.exitFullscreen()}else{await document.querySelector('.viewer').requestFullscreen()}};
 document.addEventListener('fullscreenchange',()=>{$('expand').textContent=document.fullscreenElement?'Restore':'Expand';draw()});
 $('set').onchange=$('filter').onchange=evidence;$('reset').onclick=reset;$('field').onclick=interactionFit;
 const c=$('canvas');let drag=null;c.oncontextmenu=e=>e.preventDefault();
 c.onpointerdown=e=>{c.setPointerCapture(e.pointerId);drag=[e.clientX,e.clientY,e.shiftKey||e.button===2||e.button===1,e.clientX,e.clientY];c.focus()};
 c.onpointermove=e=>{if(!drag)return;const dx=e.clientX-drag[0],dy=e.clientY-drag[1];if(drag[2]){cam.x+=dx;cam.y+=dy}else{cam.az-=dx*.009;cam.el+=dy*.009}drag[0]=e.clientX;drag[1]=e.clientY;draw()};
 c.onpointerup=e=>{if(drag&&!drag[2]&&Math.hypot(e.clientX-drag[3],e.clientY-drag[4])<5)pick(e);drag=null};c.onpointercancel=()=>drag=null;c.addEventListener('wheel',e=>{e.preventDefault();cam.zoom=Math.max(.04,Math.min(8,cam.zoom*Math.exp(-e.deltaY*.0015)));draw()},{passive:false});
 c.onkeydown=e=>{const v={ArrowLeft:[-1,0],ArrowRight:[1,0],ArrowUp:[0,-1],ArrowDown:[0,1]}[e.key];if(e.key==='Home')reset();else if(v){if(e.shiftKey){cam.x+=20*v[0];cam.y+=20*v[1]}else{cam.az+=.15*v[0];cam.el-=.15*v[1]}}else if(e.key==='+'||e.key==='-'){cam.zoom=Math.max(.04,Math.min(8,cam.zoom*(e.key==='+'?1.2:1/1.2)))}else return;e.preventDefault();draw()};
 new ResizeObserver(draw).observe(c);

}
init().catch(e=>{$('state').textContent=e.message;console.error(e)});
