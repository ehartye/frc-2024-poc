from pathlib import Path
import shutil
p=Path(__file__).resolve().parent;www=p.parent.parent/'www'
for n in ['review.js','index.html','presentation.md','review.css']:
    if not (p/('p3-'+n)).exists():shutil.copyfile(www/n,p/('p3-'+n))
s=(p/'p3-review.js').read_text(encoding='utf8')
def sub(a,b):
    global s
    assert a in s,a
    s=s.replace(a,b)
sub('function reset(){','''function poseAt(c,n,t){return c.series?.[c.rig?.[n]]?.[t]||c.frames[n]?.[t]||identity}
function interactionFit(){
 const k=current().key;if(k!=='shot'&&k!=='climb'){reset();return}
 const h=$('canvas').clientHeight||500,w=$('canvas').clientWidth||800;
 const width=k==='shot'?3300:3800,height=k==='shot'?2850:2500;
 const sc=.9*Math.min(w/width,h/height);
 cam={az:k==='shot'?-1.12:-.83,el:k==='shot'?.18:.30,x:0,y:0,center:k==='shot'?[-950,0,1190]:[-800,0,1080],zoom:480*sc/Math.min(w,h)};isolated=null;draw()
}
function reset(){''')
sub('cs.frames[n]?.[frame]||identity','poseAt(cs,n,frame)')
sub('cs.frames[name]?.[t]||cs.frames[name]?.[0]||identity','poseAt(cs,name,t)')
sub("current().frames[n]?.[+$('time').value]||current().frames[n]?.[0]||identity","poseAt(current(),n,+$('time').value)")
sub('uniform vec3 tint;varying vec3 shade;void main(){gl_Position=camera*pose*vec4(point,1.0);','uniform vec3 tint;uniform bool bend;uniform vec3 offsets[25];varying vec3 shade;void main(){vec3 pt=point;if(bend){float f=clamp((pt.y-2893.751)/100.88075,0.0,23.9999);int ix=int(f);pt+=mix(offsets[ix],offsets[ix+1],fract(f));}gl_Position=camera*pose*vec4(pt,1.0);')
sub("['camera','pose','tint']","['camera','pose','tint','bend','offsets[0]']")
sub('sceneGPU={mesh,grid,aperture:', '''const fixtures={};
 function unpack(str,Type){const raw=atob(str),b=new Uint8Array(raw.length);for(let i=0;i<raw.length;i++)b[i]=raw.charCodeAt(i);return new Type(b.buffer)}
 for(const [name,packed] of Object.entries(data.fixtureMeshes)){
   const v=unpack(packed.v,Float32Array),f=unpack(packed.f,Uint32Array),out=new Float32Array(f.length*6);
   for(let i=0;i<f.length;i+=3){const a=3*f[i],b=3*f[i+1],c=3*f[i+2];const x=v[b]-v[a],y=v[b+1]-v[a+1],z=v[b+2]-v[a+2],u=v[c]-v[a],w=v[c+1]-v[a+1],h=v[c+2]-v[a+2];const nx=y*h-z*w,ny=z*u-x*h,nz=x*w-y*u,L=Math.hypot(nx,ny,nz)||1;
    for(let j=0;j<3;j++){const k=3*f[i+j],at=6*(i+j);out[at]=v[k];out[at+1]=v[k+1];out[at+2]=v[k+2];out[at+3]=nx/L;out[at+4]=ny/L;out[at+5]=nz/L}}
   fixtures[name]=upload(out,gl.TRIANGLES)
 }
 sceneGPU={fixtures,mesh,grid,aperture:''')
sub('paint(sceneGPU.aperture,field,[.3,.78,.95]);', '''if(!isolated&&cs.key==='shot'){if(!$('cutaway').checked)paint(sceneGPU.fixtures.speakerPanel,field,[.40,.55,.69]);paint(sceneGPU.fixtures.speakerRamp,field,[.49,.66,.77]);paint(sceneGPU.fixtures.sensors,field,[1,.55,.12]);paint(sceneGPU.aperture,field,[.36,.94,1]);}
 if(!isolated&&cs.key==='climb'){
  for(const [n,col] of [['stageSteel',[.44,.57,.70]],['anchors',[.83,.77,.47]],['otherChain',[.76,.81,.85]]])paint(sceneGPU.fixtures[n],field,col);
  const now=cs.chain[t],rest=cs.chain[0],delta=new Float32Array(75);
  for(let j=0;j<25;j++)for(let k=0;k<3;k++)delta[j*3+k]=now[j][k]-rest[j][k];
  gl.uniform1i(locationMap.bend,1);gl.uniform3fv(locationMap['offsets[0]'],delta);paint(sceneGPU.fixtures.workingChain,field,[1,.76,.24]);gl.uniform1i(locationMap.bend,0)
 }''')
sub("$('fraction').textContent=Math.round(t/24*100)+'%';","$('fraction').textContent=cs.time?cs.time[t].toFixed(2)+' s':Math.round(t/24*100)+'%';$('liveMeasure').textContent=cs.measure?.[t]||'Parked assembly';")
sub("const switcher=$('conceptSwitch');switcher.href=variant?'/':'/?concept=shared';switcher.textContent=variant?'Return to retained cassette':'View passive cover and shared lift';","")
sub("options($('duty'),data.power.map", """const deep=new URLSearchParams(location.search).get('case');const ix=data.cases.findIndex(c=>c.key===deep);if(ix>=0)$('motion').selectedIndex=ix;
 options($('duty'),data.power.map""")
sub("initGL();group();reset();evidence();modelEvidence();","initGL();group();reset();evidence();modelEvidence();")
start=s.index(" $('motion').onchange=");end=s.index(" $('labelPrev')",start)
s=s[:start]+""" function activate(){
   stop();const c=current();$('time').max=c.positions.length-1;$('time').value=0;marked=[];selected=null;
   const link=new URL(location.href);link.searchParams.delete('concept');link.searchParams.set('case',c.key);history.replaceState({},'',link);
   const match=data.power.find(e=>e.stage===c.id&&e.batteryCondition==='weak');if(match){$('duty').value=match.case;budget()}
   $('cutaway').disabled=c.key!=='shot';$('basis').textContent=c.basis;$('witness').textContent='';evidence();interactionFit();
 }
 $('motion').onchange=activate;$('time').oninput=()=>{stop();draw()};
 $('play').onclick=()=>{if(timer)return stop();const c=current(),times=c.time||c.positions.map((_,i)=>i*.1);if(+$('time').value===times.length-1)$('time').value=0;const start=performance.now(),offset=times[+$('time').value],rate=+$('speed').value;$('play').textContent='Pause';timer=setInterval(()=>{const elapsed=offset+(performance.now()-start)*rate/1000;let t=+$('time').value;while(t<times.length-1&&times[t+1]<=elapsed)t++;$('time').value=t;draw();if(t===times.length-1)stop()},16)};
 $('speed').onchange=stop;$('cutaway').onchange=draw;activate();
"""+s[end:]
sub("$('field').onclick=()=>{cam.center=[-700,0,1100];cam.zoom=.15;cam.x=cam.y=0;draw()};","$('field').onclick=interactionFit;")
sub('Motion checked at 25 poses per case.','Two parked overlaps correspond to the designed compliant tread squeeze. Motion evidence appears under each replay.')
h=(p/'p3-index.html').read_text(encoding='utf8')
i=h.index('<a id="conceptSwitch"');h=h[:i]+h[h.index('</a>',i)+4:]
h=h.replace('Fit target','Fit field interaction').replace('<button id="reset">','<label><input type="checkbox" id="cutaway"> Target cutaway</label><button id="reset">')
h=h.replace('<input id="time"', '<select id="speed" aria-label="Playback speed"><option value="1">1×</option><option value=".25">¼×</option></select><input id="time"')
h=h.replace('<div id="legend">','<div class="mission-readout"><strong id="liveMeasure"></strong><small id="basis"></small></div><div id="legend">')
budget_summary = '<section id="resources" aria-label="Weight and power budgets"></section>'
ledgers = '<section class="lower power-ledger">'
assert h.count(budget_summary) == h.count(ledgers) == 1
h = h.replace(budget_summary, '').replace(ledgers, budget_summary + ledgers)
css=(p/'p3-review.css').read_text(encoding='utf8')+'''
.mission-readout{padding:12px 18px;border-left:4px solid #edbc46;background:#eff6ff;color:#15345a;display:grid;gap:6px}.mission-readout small{line-height:1.45}.playback input[type=range]{min-width:70px}
@media(max-width:560px){#resources{display:grid;grid-template-columns:1fr}}
'''
css = css.replace('#resources{order:0;margin:0 0 12px;', '#resources{order:2;margin:12px 0 0;')
guide=(p/'p3-presentation.md').read_text(encoding='utf8')+'''
## Field interactions in this revision

Use the motion selector for the reverse-belt shot and the disabled climb. Fit field interaction includes the registered field fixture. Use Target cutaway to inspect the NOTE inside the speaker. The shot includes a one-second kinematic cam rise before the motor and free-piece simulation clock starts. The climb shows the loaded chain shape, ground reaction and drive torque through shutdown. See the clearance evidence below each replay for matched controls. The CAD field source is FE-2024.STEP. Physical test data for drivetrain inertia, grip, gate entry and brake strength remain outstanding.
'''
for n,body in [('review.js',s),('review.css',css),('index.html',h),('presentation.md',guide)]:
    (p/('next-'+n)).write_text(body,encoding='utf8')
for n in ['data.json','review.js','review.css','presentation.md','index.html']:
    dest=www/n;temp=www/(n+'.next');shutil.copyfile(p/('mission-data.json' if n=='data.json' else 'next-'+n),temp);temp.replace(dest)

