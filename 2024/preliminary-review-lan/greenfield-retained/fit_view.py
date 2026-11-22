from pathlib import Path
p=Path(__file__).resolve().parent.parent/'www'/'review.js'
s=p.read_text(encoding='utf8')
a=s.index('function reset()');b=s.index('let sceneGPU',a)
s=s[:a]+"""function reset(){
 isolated=null;labelPage=0;
 const c=$('canvas'),w=c.clientWidth||1000,h=c.clientHeight||600,rail=labelRail(w);
 const az=-.88,el=.43,ca=Math.cos(az),sa=Math.sin(az),ce=Math.cos(el),se=Math.sin(el);
 const u=[-sa,ca,0],v=[-se*ca,-se*sa,ce],bounds=[Infinity,-Infinity,Infinity,-Infinity];
 const cs=current(),frame=+$('time').value;
 for(const [n,b] of entries){if(cs.removed.includes(n))continue;const m=cs.frames[n]?.[frame]||identity;
  for(const vertex of b.v){const p=flat(m,vertex),x=u.reduce((s,a,i)=>s+a*p[i],0),y=v.reduce((s,a,i)=>s+a*p[i],0);
   bounds[0]=Math.min(bounds[0],x);bounds[1]=Math.max(bounds[1],x);bounds[2]=Math.min(bounds[2],y);bounds[3]=Math.max(bounds[3],y)}}
 const midU=(bounds[0]+bounds[1])/2,midV=(bounds[2]+bounds[3])/2,depth=[ca*ce,sa*ce,se];
 const center=u.map((q,i)=>midU*q+midV*v[i]+(se*475)*depth[i]);
 const scale=.88*Math.min((w-rail-24)/(bounds[1]-bounds[0]),h/(bounds[3]-bounds[2]));
 cam={az,el,x:0,y:0,center,zoom:480*scale/Math.min(w,h)};draw()
}
"""+s[b:]
s=s.replace("  const max=rail-24;","  node.label.removeAttribute('textLength');\n  const max=rail-24;")
# One label panel remains readable even when the assembly has scores of coincident fasteners.
s=s.replace('capacity=Math.max(1,Math.floor((height-36)/rowHeight))','capacity=Math.min(13,Math.max(1,Math.floor((height-36)/rowHeight)))')
tmp=p.with_suffix('.next');tmp.write_text(s,encoding='utf8');tmp.replace(p)

