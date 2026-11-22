from pathlib import Path
import sys,re,mmap,hashlib,json,array
q=Path(__file__).parent
mode='key'
src=Path.home()/'repos/frc-2024-poc-worktrees/engineering-iterations/iterations/002/design/cad/output-key/first-party-lineage/assembly.step'
keepnames={'turned and keyed shaft','Output shaft supplier key'}
sha=hashlib.sha256()
with src.open('rb') as f, mmap.mmap(f.fileno(),0,access=mmap.ACCESS_READ) as m:
 sha.update(m);assert sha.hexdigest()=='869021c24d80f57dea7320450657ca91b871ccc44b850c0354ac4d94e977b918'
 offset=array.array('I',[0])*4100000
 starts=re.compile(rb'(?m)^#(\d+)=')
 for a in starts.finditer(m): offset[int(a[1])]=a.start()
 def raw(i):
  b=offset[i];assert b
  return m[b:m.find(b';',b)+1]
 solids={int(a[1]):a[2].decode() for a in re.finditer(rb"#(\d+)=MANIFOLD_SOLID_BREP\('([^']*)',",m)}
 keep={i for i,v in solids.items() if v in keepnames}
 missing=sorted(keepnames-{solids[i] for i in keep})
 print('selected',len(keep),'missing',missing,flush=True)
 nodes=set(int(a[1]) for a in re.finditer(rb'(?m)^#(\d+)=(?:SHAPE_DEFINITION_REPRESENTATION|CONTEXT_DEPENDENT_SHAPE_REPRESENTATION)\(',m))
 # Some Fusion products carry their occupied geometry in an additional representation relationship.
 nodes.update(int(a[1]) for a in re.finditer(rb'(?m)^#(\d+)=SHAPE_REPRESENTATION_RELATIONSHIP\(',m))
 vis={};todo=list(nodes)
 while todo:
  i=todo.pop()
  if i in vis:continue
  x=raw(i)
  if b'MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION(' in x:
   x=re.sub(rb"\(\#.*?\),",b'(),',x,flags=re.S)
  if x.split(b'=',1)[1].startswith(b'ADVANCED_BREP_SHAPE_REPRESENTATION'):
   x=x.replace(b'\r',b'').replace(b'\n',b'').replace(b' ',b'')
   match=re.match(rb"(#[0-9]+=ADVANCED_BREP_SHAPE_REPRESENTATION\('[^']*',\()([^)]*)(\),#[0-9]+\);)", x,re.S)
   assert match,x[:160]
   items=[int(j) for j in re.findall(rb'#(\d+)',match[2])]
   items=[j for j in items if j in keep or b'AXIS2_PLACEMENT_3D(' in raw(j)]
   x=match[1]+b','.join(b'#'+str(j).encode() for j in items)+match[3]
  vis[i]=x
  for j in re.findall(rb'#(\d+)',x.split(b'=',1)[1]):
   j=int(j)
   if j not in vis:todo.append(j)
 assert keep<=vis.keys(),len(keep-vis.keys())
 unexpected=vis.keys() & (solids.keys()-keep)
 if unexpected:
  j=min(unexpected);print('unexpected',j,'refs',[(i,x[:200]) for i,x in vis.items() if re.search(fr'#{j}(?![0-9])'.encode(),x)],flush=True)
 assert not unexpected
 with (q/(mode+'-selected-root.step')).open('wb') as g:
  g.write(m[:m.find(b'DATA;')+5]+b'\n')
  for i in sorted(vis):g.write(vis[i]+b'\n')
  g.write(b'ENDSEC;\nEND-ISO-10303-21;\n')
 print('named',[(i,solids[i]) for i in sorted(keep)],'nodes',len(vis),flush=True)

