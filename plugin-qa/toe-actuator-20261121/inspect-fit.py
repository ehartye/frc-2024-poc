from pathlib import Path
import sys,numpy as np,xml.etree.ElementTree as E,json
q=Path(__file__).parent
sys.path.insert(0,str(Path.home()/'repos/agent-frc/scripts'))
from mechanical_motion import bounds
from showcase_contacts import load_solids
from stock_sections import entities
from OCP.TopAbs import TopAbs_VERTEX
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
for nm in ['REV-41-1332','REV-41-1503','REV-21-1651','REV-41-1607','REV-41-1608','REV-41-1602','REV-41-1615']:
 fn=next(q.glob(nm+'.[sS][tT][eE][pP]')); a,b=load_solids(fn)
 bb=np.array([bounds(s)for s in a.values()])
 print(nm,len(a),np.round(np.concatenate([bb[:,:3].min(axis=0),bb[:,3:].max(axis=0)]),3))
 for k in list(a)[:4]:
  print(k,np.round(bounds(a[k]),2))
 if nm=='REV-41-1503':
  s=next(iter(a.values()));v=np.array([BRep_Tool.Pnt_s(TopoDS.Vertex(v)).Coord() for v in entities(s,TopAbs_VERTEX)]);np.save(q/'selected-rack-verts.npy',v)
  for k in [1,2]:
   n,c=np.unique(np.round(v[:,k],3),return_counts=True);print('freq coords',k,[(n[i],c[i])for i in np.argsort(c)[-10:]])
c=Path.home()/'repos/frc-2024-poc-worktrees/engineering-iterations/iterations/002/design/over-bumper-pickup/head-clear-96'
x=E.parse(c/'fixture.xml')
print([(b.get('name'),b.get('pos'),[(g.get('name'),g.get('pos'),g.get('size'),g.get('type'))for g in b.findall('geom')])for b in x.iter('body') if b.get('name') in ['head','fingerL','fingerR','fingerback']])
print([(g.get('name'),g.get('type'),g.get('pos'),g.get('size'),g.get('axisangle'))for g in x.find('worldbody').findall('geom')][:30])
print(json.loads((c/'floor/cycle.json').read_bytes())['samples'][300])
