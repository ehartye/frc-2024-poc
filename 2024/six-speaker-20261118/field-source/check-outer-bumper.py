import json,hashlib,math,sys
from pathlib import Path
import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Pnt,gp_Trsf
from inspect_solids import read_shape

root=Path(__file__).parent
rows=json.loads((root/'cad-extraction/occurrences.json').read_bytes())
field=[r for r in rows if len(r['path'])>2 and r['path'][1]=='GE-24300_Briweld Field Blue' and not r['children']]
bounds=np.array([r['boundsCadMm'] for r in field]).reshape(-1,2,3)
inventory=json.loads((root.parent.parent/'track-tread-candidate-20261118/route-relieved/scan.json').read_bytes())
backing=[p for p in inventory['inventory'] if 'fixed bumper' in p['name'] and 'backing' in p['name'] and 'support' not in p['name']]
assert len(backing)==4
frame=np.array([[-1.,0.,0.],[0.,0.,1.],[0.,1.,0.]])
def rz(a):
 c,s=math.cos(a),math.sin(a);return np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])
def tf(shape,m):
 tr=gp_Trsf();tr.SetValues(*m.ravel());return BRepBuilderAPI_Transform(shape,tr,True).Shape()
stream=root.parent/'result-refined/trace.jsonl'
with stream.open('rb') as f:
 for l in f: pass
q=json.loads(l)['q']
hits={}
from itertools import product
for angle in [147.5,149.25,150.5,152.]:
 base=rz(np.deg2rad(angle));rot=frame.T@base@rz(q[2])
 pos=np.array([114*.0254,275.625*.0254,0.])+base@np.array([q[0]-.658,q[1],0.])
 trans=frame.T@(1000*pos-[8270.875,4105.275,-4.7752]);found=[]
 for body in backing:
  lo,hi=np.asarray(body['boundsMm']).reshape(2,3)
  assert abs(np.prod(hi-lo)-body['volumeMm3'])<.00001
  corners=np.asarray(list(product(*zip(lo,hi))))@rot.T+trans
  candidate=np.flatnonzero(((corners.max(0)>=bounds[:,0])&(corners.min(0)<=bounds[:,1])).all(axis=1))
  robot=tf(BRepPrimAPI_MakeBox(gp_Pnt(*lo),gp_Pnt(*hi)).Shape(),np.column_stack((rot,trans)))
  for i in candidate:
   f=field[i];a=read_shape(f['brepLocal']);other=tf(a,np.asarray(f['fieldCadFromPart']))
   op=BRepAlgoAPI_Common(robot,other);op.Build();assert op.IsDone()
   prop=GProp_GProps();BRepGProp.VolumeProperties_s(op.Shape(),prop)
   if prop.Mass()>.001:found.append([body['key'],f['path'][-1],prop.Mass()])
 hits[str(angle)]=found
 print(angle,len(found),found[:2],flush=True)
(root.parent/'outer-at-release-bumper.json').write_text(json.dumps(hits,indent=2))
