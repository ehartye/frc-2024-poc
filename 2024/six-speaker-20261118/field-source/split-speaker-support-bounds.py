"""Enclose installed sloping subwoofer solids in exact clipped transverse slabs."""
import hashlib,json,itertools
from pathlib import Path
import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepGProp import BRepGProp
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Pnt,gp_Trsf
from inspect_solids import read_shape
root=Path(__file__).parent
field=json.loads((root/'whole-field-leaf-bounds.json').read_bytes())
occ=json.loads((root/'cad-extraction/occurrences.json').read_bytes())
basis=np.array([[-1.,0.,0.],[0.,0.,1.],[0.,1.,0.]])
shift=np.array([8270.875,4105.275,-4.7752])
def bounds(s):
 b=Bnd_Box();BRepBndLib.AddOptimal_s(s,b,False,True)
 if b.IsVoid():return None
 return np.array([b.CornerMin().Coord(),b.CornerMax().Coord()])
def volume(s):
 g=GProp_GProps();BRepGProp.VolumeProperties_s(s,g,1e-10);return g.Mass()
for part in field['parts']:
 name=part['path'][-1]
 if '/GE-24330_Blue/' not in '/'.join(part['path']) or part['bounds'][1][0]>2:continue
 if name not in ('GE-24341','GE-24339','GE-24343-Vinyl','GE-24334','GE-24333_Blue'):continue
 pair=np.array(part['bounds'])*1000
 def coords(row):
  bb=np.array(row['boundsCadMm']).reshape(2,3)
  return bb.mean(axis=0)@basis.T+shift
 records=[o for o in occ if o['path']==part['path'] and not o['children']]
 rec=min(records,key=lambda o:np.linalg.norm(coords(o)-pair.mean(axis=0)))
 mat=basis@np.array(rec['fieldCadFromPart']);mat[:,3]+=shift
 local=read_shape(rec['brepLocal'])
 box=Bnd_Box();BRepBndLib.AddOptimal_s(local,box,False,False)
 corners=np.array(list(itertools.product(*zip(box.CornerMin().Coord(),box.CornerMax().Coord()))))
 placed=corners@mat[:,:3].T+mat[:,3]
 assert np.allclose([placed.min(axis=0),placed.max(axis=0)],pair,rtol=0.,atol=.001)
 tr=gp_Trsf();tr.SetValues(*mat.ravel())
 obj=BRepBuilderAPI_Transform(read_shape(rec['brepLocal']),tr,True).Shape()
 bb=bounds(obj);lo,hi=bb
 cuts=np.linspace(lo[0]-1.,hi[0]+1.,max(2,int(np.ceil((hi[0]-lo[0])/25.))+1))
 cells=[];total=0
 for x,y in zip(cuts,cuts[1:]):
  box=BRepPrimAPI_MakeBox(gp_Pnt(x,lo[1]-1,lo[2]-1),gp_Pnt(y,hi[1]+1,hi[2]+1)).Shape()
  common=BRepAlgoAPI_Common(obj,box);common.Build();assert common.IsDone()
  shape=common.Shape();b=bounds(shape)
  if b is not None:
   b[0]-=.002;b[1]+=.002;cells.append((b/1000).tolist());total+=volume(shape)
 assert abs(total-volume(obj))<1e-6*volume(obj),(total,volume(obj))
 part['tightBounds']=cells
 print(name,len(cells),flush=True)
field['originalBoundsSha256']=hashlib.sha256((root/'whole-field-leaf-bounds.json').read_bytes()).hexdigest()
(root/'whole-field-sliced-bounds.json').write_text(json.dumps(field))
