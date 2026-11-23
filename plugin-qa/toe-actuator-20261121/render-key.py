from pathlib import Path
import numpy as np,sys,json,hashlib
from PIL import Image,ImageDraw
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopAbs import TopAbs_FACE,TopAbs_REVERSED
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.TopLoc import TopLoc_Location
q=Path(__file__).parent;sys.path.insert(0,str(Path.home()/'repos/agent-frc/scripts'));from showcase_contacts import load_solids;from stock_sections import entities
from mechanical_motion import bounds
p=q/'key-selected-root.step';ident={};parts,_=load_solids(p,ident);assert set(parts)=={'turned and keyed shaft','Output shaft supplier key'}
key=parts['Output shaft supplier key'];kp=np.array(bounds(key)).reshape(2,3);assert np.max(abs(kp-np.array([[-288.4980199712,194.25,629.188983849],[-286.4980199712,202.75,631.188983849]])))<.001
shaft=parts['turned and keyed shaft'];target=kp.mean(axis=0)+[0,-2.6,-2.1]
look=np.array([1,.42,1.7]);look/=np.linalg.norm(look)
right=np.array([0.,1,0]);right-=look*(right@look);right/=np.linalg.norm(right)
up=np.cross(look,right);up*=np.sign(up[2])
tri=[];report={}
for name,b in parts.items():
 BRepMesh_IncrementalMesh(b,.025,False,.14,False).Perform(); rows=[]
 for f in entities(b,TopAbs_FACE):
  face=TopoDS.Face(f);loc=TopLoc_Location();t=BRep_Tool.Triangulation_s(face,loc);assert t is not None
  xyz=np.array([t.Node(i).Transformed(loc.Transformation()).Coord()for i in range(1,t.NbNodes()+1)])
  for j in range(1,t.NbTriangles()+1):
   inds=np.array(t.Triangle(j).Get())-1; points=xyz[inds]; norm=np.cross(points[1]-points[0],points[2]-points[0]); ln=np.linalg.norm(norm)
   if ln<1e-10:continue
   norm/=ln
   if face.Orientation()==TopAbs_REVERSED:norm=-norm
   rows.append(points)
   depth=(points-target)@look; shade=.48+.52*abs(norm@np.array([.25,-.45,.85])); 
   color=np.array([255,190,69]) if 'supplier' in name else np.array([167,191,209])
   col=tuple((color*shade).astype(int))
   xy=np.stack([((points-target)@right)*38+620,-((points-target)@up)*38+350],axis=1)
   tri.append((depth.mean(),[tuple(pt) for pt in xy],col))
 report[name]={'triangles':len(rows),'bounds':bounds(b),'identity':ident[name]}
scale=2;im=Image.new('RGB',(1200*scale,700*scale),(18,39,59));d=ImageDraw.Draw(im)
for _,xy,col in sorted(tri,key=lambda x:x[0]):d.polygon([(u*scale,v*scale)for u,v in xy],fill=col)
im=im.resize((1200,700),Image.Resampling.LANCZOS);im.save(q/'2024-keyed-shaft-detail.png');report['imageSha256']=hashlib.sha256((q/'2024-keyed-shaft-detail.png').read_bytes()).hexdigest();report['stepSha256']='869021c24d80f57dea7320450657ca91b871ccc44b850c0354ac4d94e977b918';(q/'key-detail.json').write_text(json.dumps(report,indent=2));im.thumbnail((360,210));im.save(q/'detail-check.jpg');print(report)

