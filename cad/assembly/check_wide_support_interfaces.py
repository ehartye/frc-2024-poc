"""Nominal support contact faces and mutual checks, not weld qualification."""
import pathlib,json,hashlib
import servo_opening_wide_support as m
from audit_cartridge_interfaces import common
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import TopoDS
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Plane

def horizontal_faces(shape,z):
 faces=[];ex=m.h.TopExp_Explorer(shape,TopAbs_FACE)
 while ex.More():
  f=TopoDS.Face(ex.Current());a=BRepAdaptor_Surface(f)
  if a.GetType()==GeomAbs_Plane:
   plane=a.Plane()
   if abs(plane.Location().Z()-z)<1e-7 and abs(abs(plane.Axis().Direction().Z())-1)<1e-7:faces.append(f)
  ex.Next()
 return faces

def face_area(a,b,z):
 area=0
 for fa in horizontal_faces(a,z):
  for fb in horizontal_faces(b,z):
   op=BRepAlgoAPI_Common(fa,fb);op.Build();q=GProp_GProps();BRepGProp.SurfaceProperties_s(op.Shape(),q);area+=q.Mass()
 return area

def run():
 root=m.s.a.ROOT;parts=m.local_parts(0);support=next(p for n,p,k in parts if n=='wide original servo support');frame=next(p for n,p,k in parts if 'drilled mounting frame' in n)
 c=json.loads((root/'cad/assembly/intake_components.json').read_text());assembly,_=m.s.build(0,c['floorCenterZ'],0);anchor=next(m.h.move(p,-c['stageCenterX'],0,-c['floorCenterZ']) for n,p,k,role in assembly if 'fixed backbone' in n)
 contacts=[]
 for name,p in [('servo frame',frame),('fixed anchor',anchor)]:
  op=BRepAlgoAPI_Common(support,p);op.Build();prop=GProp_GProps();BRepGProp.SurfaceProperties_s(op.Shape(),prop);d=BRepExtrema_DistShapeShape(support,p);d.Perform()
  contacts.append({'part':name,'commonMm3':common(support,p),'minimumDistanceMm':d.Value(),'booleanContactSurfaceAreaMm2':prop.Mass(),'coplanarFaceIntersectionAreaMm2':face_area(support,p,199 if name=='servo frame' else 116)})
 hits=[]
 for n,p,k in parts:
  if n=='wide original servo support' or k=='source-envelope':continue
  v=common(support,p)
  if v>1e-6:hits.append({'part':n,'commonMm3':v})
 native=[]
 for name,register in [('REV-41-1097.STEP',m.fast.base.register),('REV-41-1363.STEP',m.fast.base.register_horn)]:
  p=pathlib.Path.home()/'.cache/frc-component-cartridge'/name;sha=hashlib.sha256(p.read_bytes()).hexdigest();expected=json.loads((root/'cad/assembly/evidence/servo-opening-fasteners.json').read_text())['nativeSources'][name]['sha256']
  if sha!=expected:raise ValueError('source changed')
  r=m.h.STEPControl_Reader();r.ReadFile(str(p));r.TransferRoots();native.append({'source':name,'sha256':sha,'commonMm3':common(support,register(r.OneShape()))})
 pins=dict(json.loads((root/'cad/assembly/evidence/servo-opening-wide-support.json').read_text())['inputs']);name='cad/assembly/check_wide_support_interfaces.py';pins[name]=hashlib.sha256((root/name).read_bytes()).hexdigest()
 return {'inputs':pins,'contacts':contacts,'authoredMutualHits':hits,'nativeSupportChecks':native,'accepted':False,'limits':['Zero gap/contact face is a proposed weld interface, not weld strength or assembly approval','Weld size, access, material, anchor-plate flexibility and impact/fatigue remain unresolved']}
if __name__=='__main__':
 r=run();(m.s.a.ROOT/'cad/assembly/evidence/wide-support-interfaces.json').write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n');print(r['contacts'],r['authoredMutualHits'],r['nativeSupportChecks'],flush=True)
