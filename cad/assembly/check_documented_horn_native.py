"""Private native checks of the exact historically documented retention variant."""
import pathlib,json,hashlib,itertools
import servo_opening_documented_horn as m
from audit_cartridge_interfaces import common
from OCP.TopAbs import TopAbs_SOLID,TopAbs_FACE
from OCP.TopoDS import TopoDS
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Plane
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

def end_faces(shape):
 faces=[];ex=m.h.TopExp_Explorer(shape,TopAbs_FACE)
 while ex.More():
  f=TopoDS.Face(ex.Current());a=BRepAdaptor_Surface(f)
  if a.GetType()==GeomAbs_Plane:
   p=a.Plane()
   if abs(p.Location().X()-130.5)<1e-7 and abs(abs(p.Axis().Direction().X())-1)<1e-7:faces.append(f)
  ex.Next()
 return faces

def seat_area(a,b):
 area=0
 for fa in end_faces(a):
  for fb in end_faces(b):
   op=BRepAlgoAPI_Common(fa,fb);op.Build();prop=GProp_GProps();BRepGProp.SurfaceProperties_s(op.Shape(),prop);area+=prop.Mass()
 return area

def run():
 root=m.s.a.ROOT;cache=pathlib.Path.home()/'.cache/frc-component-cartridge';src={};native={};meta={}
 hashes={'REV-41-1097.STEP':'2ae8de8a60ea26b95021ca04b3a71c2f0a68ec0f3b7cafd27d5b0353f536afb2',**m.SOURCE_SHA}
 for name,sha in hashes.items():
  p=cache/name
  if hashlib.sha256(p.read_bytes()).hexdigest()!=sha:raise ValueError('source hash')
  r=m.h.STEPControl_Reader();r.ReadFile(str(p));r.TransferRoots();shape=r.OneShape()
  if not m.h.BRepCheck_Analyzer(shape).IsValid():raise ValueError('invalid source')
  src[name]=shape;meta[name]={'sha256':sha,'roots':r.NbRootsForTransfer(),'url':'https://www.revrobotics.com/content/cad/'+name}
 native['servo']=m.wide.fast.base.register(src['REV-41-1097.STEP']);native['horn']=m.register_horn(src['REV-41-1828.STEP']);native['center screw']=m.register_screw(src['REV-41-1359.STEP'])
 for dy,dz in m.OFFSETS:native['attachment screw '+str(dy)]=m.register_screw(src['REV-41-1359.STEP'],137,dy,dz)
 rows=[]
 for (a,p),(b,q) in itertools.combinations(native.items(),2):rows.append({'a':a,'b':b,'commonMm3':common(p,q)})
 interfaces=[]
 for n,p,k in m.local_parts(0):
  if k in ['source-envelope','threaded-fastener-envelope']:continue
  for name,q in native.items():
   v=common(p,q)
   interfaces.append({'part':n,'native':name,'commonMm3':v})
 representations={n:p for n,p,k in m.local_parts(0)};containment=[]
 for key,label in [('horn','1828 source-sized conservative envelope'),('center screw','1359 conservative envelope center')]+[('attachment screw '+str(dy),'1359 conservative envelope '+str(dy)) for dy,dz in m.OFFSETS]:
  outside=m.s.cut(native[key],representations[label]);containment.append({'native':key,'outsideRepresentationMm3':common(outside,outside)})
 pocketed=m.s.fuse(m.h.cyl((130.5,196.25,185),(1,0,0),20,3.5),m.h.cyl((127,196.25,185),(1,0,0),4.5,3.5))
 for origin,radius,length in [((126,196.25,185),1.6,9),((126,196.25,185),3,4.5),((131.75,196.25,185),4.25,3)]:pocketed=m.s.cut(pocketed,m.h.cyl(origin,(1,0,0),radius,length))
 rejected=m.s.cut(native['horn'],pocketed);rejected_volume=common(rejected,rejected)
 sb=m.s.a.assembly.robot.bounds(native['center screw']);negative=common(m.h.move(native['horn'],-.5,0,0),native['servo'])
 pins=dict(json.loads((root/'cad/assembly/evidence/servo-opening-wide-support.json').read_text())['inputs'])
 for name in ['servo_opening_documented_horn.py','documented-horn-sources.json','check_documented_horn_native.py']:
  p=root/'cad/assembly'/name;pins['cad/assembly/'+name]=hashlib.sha256(p.read_bytes()).hexdigest()
 for p,sha in pins.items():
  if hashlib.sha256((root/p).read_bytes()).hexdigest()!=sha:raise ValueError('source changed')
 return {'inputs':pins,'nativeSources':meta,'rejectedPocketedEnvelopeOutsideMm3':rejected_volume,'hornSourceBoundsMm':m.s.a.assembly.robot.bounds(native['horn']),'hornRepresentationBoundsMm':m.s.a.assembly.robot.bounds(representations['1828 source-sized conservative envelope']),'nativeRepresentationContainment':containment,'nativePairRows':rows,'authoredInterfaceRows':interfaces,'centerScrewBoundsMm':sb,'nominalAxialSeatFaceAreaMm2':seat_area(native['servo'],native['horn']),'nominalHeadRecessMm':134-sb[3],'hornShiftIntoShaftNegativeMm3':negative,'accepted':False,'limits':['Manufacturer exactpair/length instruction is separate from simplified native spline/thread geometry','Raw thread intersections retained; no load/locking/torque acceptance','Nominal head recess is not a tolerance-qualified margin','Onlytheta0 nativeinterfaces; full robot/contact states checked separately']}
if __name__=='__main__':
 r=run();(m.s.a.ROOT/'cad/assembly/evidence/documented-horn-native.json').write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n');print(r['nativePairRows'],r['nominalHeadRecessMm'],r['hornShiftIntoShaftNegativeMm3']);print([x for x in r['authoredInterfaceRows'] if x['commonMm3']>1e-6],flush=True)
