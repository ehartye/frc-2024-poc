"""Source-geometry insertion study; geometric bore band is not a thread specification."""
import pathlib,json,hashlib,math
import servo_opening_fasteners as f
from audit_cartridge_interfaces import common
from OCP.TopAbs import TopAbs_FACE,TopAbs_SOLID
from OCP.TopoDS import TopoDS
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.TopAbs import TopAbs_IN
h=f.h

def run():
 root=f.s.a.ROOT;cache=pathlib.Path.home()/'.cache/frc-component-cartridge';p=cache/'REV-41-1097.STEP';sha=hashlib.sha256(p.read_bytes()).hexdigest()
 if sha!='2ae8de8a60ea26b95021ca04b3a71c2f0a68ec0f3b7cafd27d5b0353f536afb2':raise ValueError('source changed')
 r=h.STEPControl_Reader();r.ReadFile(str(p));r.TransferRoots();raw=r.OneShape();whole=f.base.register(raw)
 ex=h.TopExp_Explorer(raw,TopAbs_FACE);bands=[]
 while ex.More():
  face=TopoDS.Face(ex.Current());a=BRepAdaptor_Surface(face)
  if a.GetType()==GeomAbs_Cylinder:
   cy=a.Cylinder();pt=cy.Axis().Location();axis=cy.Axis().Direction()
   if abs(cy.Radius()-1.25)<1e-7 and abs(pt.X())<1e-7 and abs(pt.Y()+10.05259329813)<1e-7 and abs(abs(axis.Z())-1)<1e-7:
    b=f.s.a.assembly.robot.bounds(f.base.register(face));bands.append([b[0],b[3]])
  ex.Next()
 if not bands:raise ValueError('no bore band')
 lo=min(b[0] for b in bands);hi=max(b[1] for b in bands);rows=[]
 # Conservative band from observed surface bounds, not inferred blind-hole depth.
 target=h.cyl((lo,196.25,185),(1,0,0),1.500001,hi-lo)
 for length in [2,4,6,8,10,12,14]:
  tip=133-length;shank=h.cyl((tip,196.25,185),(1,0,0),1.5,length)
  op=BRepAlgoAPI_Common(shank,whole);op.Build();outside=f.s.cut(op.Shape(),target)
  witnesses=[]
  exsolid=h.TopExp_Explorer(whole,TopAbs_SOLID);native_solids=[]
  while exsolid.More():native_solids.append(exsolid.Current());exsolid.Next()
  classifiers=[BRepClass3d_SolidClassifier(q) for q in native_solids];sc=BRepClass3d_SolidClassifier(shank)
  for x in [127.9,128,128.1,128.2]:
   for j in range(8):
    a=2*math.pi*j/8;pt=h.gp_Pnt(x,196.25+1.4*math.cos(a),185+1.4*math.sin(a))
    sc.Perform(pt,1e-7);inside=False
    if sc.State()==TopAbs_IN:
     for classifier in classifiers:
      classifier.Perform(pt,1e-7)
      if classifier.State()==TopAbs_IN:inside=True;break
    if inside:witnesses.append([pt.X(),pt.Y(),pt.Z()])
  rows.append({'strictSharedINCount':len(witnesses),'strictSharedINExamples':witnesses[:8],'nativeVerdict':'UNKNOWN' if witnesses and common(shank,whole)==0 else 'RAW_MATERIAL','lengthMm':length,'tipXmm':tip,'projectedBandEngagementMm':max(0,min(133,hi)-max(tip,lo)),'extendsPastBandMm':max(0,lo-tip),'rawNativeCommonMm3':common(shank,whole),'commonOutsideObservedBandMm3':common(outside,outside)})
 pins=dict(json.loads((root/'cad/assembly/evidence/servo-opening-fasteners.json').read_text())['inputs'])
 file='cad/assembly/check_servo_retention_band.py';pins[file]=hashlib.sha256((root/file).read_bytes()).hexdigest()
 return {'inputs':pins,'nativeSourceSha256':sha,'surfaceCount':len(bands),'observedBoreRadiusMm':1.25,'observedSurfaceAxialBandMm':[lo,hi],'headSeatingPlaneXmm':133,'rows':rows,'accepted':False,'limits':['Cylindrical face band is geometric evidence only, not certified M3 thread depth, pitch, class or torque','Major-diameter envelope overlaps retained as raw material evidence, not validated thread engagement','Manufacturer center M3 guidance does not specify exact V1-horn8mm screw length','No V2 thread-depth specification transferred','Spline fit and axial seating remain unresolved; geometry does not prove preload or pullout capacity']}
if __name__=='__main__':
 r=run();(f.s.a.ROOT/'cad/assembly/evidence/servo-retention-band.json').write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n');print(r['surfaceCount'],r['observedSurfaceAxialBandMm'],r['rows'],flush=True)
