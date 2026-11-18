"""Pinned vendor inputs remain private; exports contain original authored solids only."""
import pathlib,json,math,hashlib,urllib.request,argparse
from OCP.gp import gp_Pnt,gp_Vec,gp_Trsf,gp_Ax1,gp_Dir
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder,BRepPrimAPI_MakeBox,BRepPrimAPI_MakePrism
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon,BRepBuilderAPI_MakeFace,BRepBuilderAPI_Transform
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut,BRepAlgoAPI_Common
from OCP.STEPControl import STEPControl_Reader,STEPControl_Writer,STEPControl_AsIs
from OCP.IFSelect import IFSelect_RetDone
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.BRepCheck import BRepCheck_Analyzer
R=pathlib.Path(__file__).resolve().parents[2]
def props(s):
 p=GProp_GProps();BRepGProp.VolumeProperties_s(s,p);return p

def shifted(s,z,flip=False):
 if flip:
  t=gp_Trsf();t.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(1,0,0)),math.pi);s=BRepBuilderAPI_Transform(s,t,True).Shape()
 t=gp_Trsf();t.SetTranslation(gp_Vec(0,0,z));return BRepBuilderAPI_Transform(s,t,True).Shape()

def hexshaft(L,flat,corner,phase=0):
 w=BRepBuilderAPI_MakePolygon();rad=flat/math.sqrt(3)
 for n in range(6):
  a=math.radians(phase+n*60);w.Add(gp_Pnt(rad*math.cos(a),rad*math.sin(a),0))
 w.Close();s=BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(w.Wire()).Face(),gp_Vec(0,0,L)).Shape();return BRepAlgoAPI_Common(s,BRepPrimAPI_MakeCylinder(corner/2,L).Shape()).Shape()

def ring(ro,ri,L):return BRepAlgoAPI_Cut(BRepPrimAPI_MakeCylinder(ro,L).Shape(),BRepPrimAPI_MakeCylinder(ri,L).Shape()).Shape()
def overlap(a,b):return max(0,props(BRepAlgoAPI_Common(a,b).Shape()).Mass())
def originals(c,phase):
 d=c['layout'];shaft=shifted(hexshaft(d['shaftLength'],d['shaftAcrossFlats'],d['shaftCornerDiameter'],phase),-d['shaftLength']/2);out={'original rounded hex shaft':shaft}
 pw,ph,pt=d['plateSize'];plate=BRepPrimAPI_MakeBox(gp_Pnt(-pw/2,-ph/2,-pt),pw,ph,pt).Shape()
 for x,r in [(0,d['plateBore']/2),(-d['mountHoleSpacing']/2,d['mountHoleDiameter']/2),(d['mountHoleSpacing']/2,d['mountHoleDiameter']/2)]:
  t=gp_Trsf();t.SetTranslation(gp_Vec(x,0,-d['plateSize'][2]));cut=BRepBuilderAPI_Transform(BRepPrimAPI_MakeCylinder(r,d['plateSize'][2]).Shape(),t,True).Shape();plate=BRepAlgoAPI_Cut(plate,cut).Shape()
 for side,bc in zip([-1,1],d['bearingCenters']):
  out['original bored mount plate '+str(side)]=shifted(plate,bc+side*d['bearingFlangeSeatOffset'],side<0)
  sp=ring(d['spacerOuterDiameter']/2,d['spacerInnerDiameter']/2,d['spacerLength']);out['original inner-race spacer '+str(side)]=shifted(sp,side*d['wheelWidth']/2,side<0)
 return out

def run(c,cache,out):
 cache.mkdir(parents=True,exist_ok=True);out.mkdir(parents=True,exist_ok=True);native={};source={}
 for name,r in c['sources'].items():
  f=cache/(r['part']+'.step')
  raw=f.read_bytes() if f.exists() else urllib.request.urlopen(r['url'],timeout=60).read()
  if hashlib.sha256(raw).hexdigest()!=r['sha256']:raise ValueError('source changed '+name)
  if not f.exists():f.write_bytes(raw)
  reader=STEPControl_Reader()
  if reader.ReadFile(str(f))!=IFSelect_RetDone or reader.TransferRoots()!=1:raise ValueError('native import '+name)
  native[name]=reader.OneShape();source[name]={'sha256':r['sha256'],'bytes':len(raw),'part':r['part']}
 native['wheel']=shifted(native['wheel'],-c['layout']['wheelWidth']/2)
 trials=[]
 for phase in [0,30]:
  o=originals(c,phase);shaft=o['original rounded hex shaft'];trials.append({'phaseDegrees':phase,'shaftWheelOverlapMm3':overlap(shaft,native['wheel']),'shaftBearingOverlapMm3':overlap(shaft,shifted(native['bearing'],c['layout']['bearingCenters'][1]))})
 best=min(trials,key=lambda t:t['shaftWheelOverlapMm3']+t['shaftBearingOverlapMm3']);o=originals(c,best['phaseDegrees']);checks={}
 for side,bc,cc in zip([-1,1],c['layout']['bearingCenters'],c['layout']['collarCenters']):
  bearing=shifted(native['bearing'],bc,side<0);collar=shifted(native['collar'],cc,side<0);plate=o['original bored mount plate '+str(side)];sp=o['original inner-race spacer '+str(side)]
  for name,a,b in [('shaft-bearing',o['original rounded hex shaft'],bearing),('shaft-collar',o['original rounded hex shaft'],collar),('plate-bearing',plate,bearing),('spacer-bearing',sp,bearing),('spacer-wheel',sp,native['wheel']),('collar-bearing',collar,bearing)]:checks[name+str(side)]=overlap(a,b)
 # Public STEP consists only of our shaft, bored plates and spacers, no vendor solids.
 writer=STEPControl_Writer();mass=[];rho=c['materials']['densityKgM3']
 for name,s in o.items():
  if not BRepCheck_Analyzer(s).IsValid():raise ValueError(name)
  writer.Transfer(s,STEPControl_AsIs);p=props(s);v=p.Mass();center=p.CentreOfMass();mass.append({'shaftAxisInertiaKgM2':(p.MatrixOfInertia().Value(3,3)+v*(center.X()**2+center.Y()**2))*rho*1e-15,'rotating':('shaft' in name or 'spacer' in name),'id':name,'massKg':v*rho*1e-9,'centroidMm':[center.X(),center.Y(),center.Z()],'basis':'original solid volume times assumed uniform density'})
 if writer.Write(str(out/'original-cartridge.step'))!=IFSelect_RetDone:raise ValueError('write')
 check=STEPControl_Reader()
 if check.ReadFile(str(out/'original-cartridge.step'))!=IFSelect_RetDone:raise ValueError('reopen')
 check.TransferRoots()
 from OCP.TopExp import TopExp_Explorer
 from OCP.TopAbs import TopAbs_SOLID
 ex=TopExp_Explorer(check.OneShape(),TopAbs_SOLID);count=0
 while ex.More():count+=1;ex.Next()
 if count!=len(o):raise ValueError('solid roundtrip')
 p=props(native['wheel']);center=p.CentreOfMass();iw=(p.MatrixOfInertia().Value(3,3)+p.Mass()*(center.X()**2+center.Y()**2))*rho*1e-15
 total=sum(x['massKg'] for x in mass)+p.Mass()*rho*1e-9
 result={'schema':'frc-a2-real-component-cartridge/1','inputConfigSha256':hashlib.sha256(json.dumps(c,sort_keys=True,separators=(',',':')).encode()).hexdigest(),'sourceRecords':source,'clockingTrials':trials,'selectedClockingDegrees':best['phaseDegrees'],'nativePairCommonVolumesMm3':checks,'originalPartMasses':mass,'bareWheelMassKg':p.Mass()*rho*1e-9,'bareWheelShaftInertiaKgM2':iw,'accountedRotatingInertiaKgM2':iw+sum(x['shaftAxisInertiaKgM2'] for x in mass if x['rotating']),'accountedMassKg':total,'massCompleteness':'partial; bearings/collars/motor/gearbox/tread/fasteners/electrical excluded; not robot CoG','originalReopenedSolids':count,'publicStepScope':'five original shaft/plate/spacer solids only; native vendor CAD not redistributed','service':{'driveEndAllowanceMm':c['layout']['shaftLength']/2-max(abs(x) for x in c['layout']['collarCenters'])-c['layout']['collarHalfWidth'],'basis':'nominal axial allocation only; no motor/coupling/tool clearance claim'},'acceptance':'unvalidated original interface candidate; zero common volume is not tolerance/load/fit proof','limits':c['openInterfaces']}
 (out/'report.json').write_text(json.dumps(result,indent=2)+'\n');return result
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--vendor-cache',required=True,type=pathlib.Path);a.add_argument('--output',default=R/'artifacts/component-cartridge',type=pathlib.Path);v=a.parse_args();c=json.loads((R/'cad/components/a2-wheel-cartridge.json').read_text());print(json.dumps(run(c,v.vendor_cache,v.output),indent=2))
