"""Original short shaft and blind-retention screw against private native output socket."""
import pathlib,sys,json,math,hashlib,argparse
ROOT=pathlib.Path(__file__).resolve().parents[2];sys.path[:0]=[str(ROOT/'cad'),str(ROOT/'cad/assembly'),str(ROOT/'cad/components')]
import a2_primitives as h
import build_robot as bounds_helper
from build_cartridge import hexshaft
import fasteners
from OCP.TopAbs import TopAbs_SOLID,TopAbs_FACE
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common,BRepAlgoAPI_Cut
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

def orient_x(s,start):
 tr=h.gp_Trsf();tr.SetRotation(h.gp_Ax1(h.gp_Pnt(0,0,0),h.gp_Dir(0,1,0)),math.pi/2)
 return h.move(h.BRepBuilderAPI_Transform(s,tr,True).Shape(),start,0,0)

def originals(phase=28.75537046065601):
 # Native opening X9.525; socket floor X-1.5875; selected10.5mm insertion.
 start=-.975;shaft=orient_x(hexshaft(49,12.7,13.75,phase),start)
 shaft=BRepAlgoAPI_Cut(shaft,h.cyl((start-.1,0,0),(1,0,0),2.0193,10.1)).Shape()
 bolt=h.cyl((-2.8575,0,0),(1,0,0),2.413,9.525)
 head=h.cyl((-7.6835,0,0),(1,0,0),3.9624,4.826)
 recess=orient_x(fasteners.hex_prism(0,0,0,3.96875,2.5),-7.6835)
 head=BRepAlgoAPI_Cut(head,recess).Shape()
 return [('original rounded hex short shaft with pilot bore',shaft),('nominal #10-32 x3/8 retention screw shank',bolt),('nominal socket head with driver recess',head)]

def volume(s):
 ex=h.TopExp_Explorer(s,TopAbs_SOLID);total=0
 while ex.More():
  prop=GProp_GProps();BRepGProp.VolumeProperties_s(ex.Current(),prop);total+=abs(prop.Mass());ex.Next()
 return total

def boolean_shape(operation):
 operation.Build()
 if not operation.IsDone():raise RuntimeError('native Boolean failed')
 result=operation.Shape()
 if result.IsNull() or not h.BRepCheck_Analyzer(result).IsValid():raise RuntimeError('invalid Boolean result')
 return result

def common(a,b):return volume(boolean_shape(BRepAlgoAPI_Common(a,b)))

def input_pins():
 files=['cad/components/gearbox_output_coupling.py','cad/components/build_cartridge.py','cad/a2_primitives.py','cad/assembly/build_robot.py','cad/electrical/fasteners.py']
 return {name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in files}

def run(path):
 before=input_pins();raw=path.read_bytes();digest=hashlib.sha256(raw).hexdigest()
 if digest!='25fe063c3095015160c63594a90ce4dd69c4d8e464baed9ef4a8ea99b97222c8':raise RuntimeError('private native output subset mismatch')
 reader=h.STEPControl_Reader()
 if reader.ReadFile(str(path))!=h.IFSelect_RetDone:raise RuntimeError('native import failed')
 reader.TransferRoots();shape=reader.OneShape();solids=[];ex=h.TopExp_Explorer(shape,TopAbs_SOLID)
 while ex.More():
  solid=ex.Current()
  if not h.BRepCheck_Analyzer(solid).IsValid():raise RuntimeError('invalid source solid')
  solids.append(solid);ex.Next()
 if len(solids)!=35 or h.TopExp_Explorer(shape,TopAbs_FACE,TopAbs_SOLID).More():raise RuntimeError('unexpected source topology')
 parts=originals();rows=[]
 for name,part in parts:
  if not h.BRepCheck_Analyzer(part).IsValid():raise RuntimeError(name)
  a=bounds_helper.bounds(part);hits=[]
  for i,solid in enumerate(solids):
   b=bounds_helper.bounds(solid)
   if min(min(a[j+3],b[j+3])-max(a[j],b[j]) for j in range(3))<=1e-5:continue
   v=common(part,solid)
   if v>1e-4:hits.append({'nativeSolidIndex':i,'commonMm3':v})
  rows.append({'part':name,'nativeIntersections':hits})
 bad=originals(phase=58.75537046065601)[0][1];negative=sum(common(bad,s) for s in solids)
 threadIntersection=boolean_shape(BRepAlgoAPI_Common(parts[0][1],parts[1][1]));zone=h.cyl((-.975,0,0),(1,0,0),2.414,7.643)
 outside=volume(boolean_shape(BRepAlgoAPI_Cut(threadIntersection,zone)))
 out=ROOT/'artifacts/gearbox-output-coupling';out.mkdir(parents=True,exist_ok=True);writer=h.STEPControl_Writer()
 for name,part in parts:writer.Transfer(part,h.STEPControl_AsIs)
 dest=out/'original-only.step';writer.Write(str(dest));check=h.STEPControl_Reader();check.ReadFile(str(dest));check.TransferRoots()
 if check.NbRootsForTransfer()!=3:raise RuntimeError('original export inventory mismatch')
 if before!=input_pins() or digest!=hashlib.sha256(path.read_bytes()).hexdigest():raise RuntimeError('inputs changed during native check')
 q={'inputs':before,'inputsStable':True,'schema':'gearbox-output-coupling-candidate/1','scriptSha256':hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),'helperSha256':hashlib.sha256((ROOT/'cad/components/build_cartridge.py').read_bytes()).hexdigest(),'nativeSubsetSha256':digest,'nativeAssemblySha256':'3c7ab9aef641e3cbfd317e794d49d983ae80fe46834c155e0a21f0753173ded4','nativeSolidCount':35,'nativeLooseFaceCount':0,'source':'https://www.revrobotics.com/content/cad/MAXPlanetary%202-Stage.STEP','checks':rows,'wrongClock30DegreeNativeOverlapMm3':negative,'nominalThreadZoneOverlapMm3':volume(threadIntersection),'threadIntersectionOutsideDeclaredZoneMm3':outside,'geometry':{'socketAcrossFlatsMm':12.7127,'nominalShaftAcrossFlatsMm':12.7,'sourceSocketDepthMm':11.1125,'selectedInsertionMm':10.5,'floorGapMm':.6125,'selectedScrewUnderHeadMm':9.525,'nominalShaftThreadEngagementMm':7.6425,'proposedPilotDepthMm':10,'screwTipToBlindBottomMm':2.3575,'localShaftBoundsXmm':[-.975,48.025]},'originalStepSha256':hashlib.sha256(dest.read_bytes()).hexdigest(),'originalReopenedRoots':3,'acceptance':False,'limits':['Nominal source geometry only; no tolerance, thread strength, torque or load acceptance','Screw dimensions are standard proxies; exact purchased screw not selected','Thread represented by major-diameter shank against pilot bore within declared engagement zone, not helical threads','Retainer accessed before gearbox stage assembly; whole gearbox/tool access not yet screened','Shaft-to-sprocket collar and motor/output mounting remain separate gates','No native supplier geometry exported']}
 (out/'report.json').write_text(json.dumps(q,indent=2)+'\n',encoding='utf-8',newline='\n');return q
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--native-output',type=pathlib.Path,required=True);v=p.parse_args();q=run(v.native_output);print(json.dumps(q))
