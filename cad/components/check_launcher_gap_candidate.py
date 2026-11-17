"""Additive nominal gap alternative; leaves the integrated assembly unchanged."""
import pathlib,sys,json,math,hashlib,argparse
ROOT=pathlib.Path(__file__).resolve().parents[2];sys.path[:0]=[str(ROOT/'cad'),str(ROOT/'cad/assembly'),str(ROOT/'cad/components')]
import a2_primitives as h
import build_connected_candidate as connected
import build_robot as b
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common,BRepAlgoAPI_Cut
from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
from OCP.TopAbs import TopAbs_SOLID
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def pins():
 p=connected.input_pins()
 for n in ['cad/components/check_launcher_gap_candidate.py','cad/components/a2-wheel-cartridge.json']:p[n]=sha(ROOT/n)
 return p

def volume(a,b):
 op=BRepAlgoAPI_Common(a,b);op.Build()
 if not op.IsDone():raise RuntimeError('Boolean failed')
 shape=op.Shape()
 if shape.IsNull() or not h.BRepCheck_Analyzer(shape).IsValid():raise RuntimeError('Invalid Boolean output')
 ex=h.TopExp_Explorer(shape,TopAbs_SOLID);v=0
 while ex.More():
  p=GProp_GProps();BRepGProp.VolumeProperties_s(ex.Current(),p);v+=abs(p.Mass());ex.Next()
 return v

def run(cache):
 before=pins();src=json.loads((ROOT/'cad/components/a2-wheel-cartridge.json').read_text())['sources']['wheel'];p=cache/'am-3608.step'
 if sha(p)!=src['sha256']:raise RuntimeError('native source mismatch')
 reader=h.STEPControl_Reader()
 if reader.ReadFile(str(p))!=h.IFSelect_RetDone:raise RuntimeError('source import failed')
 reader.TransferRoots();native=reader.OneShape();ex=h.TopExp_Explorer(native,TopAbs_SOLID);solids=[]
 while ex.More():solids.append(ex.Current());ex.Next()
 if len(solids)!=1 or not h.BRepCheck_Analyzer(native).IsValid():raise RuntimeError('invalid native wheel')
 tr=h.gp_Trsf();tr.SetRotation(h.gp_Ax1(h.gp_Pnt(0,0,0),h.gp_Dir(1,0,0)),-math.pi/2)
 wheel=h.BRepBuilderAPI_Transform(native,tr,True).Shape();box=Bnd_Box();BRepBndLib.AddOptimal_s(wheel,box,False,True);lo=box.CornerMin();hi=box.CornerMax();bounds=[lo.X(),lo.Y(),lo.Z(),hi.X(),hi.Y(),hi.Z()]
 tx=json.loads((ROOT/'cad/components/a2-transmission.json').read_text());t=tx['serviceInterfaces']['tread'];radius=t['nativeSeatRadiusMm']+t['thicknessMm'];i=json.loads((ROOT/'cad/assembly/intake_components.json').read_text())
 rows=[]
 for gap in [46.8,49.2]:
  sep=gap/2+radius;clearance=sep-max(abs(bounds[2]),abs(bounds[5]))-i['note']['tubeRadius'];samples=[]
  for advance in [140,150,160,170,220,270,280,290,300]:
   note=BRepPrimAPI_MakeTorus(h.gp_Ax2(h.gp_Pnt(advance-220,0,0),h.gp_Dir(0,0,1)),i['note']['majorRadius'],i['note']['tubeRadius']).Shape();contacts=[]
   for side in [-1,1]:
    for lane in [-152.4,152.4]:
     native_placed=h.move(wheel,0,lane-t['assemblyTreadCenterOffsetMm']-15.875,side*sep)
     tread=BRepAlgoAPI_Cut(h.cyl((0,lane-t['widthMm']/2,side*sep),(0,1,0),radius,t['widthMm']),h.cyl((0,lane-t['widthMm']/2,side*sep),(0,1,0),t['nativeSeatRadiusMm'],t['widthMm'])).Shape()
     contacts.append({'side':side,'laneMm':lane,'bareWheelOverlapMm3':volume(native_placed,note),'treadOverlapMm3':volume(tread,note)})
   samples.append({'advanceMm':advance,'contacts':contacts})
  rows.append({'finishedGapMm':gap,'nominalIndentationPerSideMm':i['note']['tubeRadius']-gap/2,'bareWheelAlignedPlaneClearanceMm':clearance,'samples':samples});print(gap,flush=True)
 if before!=pins() or sha(p)!=src['sha256']:raise RuntimeError('inputs changed')
 report={'schema':'launcher-gap-candidate/1','inputs':before,'source':src,'nativeSolids':1,'nativeLocalBoundsMm':bounds,'boundMethod':'BRepBndLib.AddOptimal without triangulation, including shape tolerance','cases':rows,'candidateGapMm':49.2,'adopted':False,'acceptance':False,'limits':['Rigid nominal NOTE and single native source clocking; no manufacturing/wear tolerance acceptance','Plane separation covers aligned translation at this body pitch; yaw/roll/deflection are not covered','Tread overlap is geometric indentation only, not measured preload, friction or contact force','Increasing gap moves entire cartridges and requires assembly/mount/service recheck before adoption','Existing launch-speed and power evidence does not transfer']}
 out=ROOT/'artifacts/launcher-gap-candidate';out.mkdir(parents=True,exist_ok=True);(out/'report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8',newline='\n');return report
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--vendor-cache',type=pathlib.Path,required=True);run(p.parse_args().vendor_cache)
