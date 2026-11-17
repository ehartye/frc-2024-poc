"""Additive native flange-seat study; leaves current intake assembly unchanged."""
import pathlib,json,hashlib,math,argparse
import build_connected_candidate as a
from audit_cartridge_interfaces import common
from build_cartridge import hexshaft
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut,BRepAlgoAPI_Fuse
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.TopAbs import TopAbs_SOLID,TopAbs_FACE

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def pins():
 p=a.input_pins()
 for n in ['cad/assembly/bearing_seat_candidate.py','cad/assembly/audit_cartridge_interfaces.py','cad/components/build_cartridge.py','cad/components/a2-wheel-cartridge.json']:p[n]=sha(a.ROOT/n)
 return p

def flip(s):
 t=a.h.gp_Trsf();t.SetRotation(a.h.gp_Ax1(a.h.gp_Pnt(0,0,0),a.h.gp_Dir(1,0,0)),math.pi);return a.h.BRepBuilderAPI_Transform(s,t,True).Shape()
def tight_bounds(s):
 b=Bnd_Box();BRepBndLib.AddOptimal_s(s,b,False,True);p,q=b.CornerMin(),b.CornerMax();return [p.X(),p.Y(),p.Z(),q.X(),q.Y(),q.Z()]
def run(cache):
 before=pins();source=json.loads((a.ROOT/'cad/components/a2-wheel-cartridge.json').read_text())['sources']['bearing'];path=cache/'REV-21-1916-PK4.step'
 if sha(path)!=source['sha256']:raise RuntimeError('source mismatch')
 rd=a.h.STEPControl_Reader()
 if rd.ReadFile(str(path))!=a.h.IFSelect_RetDone:raise RuntimeError('source read failed')
 rd.TransferRoots();native=rd.OneShape();ex=a.h.TopExp_Explorer(native,TopAbs_SOLID);count=0
 while ex.More():
  if not a.h.BRepCheck_Analyzer(ex.Current()).IsValid():raise RuntimeError('invalid source')
  count+=1;ex.Next()
 if count!=1 or a.h.TopExp_Explorer(native,TopAbs_FACE,TopAbs_SOLID).More():raise RuntimeError('source topology')
 plate=a.assembly.intake_components.ring_z(0,0,-4.7625,17,14.3,3.175)
 trials=[]
 for phase in [0,30]:
  shaft=a.h.move(hexshaft(20,12.7,13.75,phase),0,0,-10);trials.append({'phaseDeg':phase,'shaftNativeOverlapMm3':common(shaft,native)})
 best=min(trials,key=lambda x:x['shaftNativeOverlapMm3']);shaft=a.h.move(hexshaft(20,12.7,13.75,best['phaseDeg']),0,0,-10)
 outer=BRepAlgoAPI_Fuse(a.h.cyl((0,0,-3.96875),(0,0,1),14.2875,6.35),a.h.cyl((0,0,2.38125),(0,0,1),15.5575,1.5875)).Shape();proxy=BRepAlgoAPI_Cut(outer,a.h.cyl((0,0,-5),(0,0,1),6.875,10)).Shape()
 cases=[]
 for label,do_flip,shift,negative_delta,plate_shift in [('lower-flange-below',True,-2.38125,1,0),('upper-flange-above',False,-3.96875,-1,0),('lower-raised-carrier-2mm',True,-.38125,1,2)]:
  bearing=a.h.move(flip(native) if do_flip else native,0,0,shift);seat_plate=a.h.move(plate,0,0,plate_shift);d=BRepExtrema_DistShapeShape(seat_plate,bearing);d.Perform()
  if not d.IsDone():raise RuntimeError('distance query failed')
  cases.append({'case':label,'flipAboutX180':do_flip,'axialShiftFromOldCenterMm':shift,'carrierRaiseMm':plate_shift,'plateNativeCommonMm3':common(seat_plate,bearing),'minimumSeatGapMm':d.Value(),'wrongAxialOneMmOverlapMm3':common(seat_plate,a.h.move(bearing,0,0,negative_delta)),'seatedNativeBoundsMm':tight_bounds(bearing)})
 c=json.loads((a.ROOT/'cad/assembly/intake_components.json').read_text());floor=c['floorCenterZ']+c['pinch']['lowerBearingZ']+cases[0]['seatedNativeBoundsMm'][2]
 out=a.ROOT/'artifacts/bearing-seat-candidate';out.mkdir(parents=True,exist_ok=True);writer=a.h.STEPControl_Writer()
 for name,s in [('original existing annular carrier study',plate),('original nominal stepped bearing representation',proxy),('original rounded hex test shaft',shaft)]:
  if not a.h.BRepCheck_Analyzer(s).IsValid():raise RuntimeError(name)
  writer.Transfer(s,a.h.STEPControl_AsIs)
 dest=out/'original-seat-study.step'
 if writer.Write(str(dest))!=a.h.IFSelect_RetDone:raise RuntimeError('export failed')
 check=a.h.STEPControl_Reader();check.ReadFile(str(dest));check.TransferRoots()
 if check.NbRootsForTransfer()!=3:raise RuntimeError('export roots')
 if before!=pins() or sha(path)!=source['sha256']:raise RuntimeError('inputs changed')
 r={'schema':'bearing-flange-seat-candidate/1','inputs':before,'source':source,'nativeSolidCount':1,'nativeBoundsMm':tight_bounds(native),'oldEnvelope':{'outerDiameterMm':28.575,'axialLengthMm':6.35},'nominalSteppedProxy':{'bodyRadiusMm':14.2875,'bodyZmm':[-3.96875,2.38125],'flangeRadiusMm':15.5575,'flangeZmm':[2.38125,3.96875],'roundBoreDiameterMm':13.75,'testShaftRoundedHexAcrossFlatsMm':12.7,'testShaftCornerDiameterMm':13.75,'testShaftPhaseDeg':best['phaseDeg']},'shaftClockingTrials':trials,'oversizedShaftControlMm3':common(a.h.move(hexshaft(20,12.9,14,0),0,0,-10),native),'drawing':'https://www.revrobotics.com/content/docs/REV-21-1916-DR.pdf','cases':cases,'lowerBearingGroundGapAtCurrentFloorPoseMm':floor,'raisedLowerBearingGroundGapMm':floor+2,'existingStubExtensionBelowRaisedBearingMm':c['pinch']['lowerBearingZ']+cases[2]['seatedNativeBoundsMm'][2]-c['pinch']['shaftLower'],'originalStepSha256':sha(dest),'originalReopenedRoots':3,'adopted':False,'acceptance':False,'limits':['Actual carrier seating candidate checked against native nominal bearing, not tolerance/load/retention acceptance','Bearing has round13.75mm bore for rounded-hex shaft; zero overlap at0/30deg is not a keyed torque-transfer or clocking proof','Lower flange gives only0.65mm nominal ground clearance; carpet/deflection/tilt not covered','Unraised carrier needs lower stub revision; raised2mm candidate retains existing stub with only0.25mm nominal end extension; axial retention remains open','Nominal stepped representation omits grooves/fillets/internals and is not automatically a conservative material envelope','Existing CAD remains unchanged; full motion, NOTE path, assembly access and axial retention require recheck before adoption','No supplier native geometry exported']}
 (out/'report.json').write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps({'cases':cases,'clocking':trials,'floorGapMm':floor}))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--vendor-cache',type=pathlib.Path,required=True);run(p.parse_args().vendor_cache)
