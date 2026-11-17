"""Rigid full-NOTE feed screen against intake only; all raw hard contacts retained."""
import pathlib,json,math,hashlib,argparse
import build_connected_candidate as connected
from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
from OCP.gp import gp_Ax2,gp_Pnt,gp_Dir
assembly=connected.assembly;h=connected.h

def possible_torus_box(q,advance,major,tube):
 # Conservative annular and height rejection in the NOTE-aligned frame.
 dz=max(q[2],-q[5],0)
 if dz>tube:return False
 dx=max(q[0]-advance,advance-q[3],0);dy=max(q[1],-q[4],0)
 minimum=math.hypot(dx,dy)
 maximum=math.hypot(max(abs(q[0]-advance),abs(q[3]-advance)),max(abs(q[1]),abs(q[4])))
 reach=math.sqrt(max(0,tube*tube-dz*dz))
 return minimum<=major+reach+1e-6 and maximum>=major-reach-1e-6

def run(cache=None):
 before=connected.input_pins();script=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest();c=json.loads((assembly.ROOT/'cad/assembly/intake_components.json').read_text());a=math.radians(c['pitchDeg']);parts=assembly.intake_components.build('handoff');rows=[];native=[];source=None
 if cache:
  p=cache/'REV-21-2201.step';expected='c2825524b2293af12647d92ad57e9162379ad64105b86f867be2223037059c68'
  if hashlib.sha256(p.read_bytes()).hexdigest()!=expected:raise RuntimeError('pulley source mismatch')
  r=h.STEPControl_Reader()
  if r.ReadFile(str(p))!=h.IFSelect_RetDone:raise RuntimeError('pulley import failed')
  r.TransferRoots();pulley=r.OneShape();print('native pulley imported',flush=True)
  if not h.BRepCheck_Analyzer(pulley).IsValid():raise RuntimeError('invalid pulley')
  source={'part':'REV-21-2201','sha256':expected,'boundsMm':assembly.robot.bounds(pulley),'nativeRedistributed':False}
  pitch=c['pinch']['pitch'];teeth=c['pinch']['teeth'];center=(c['pinch']['beltTeethHypothesis']-teeth)*pitch/2;yabs=c['pinch']['innerFaceYAbs']+teeth*pitch/(2*math.pi)+1
  for side in [-1,1]:
   for x in [-center/2,center/2]:native.append((str((side,x)),h.pitch(h.move(pulley,x,side*yabs,0),a,[c['stageCenterX'],0,c['handoffCenterZ']])))
 def local_bounds(s):return assembly.robot.bounds(h.pitch(h.move(s,-c['stageCenterX'],0,-c['handoffCenterZ']),-a,[0,0,0]))
 prepared=[(n,s,local_bounds(s)) for n,s,k in parts];witness=[(n,s,local_bounds(s)) for n,s in native]
 for advance in range(0,481,10):
  note=BRepPrimAPI_MakeTorus(gp_Ax2(gp_Pnt(c['stageCenterX']+advance*math.cos(a),0,c['handoffCenterZ']+advance*math.sin(a)),gp_Dir(-math.sin(a),0,math.cos(a))),c['note']['majorRadius'],c['note']['tubeRadius']).Shape();b=assembly.robot.bounds(note);hits=[];nativehits=[]
  for group,dest in [(prepared,hits),(witness,nativehits)]:
   for n,s,q in group:
    if not possible_torus_box(q,advance,c['note']['majorRadius'],c['note']['tubeRadius']):continue
    volume=abs(connected.upper.lower.volume(note,s))
    if volume>1e-5:dest.append({'part':n,'commonMm3':volume,'intendedContactHypothesis':'side pinch belt' in n})
  rows.append({'advanceMm':advance,'hits':hits,'nativePulleyHits':nativehits});print(advance,[x['part'] for x in hits if not x['intendedContactHypothesis']],flush=True)
 if before!=connected.input_pins() or script!=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest():raise RuntimeError('source changed')
 return {'schema':'intake-rigid-feed-screen/1','inputs':before,'scriptSha256':script,'nativePulleySource':source,'samples':rows,'acceptance':False,'limits':['Intake only; launcher and full robot separately screened','49 finite guide-aligned rigid NOTE positions, not continuous dynamics','Raw pulley/bearing/support contacts are not waived','Belt traction and foam compression uncalibrated','Native pulley retained privately; no imported supplier geometry exported']}
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--vendor-cache',type=pathlib.Path);v=p.parse_args();q=run(v.vendor_cache);out=assembly.ROOT/'artifacts/intake-component-candidate/intake-feed.json';out.write_text(json.dumps(q,indent=2)+'\n',encoding='utf-8')
