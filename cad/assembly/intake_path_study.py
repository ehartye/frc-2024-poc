"""Rejectable rigid NOTE pose path, not a capture/contact or actuator simulation."""
import pathlib,json,math,sys,hashlib
ROOT=pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'cad/assembly'))
import build_robot as robot
from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Ax2,gp_Pnt,gp_Dir

def poses():
 # Hypothesis: floor tangent through bumper; rotate fully behind backing; lift then feed.
 for x in list(range(650,179,-10))+[178]:yield 'level intake',x,25.4,0
 for deg in range(0,61,5):yield 'rotate behind backing',178,25.4+152.4*math.sin(math.radians(deg)),deg
 start=25.4+152.4*math.sin(math.radians(60));end=492.153903+96*math.sin(math.radians(60))
 for i in range(1,31):yield 'vertical lift',178,start+(end-start)*i/30,60

def run(open_throat=False):
 parts,dc=robot.build();obstacles=[(n,s,robot.bounds(s)) for n,s,k in parts if k!='service' and not n.startswith('cartridge') and not n.startswith('original carrier')]
 if open_throat:
  revised=[]
  for n,s,q in obstacles:
   if n in ['am-3920-2','am-2974-cut-2','frame fastening bridge-0','frame fastening bridge-1']:
    s=robot.cut(s,robot.h.box(50,-200,-10,400,400,410));q=robot.bounds(s)
   revised.append((n,s,q))
  obstacles=revised
 conflicts=[];records=[]
 for phase,x,z,deg in poses():
  a=math.radians(deg);note=BRepPrimAPI_MakeTorus(gp_Ax2(gp_Pnt(x,0,z),gp_Dir(-math.sin(a),0,math.cos(a))),152.4,25.4).Shape();b=robot.bounds(note);hits=[]
  for n,s,q in obstacles:
   if min(min(b[i+3],q[i+3])-max(b[i],q[i]) for i in range(3))<=1e-6:continue
   v=GProp_GProps();BRepGProp.VolumeProperties_s(BRepAlgoAPI_Common(note,s).Shape(),v)
   if v.Mass()>1e-4:hits.append({'part':n,'commonMm3':v.Mass()})
  row={'phase':phase,'centerMm':[x,0,z],'pitchDeg':deg,'hits':hits};records.append(row)
  if hits:conflicts.append(row)
 return {'schema':'2024-intake-rigid-pose-screen/1','centralThroatHypothesis':open_throat,'inputs':robot.input_pins(),'studySha256':hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),'sampledPoses':len(records),'conflictPoses':conflicts,'poses':records,'acceptance':False,'limits':['Chassis supplier boxes may overestimate collisions; a collision is a review gate not exact native diagnosis','Excludes launcher cartridges and carrier webs to study acquisition architecture first; launch handoff unresolved','Finite pose sampling not continuous swept proof','No physical capture,retention,actuation,torque or friction model','No chassis FRO cut applied yet; baseline rail interference should reject unmodifiedchassis']}
if __name__=='__main__':
 q=run('--open-throat' in sys.argv);p=ROOT/'artifacts/whole-robot/intake-path-study.json';p=p.with_name('intake-path-open-throat.json') if q['centralThroatHypothesis'] else p;p.write_text(json.dumps(q,indent=2)+'\n',encoding='utf-8');print(json.dumps({'poses':q['sampledPoses'],'conflictPoses':len(q['conflictPoses']),'parts':sorted({h['part'] for r in q['conflictPoses'] for h in r['hits']})}))
