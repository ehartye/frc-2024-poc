"""Exact native diagnostic at immutable free-acquisition stopped states, not path acceptance."""
import json,math,pathlib,hashlib,argparse
import build_connected_candidate as a
from check_native_floor_entry import common,possible_torus_box
from OCP.TopAbs import TopAbs_SOLID,TopAbs_FACE
from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
EXPECTED='1e3bf6d0566691bce05bfaed512f0ab1226eecef8587803fe8da988f25cb0d26'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def pins():
 p=a.input_pins()
 for f in ['cad/assembly/check_free_acquisition_witness.py','cad/assembly/check_native_floor_entry.py','cad/assembly/audit_cartridge_interfaces.py','cad/assembly/audit_intake_feed.py','cad/components/a2-wheel-cartridge.json']:p[f]=sha(a.ROOT/f)
 return p
def run(cache,experiment):
 before=pins();ep=experiment/'sim/acquisition/free-planar-evidence.json'
 if sha(ep)!=EXPECTED:raise RuntimeError('wrong experiment receipt')
 evidence=json.loads(ep.read_text())
 def check_external():
  if sha(ep)!=EXPECTED or any(sha(experiment/k)!=v for k,v in evidence['inputs'].items()):raise RuntimeError('experiment stale')
 check_external()
 for k,v in a.input_pins().items():
  if evidence['inputs'].get(k)!=v:raise RuntimeError('geometry differs '+k)
 c=json.loads((a.ROOT/'cad/assembly/intake_components.json').read_text());p=c['pinch'];native={}
 sources={'pulley':{'file':'REV-21-2201.step','sha256':'c2825524b2293af12647d92ad57e9162379ad64105b86f867be2223037059c68','solids':3},'bearing':{'file':'REV-21-1916-PK4.step','sha256':'3e16cb3f5f215288b8706eb8228d9580fea78c425eee536ef95195b5b98f02e6','solids':1}}
 for name,spec in sources.items():
  path=cache/spec['file']
  if sha(path)!=spec['sha256']:raise RuntimeError('source changed')
  reader=a.h.STEPControl_Reader()
  if reader.ReadFile(str(path))!=a.h.IFSelect_RetDone:raise RuntimeError('read failed')
  reader.TransferRoots();shape=reader.OneShape();ex=a.h.TopExp_Explorer(shape,TopAbs_SOLID);ss=[]
  while ex.More():
   s=ex.Current()
   if not a.h.BRepCheck_Analyzer(s).IsValid():raise RuntimeError('invalid source')
   ss.append(s);ex.Next()
  if len(ss)!=spec['solids'] or a.h.TopExp_Explorer(shape,TopAbs_FACE,TopAbs_SOLID).More():raise RuntimeError('source topology')
  native[name]=ss
 radius=p['teeth']*p['pitch']/(2*math.pi);yabs=p['innerFaceYAbs']+radius+1;center=(p['beltTeethHypothesis']-p['teeth'])*p['pitch']/2;placed=[]
 for side in [-1,1]:
  for x in [p['beltCenterX']-center/2,p['beltCenterX']+center/2]:
   for i,s in enumerate(native['pulley']):
    w=a.h.move(s,c['stageCenterX']+x,side*yabs,c['floorCenterZ']);placed.append((f'pulley/{side}/{x}/{i}',w,a.assembly.robot.bounds(w)))
   for z in [p['lowerBearingZ'],p['upperBearingZ']]:
    for flip in [False,True]:
     s=native['bearing'][0]
     if flip:
      t=a.h.gp_Trsf();t.SetRotation(a.h.gp_Ax1(a.h.gp_Pnt(0,0,0),a.h.gp_Dir(1,0,0)),math.pi);s=a.h.BRepBuilderAPI_Transform(s,t,True).Shape()
     w=a.h.move(s,c['stageCenterX']+x,side*yabs,c['floorCenterZ']+z);placed.append((f'bearing/{side}/{x}/{z}/flip{flip}',w,a.assembly.robot.bounds(w)))
 rows=[];R=c['note']['majorRadius'];r=c['note']['tubeRadius']
 selected=[case for case in evidence['cases'] if case['name'] in ['right','stiff-right','stiffer-right']]
 if len(selected)!=3:raise RuntimeError('case inventory')
 for case in selected:
  for state_name in ['final','stoppedAttempt']:
   state=case[state_name];x,y=[v*1000 for v in state['relativeRobotXY']];z=c['floorCenterZ'];note=BRepPrimAPI_MakeTorus(a.h.gp_Ax2(a.h.gp_Pnt(x,y,z),a.h.gp_Dir(0,0,1)),R,r).Shape();checks=[]
   for name,s,b in placed:
    relative=[b[0]-x,b[1]-y,b[2]-z,b[3]-x,b[4]-y,b[5]-z]
    possible=possible_torus_box(relative,0,R,r);v=common(s,note) if possible else 0
    checks.append({'part':name,'commonMm3':v,'conservativeRejected':not possible})
   rows.append({'case':case['name'],'state':state_name,'centerRobotMm':[x,y,z],'checks':checks,'hits':[q for q in checks if q['commonMm3']>1e-5]});print(case['name'],state_name,len(rows[-1]['hits']),flush=True)
 check_external()
 if before!=pins() or any(sha(cache/s['file'])!=s['sha256'] for s in sources.values()):raise RuntimeError('inputs changed')
 report={'schema':'free-acquisition-native-witness/1','inputs':before,'experimentReceiptSha256':EXPECTED,'experimentInputs':evidence['inputs'],'sources':sources,'rows':rows,'acceptance':False,'limits':['Six exact diagnostic states, not continuous trajectory clearance','Flat nominal rigid NOTE; foam compliance and measured forces absent','Pulley phase not integrated or registered: source-clock material witness only, not all rotating tooth phases','Original unseated bearing centers and both flange orientations retained; new seating study not adopted','Only track pulleys and bearings checked, not whole robot','Conservative gate stops remain unknown outside these sampled material witnesses','No vendor geometry distributed']}
 out=a.ROOT/'artifacts/free-acquisition-native';out.mkdir(parents=True,exist_ok=True);(out/'report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8',newline='\n')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--vendor-cache',type=pathlib.Path,required=True);p.add_argument('--experiment',type=pathlib.Path,required=True);args=p.parse_args();run(args.vendor_cache,args.experiment)
