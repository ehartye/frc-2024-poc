"""Raw additive cassette-interface conflicts; spring envelope is not a force law."""
import json,hashlib,pathlib
import spring_cassette_candidate as s
from audit_cartridge_interfaces import common
A=s.a

def run():
 before=s.pins();before['cad/assembly/check_spring_cassette.py']=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest();c=json.loads((A.ROOT/'cad/assembly/intake_components.json').read_text());rows=[]
 for state in ['floor','rotated','handoff']:
  angle,z=A.pose(state,c);baseline,_=A.build(state)
  for travel in [0,.75,1.5]:
   new,removed=s.build(angle*180/__import__('math').pi,z,travel);rest=[(n,p,k,'baseline') for n,p,k in baseline if n not in removed];allparts=rest+new;bounds={n:A.assembly.robot.bounds(p) for n,p,k,m in allparts};hits=[];seen=set()
   for n,p,k,m in new:
    for name,q,role,motion in allparts:
     key=tuple(sorted([n,name]))
     if n==name or key in seen or role=='service':continue
     seen.add(key);b=bounds[n];v=bounds[name]
     if min(min(b[i+3],v[i+3])-max(b[i],v[i]) for i in range(3))<=1e-6:continue
     volume=common(p,q)
     if volume>1e-5:hits.append({'a':n,'b':name,'commonMm3':volume,'aRole':k,'bRole':role,'aMotion':m,'bMotion':motion})
   rows.append({'state':state,'travelMm':travel,'candidatePartCount':len(new),'replacedCount':len(removed),'fullPartCount':len(allparts),'hits':hits,'minimumCandidateZmm':min(bounds[n][2] for n,p,k,m in new)});print(state,travel,len(hits),flush=True)
 if any(hashlib.sha256((A.ROOT/k).read_bytes()).hexdigest()!=v for k,v in before.items()):raise ValueError('source changed')
 out=A.ROOT/'artifacts/spring-cassette';out.mkdir(parents=True,exist_ok=True);report={'schema':'spring-cassette-interface-candidate/1','inputs':before,'config':s.CONFIG,'samples':rows,'adopted':False,'acceptance':False,'loadPath':['NOTE reaction -> belt/pulley/bearings -> moving cassette weldment/upper bridge','moving guide blocks -> compression springs -> fixed outer seats/backbone','transverse guide rods carry off-axis loads; travel stops bound motion','fixed backbone -> unresolved pivot-shaft clamp/torque interface -> carriage pivot/lift structure'],'forceContract':{'inwardCoordinateMm':'u in[0,1.5]','springInstalledLengthMm':'7+u, per spring before seat adjustment','sideSpringForce':'sum(k_i*max(0,Lfree_i-(7+u-adjust_i))); k/Lfree unselected','noteNormalForce':'not equal to spring force automatically; cassette inertia, guide friction, belt reactions and contact distribution must be solved'},'limits':['Original one-side editable interface prototype, not complete robot redesign or sourced spring/bushing selection','Current belt centers/pulleys/motor move as one registered cassette; baseline gap applies only atzero travel','All raw conflicts retained, including inherited drive-envelope engagement','Pivot-shaft torque clamp, guide retention/fastening, spring calibration and assembly access remain unresolved','No force acceptance, tolerance/load proof or physical capture; baseline Â±5mm witnesses do not transfer to narrowed gap','Threeposes and three travel samples, not continuous combined sweep','Bearing seat94ab67d not adopted; canonical inherited representations remain']};(out/'report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8',newline='\n')
if __name__=='__main__':run()
