from pathlib import Path
import runpy,json,hashlib,numpy as np
q=Path(__file__).parent;w=runpy.run_path(str(q/'support.py'));X=w['xf'];B=w['box'];C=w['cyl'];common=w['common'];distance=w['distance'];sw=next(iter(w['load_solids'](q/'switch.step')[0].values()))
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
parts={};wires=[];controls={}
for label,R,t in [('left',[[1,0,0],[0,0,-1],[0,1,0]],[-.93,40.92,75]),('right',[[1,0,0],[0,0,1],[0,-1,0]],[-.93,-40.92,75]),('rear',[[0,0,1],[1,0,0],[0,1,0]],[-7.92,-10.93,62])]:
 R=np.array(R);t=np.array(t);switch=X(sw,R,t);parts[label]=(switch,R,t)
 # Three distinct terminal pull directions through the vendor's 3.2 mm fast-on envelope; use a 4mm bore and 16 mm straight withdrawal for each.
 wires.extend([(label,i,X(C([x,0,9.6],[0,0,1],2,16),R,t)) for i,x in enumerate([-7.7,1.2,8.5])])
def post(label,stroke):
 s=1 if label=='left' else -1
 if label=='rear':return B([-79-stroke,-4,6],[-71-stroke,4,92])
 y=s*(78+stroke);return B([6,y-4,6],[14,y+4,92])
for label,(sw,R,t) in parts.items():
 home=-53 if label=='rear' else -23;sel=[]
 # Crop a conservative source-space bump-tip region that excludes rigid housing. Isolate all nominal common to that lever.
 bump=X(B([9,-3.25,-10.7],[12,3.25,-8.3]),R,t)
 for dd in [0,.25,.5,.75,1,3,10,34-home]:
  p=post(label,home+dd);total=common(sw,p);clip=BRepAlgoAPI_Common(sw,p).Shape()
  tip=common(clip,bump) if total>1e-8 else 0
  sel.append(dict(mmBehindHome=dd,rigidFreeModelCommonMm3=total,inLeverTipMm3=tip,bodyCommonOutsideTipMm3=total-tip,freeGapMm=distance(sw,p)))
 controls[label]=sel
service={}
for lab,i,g in wires:
 service[f'{lab}/{i}']={'otherSwitchCommonMm3':sum(common(g,w) for k,(w,_,_)in parts.items() if k!=lab),'homePostCommonMm3':sum(common(g,post(k,-53 if k=='rear' else -23)) for k in parts),'otherTerminalCommonMm3':sum(common(g,h) for ll,ii,h in wires if (ll,ii)!=(lab,i))}
out=dict(modelSHA=hashlib.sha256((q/'switch.step').read_bytes()).hexdigest(),positionedThreeSwitches=controls,wireReservationDiameterMm=4,wirePullLengthMm=16,service=service,logic='Require home->away->home transition per physical finger, with distinct NC and NO channels complementary at stable states. Require all three home plus raised/cleared head before feed. Lost wire, stuck contact, no transition, disagreement or timeout veto feed, stop toes. A garage test or native rigid contact does not define trip or overtravel; calibrate start/release, differential, force and actuation before assigning clearance.',stuckRear={'rearMmBeforeHome':10,'healthyLeftAndRightAtHome':True,'rearGeometricContact':False,'permitFeed':False})
(q/'three-witnesses-lowered.json').write_text(json.dumps(out,indent=1));print({k:(v[0],v[-2]) for k,v in controls.items()},service)

