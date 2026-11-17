import pathlib,sys,json,hashlib
from dataclasses import asdict
R=pathlib.Path(__file__).resolve().parents[2];sys.path[:0]=[str(R/'sim/pressure'),str(R/'cad/assembly')]
import cassette_inertia as c
FILES=['sim/pressure/cassette_inertia.py','sim/pressure/run_cassette_inertia.py','tests/test_cassette_inertia.py','cad/assembly/powered_cam_contact.py','cad/assembly/intake_components.json','sim/drivetrain/config.json']
def pins():return {p:hashlib.sha256((R/p).read_bytes()).hexdigest() for p in FILES}
def run(output):
 if output.exists():raise ValueError('fresh output required')
 before=pins();p=c.default_partition();rows=[]
 fixtures=[('locked-moving',[0,0,.4,0,0,.2,.2],[.3,-.2,.7,0,0,0,0],[45,90,18,0,0,0,0],(3,4,5,6)),('symmetric-internal',[0,0,0,0,0,.2,.2],[0]*7,[0,0,0,1,1,0,0],()),('asymmetric-internal',[0,0,0,0,0,.2,.2],[0]*7,[0,0,0,1,0,0,0],()),('moving-asymmetric',[.2,-.1,.7,.0004,.0008,.3,.5],[.4,-.2,.9,.003,-.002,.1,-.2],[2,-3,.4,4,-2,.001,-.002],())]
 for name,q,v,f,locked in fixtures:rows.append(dict(name=name,q=q,velocity=v,forces=f,locked=locked,result=c.acceleration(p,q,v,f,locked)))
 if before!=pins():raise ValueError('source drift')
 report=dict(schema='cassette-planar-inertia/1',inputs=before,partition=asdict(p),fixtures=rows,acceptance=False,limits=['Pure mass/bias and force-to-acceleration port; not a trajectory or pressure integration.','42+1.5+1.5 kg is an explicit partition hypothesis of the existing45 kg, not measured moving masses.','Moving COMs use candidate belt center/face coordinates as named assumptions, not CAD mass properties. Fixed COM/yaw inertia are adjusted to recover locked aggregate at u=0.','Moving groups are point-mass approximations; relative cam inertias are output-referred hypotheses with roll/pitch reactions constrained.','No old checkpoint is imported or re-referenced; actual dynamics must declare initial kinetic and potential energy.'])
 output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n')
if __name__=='__main__':run(pathlib.Path(sys.argv[1]))
