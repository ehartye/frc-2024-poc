"""Surface-grid phase sensitivity: torus yaw changes no physical geometry here."""
import json,pathlib,sys,hashlib,math
R=pathlib.Path(__file__).resolve().parents[2];sys.path[:0]=[str(R/'sim/pressure'),str(R/'sim/integrated')]
from finite_area import evaluate
from patch_registration import rotation
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def run(output):
 if output.exists():raise ValueError('fresh output required')
 source=R/'sim/pressure/evidence.json';report=json.loads(source.read_text());pins=dict(report['inputs']);pins['sim/pressure/evidence.json']=sha(source);pins['sim/pressure/phase_sensitivity.py']=sha(pathlib.Path(__file__))
 if any(sha(R/f)!=h for f,h in pins.items()):raise ValueError('stale source')
 c=json.loads((R/'cad/assembly/intake_components.json').read_text());rows=[]
 for n in [64,128,256,512,1024]:
  for cell_phase in [0.,.25,.5]:
   angle=cell_phase*2*math.pi/n;q=evaluate(c,note_center=[.010567440020061214,0,0],note_basis=rotation(angle,0),resolution=(n,n));rows.append(dict(resolution=n,majorCellPhase=cell_phase,angleRad=angle,result=q));print(n,cell_phase,q['potentialJ'],q['sides'][1]['normalMagnitudeN'],q['noteMomentWorldNm'][2],flush=True)
 if any(sha(R/f)!=h for f,h in pins.items()):raise ValueError('source changed')
 r=dict(schema='finite-pressure-grid-phase/1',inputs=pins,cases=rows,acceptance=False,meaning='Axisymmetric torus geometry is unchanged by yaw; variation with grid phase is numerical quadrature anisotropy, not physical material torque or evidence of grip.')
 output.parent.mkdir(parents=True,exist_ok=True);temp=output.with_suffix('.tmp');temp.write_text(json.dumps(r,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');temp.replace(output)
if __name__=='__main__':run(pathlib.Path(sys.argv[1]))
