"""Fresh source-bound full nominal trace equivalence, with unchanged supply scan."""
import pathlib,sys,json,hashlib,time
R=pathlib.Path(__file__).resolve().parents[2];sys.path[:0]=[str(R/'sim/integrated'),str(R/'sim/acquisition')]
from electrical_midpoint import ElectricalOnlyCoupled
from coupled_acquisition import CoupledAcquisition,DomainExit
import run_free_planar,free_geometry,obstacle_gate
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
FILES=['sim/integrated/verify_electrical_midpoint.py','sim/integrated/electrical_midpoint.py','sim/integrated/coupled_acquisition.py','sim/integrated/shared_supply.py','sim/drivetrain/plant.py','sim/drivetrain/config.json','sim/acquisition/free-planar-config.json','cad/assembly/intake_components.json']
def run(output):
 if output.exists():raise ValueError('fresh output required')
 obs,_,pins=run_free_planar.load(R/'sim/acquisition/free-obstacles.json');pins.update({f:sha(R/f) for f in FILES})
 dc=json.loads((R/'sim/drivetrain/config.json').read_text());nc=json.loads((R/'sim/acquisition/free-planar-config.json').read_text())['model'];cad=json.loads((R/'cad/assembly/intake_components.json').read_text());loads=[dc['massKg']*9.80665/2]*2
 geom=free_geometry.make_geometry(cad,obstacle_gate.make_gate(obs,cad['note']['majorRadius'],cad['note']['tubeRadius'],cad['floorCenterZ']))
 init=dict(p=[.658,0.],v=[0.,0.],yaw=0.,spin=0.,w=[0.,0.]);models=[CoupledAcquisition(dc,nc,geom,init),ElectricalOnlyCoupled(dc,nc,geom,init)]
 timings=[0.,0.];digests=[hashlib.sha256(),hashlib.sha256()];trace=[];status='HORIZON';maxres=0.;firstcontact=None;steps=0
 for i in range(1000):
  rows=[]
  for j,p in enumerate(models):
   start=time.perf_counter()
   try:row=p.step(.001,(.2,.2),(.12,.12),loads)
   except (DomainExit,ValueError) as e:row={'exception':type(e).__name__,'message':str(e)}
   timings[j]+=time.perf_counter()-start;rows.append(row);digests[j].update(json.dumps(row,sort_keys=True,allow_nan=False).encode())
  if rows[0]!=rows[1]:raise ValueError('trace divergence at step '+str(i))
  if 'exception' in rows[0]:status='UNKNOWN_DOMAIN: '+rows[0]['message'];break
  steps+=1;row=rows[0];maxres=max(maxres,abs(row['energyJ']['residual']))
  if row['contacts'] and firstcontact is None:firstcontact=row['timeS']
  if i%25==0:trace.append(row)
  if i%100==0:print(i,row['timeS'],timings,flush=True)
 if rows and ('exception' not in rows[0]) and (not trace or trace[-1]['timeS']!=rows[0]['timeS']):trace.append(rows[0])
 if any(sha(R/f)!=h for f,h in pins.items()):raise ValueError('inputs changed')
 result=dict(schema='electrical-midpoint-equivalence/1',inputs=pins,stepS=.001,horizonS=1.,steps=steps,status=status,exactEveryRowEqual=True,traceSha256=[d.hexdigest() for d in digests],wallSeconds=dict(reference=timings[0],electricalOnly=timings[1]),firstContactS=firstcontact,maxAbsEnergyResidualJ=maxres,trace=trace,acceptance=False,limits=['Preserves full 513-point sampled root search, not certified global root discovery.','Exact equality verified for this complete nominal model trace; no physical calibration or completed robot claim.','Hypothetical tires/compliance/mass and conservative obstacle-domain stops unchanged.'])
 output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');print(status,steps,timings,maxres,flush=True)
if __name__=='__main__':run(pathlib.Path(sys.argv[1]))
