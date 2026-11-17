"""Fresh static contact/velocity queries; not a simulated acquisition episode."""
import pathlib,sys,json,hashlib
R=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(R/'sim/pressure'))
import finite_transport as t
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
FILES=['sim/pressure/finite_transport.py','sim/pressure/run_transport.py','tests/test_finite_pressure_transport.py']
def run(output):
 if output.exists():raise ValueError('fresh output required')
 source=json.loads((R/'sim/pressure/evidence.json').read_text());pins=dict(source['inputs']);pins['sim/pressure/evidence.json']=sha(R/'sim/pressure/evidence.json');pins.update({f:sha(R/f) for f in FILES})
 if any(sha(R/f)!=h for f,h in pins.items()):raise ValueError('stale source')
 c=json.loads((R/'cad/assembly/intake_components.json').read_text());radius=c['pinch']['teeth']*c['pinch']['pitch']/1000/(2*t.math.pi);rows=[]
 motions=[dict(name='positive-acquisition',rotor_omega=[30.,30.]),dict(name='reverse-asymmetric',rotor_omega=[-10.,4.],note_velocity=[.1,.02,0.],note_omega=[0.,0.,.03],cradle_velocity=[.05,-.01,0.],cradle_omega=[0.,0.,-.02]),dict(name='zero-friction-control',rotor_omega=[30.,30.],mu=0.)]
 for fixture,center in [('no-contact',[-1.,0.,0.]),('front-wrap',[.158,0.,0.]),('indexed-diagnostic',[.010567440020061214,0.,0.])]:
  args=dict(note_center=center,resolution=(512,512));p=t.prepare(c,**args)
  for motion in motions:
   kw={k:v for k,v in motion.items() if k!='name'};v=t.rates(p,pitch_radius_m=radius,**kw);rows.append(dict(fixture=fixture,motion=motion['name'],geometryArguments=args,rateArguments=dict(kw,pitch_radius_m=radius),potentialJ=p['potentialJ'],features=p['features'],activeSamples=len(p['contacts']),result=v));print(fixture,motion['name'],len(p['contacts']),p['features'],v['workResidualW'],flush=True)
 if any(sha(R/f)!=h for f,h in pins.items()):raise ValueError('source changed')
 report=dict(schema='finite-pressure-transport-interface/1',inputs=pins,cases=rows,acceptance=False,limits=['Static geometry/rate queries only; not a trajectory or acquisition result.','End-wrap gradient/tangent extends the straight-only standalone law, not its archived results.','Source belt tangent uses inherited pitch-radius speed; outer-layer strain is uncalibrated.','No motor state/current/battery is hidden here: signed rotor reaction torque and work are outputs for a joint integrator.','Material pressure and friction are hypothetical; deep/cavity/tie contacts remain unknown.','No old-checkpoint potential reset or continuity claim.','512 grid used here for an interface check, not a whole-trajectory convergence certificate.'])
 output.parent.mkdir(parents=True,exist_ok=True);temp=output.with_suffix('.tmp');temp.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');temp.replace(output)
if __name__=='__main__':run(pathlib.Path(sys.argv[1]))
