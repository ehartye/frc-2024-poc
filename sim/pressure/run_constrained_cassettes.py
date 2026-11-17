"""Exact delegated acquisition prefix, not a dynamic cassette handoff."""
import pathlib,sys,json,copy,hashlib
R=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(R/'sim/pressure'))
import compare_entry as entry
import run_two_cassette as previous
import constrained_cassettes as c
FILES=['sim/pressure/constrained_cassettes.py','sim/pressure/run_constrained_cassettes.py','tests/test_constrained_cassettes.py','sim/pressure/compare_entry.py','sim/pressure/acquisition-entry-evidence.json']
def inventory():
 _,pins=entry.setup();pins.update(previous.inventory())
 pins.update({p:hashlib.sha256((R/p).read_bytes()).hexdigest() for p in FILES});return pins
def run(output):
 if output.exists():raise ValueError('fresh output required')
 pins=inventory();a,_=entry.setup();b=c.FixedCassettes(a);loads=[a.drive.c['massKg']*9.80665/2]*2;rows=[]
 for _ in range(3):
  ref=a.step(.001,[.2,.2],[.12,.12],loads);out=b.step(.001,[.2,.2],[.12,.12],loads)
  if ref!=out['acquisition']:raise AssertionError('fixed-limit prefix differs')
  rows.append(out)
 f=json.loads((R/'sim/pressure/acquisition-entry-evidence.json').read_text())['final']
 expected=a.acquisition_rates(f['note'],f['chassis'],f['busV'],f['commands']['pinch'],{})
 actual=b.plant.acquisition_rates(f['note'],f['chassis'],f['busV'],f['commands']['pinch'],{})
 if expected!=actual:raise AssertionError('same-time contact rates differ')
 query=dict(timeS=f['timeS'],rates=actual,constraints=b.constraint_ports(f['chassis'],f['note']),kind='read-only historical same-law state; not imported or advanced')
 if inventory()!=pins:raise ValueError('source drift')
 result=dict(schema='fixed-cassette-acquisition-limit/1',inputs=pins,trace=rows,contactQuery=query,exactPrefix=True,acceptance=False,limits=['Acquisition, friction, rotors and shared supply are delegated unchanged; bilateral u and theta are locked, not actuated.','Fresh spring/cam initial potential is explicit and constant. No NOTE kinetic duplication or added moving mass on the 45 kg chassis.','Constraint forces have zero relative work; inherited chassis reactions remain in acquisition.','No moving-u release, negative-side hardware adoption, electrical servo, capture or lift acceptance.'])
 output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n')
 return result
if __name__=='__main__':run(pathlib.Path(sys.argv[1]))
