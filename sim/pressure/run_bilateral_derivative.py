import pathlib,sys,json,hashlib
R=pathlib.Path(__file__).resolve().parents[2];sys.path[:0]=[str(R/'sim/pressure'),str(R/'cad/assembly')]
import run_constrained_cassettes as prior
import compare_entry as entry
import bilateral_derivative as c
FILES=['sim/pressure/bilateral_geometry.py','sim/pressure/bilateral_transport.py','sim/pressure/bilateral_derivative.py','sim/pressure/run_bilateral_derivative.py','sim/pressure/cassette_inertia.py','tests/test_bilateral_transport.py','tests/test_bilateral_derivative.py']
def inventory():
 pins=prior.inventory();pins.update({p:hashlib.sha256((R/p).read_bytes()).hexdigest() for p in FILES});return pins
def run(out):
 if out.exists():raise ValueError('fresh output required')
 pins=inventory();p,_=entry.setup();loads=[p.drive.c['massKg']*9.80665/2]*2;f=json.loads((R/'sim/pressure/acquisition-entry-evidence.json').read_text())['final'];rows=[]
 for name,d,n,bus,locked in [('fresh-free',p.drive.state,p.note,12.,()),('historical-contact-locked',f['chassis'],f['note'],f['busV'],(3,4,5,6)),('historical-contact-free-query',f['chassis'],f['note'],f['busV'],())]:
  result=c.derivative(p,d,n,c.Mechanism(tp=.2,tn=.2),bus,[.2,.2],[.12,.12],loads,locked=locked)
  rows.append(dict(name=name,drive=d,note=n,busV=bus,locked=locked,result=result))
 if inventory()!=pins:raise ValueError('source drift')
 report=dict(schema='bilateral-acquisition-derivative/1',inputs=pins,fixtures=rows,acceptance=False,limits=['Pure same-time force/acceleration and energy queries; no trajectory or checkpoint adoption.','Historical contact states are read-only probes, not reinitialized dynamic experiments. New component energy is explicitly separate.','Existing normal/friction/rotor/tire/electrical laws retained; selected bus voltage is supplied for query, not a new shared-supply solve.','Bilateral moving groups and negative-side geometry are hypothetical; actual mounting and calibration unresolved.','Fixed aggregate mass/inertia recovered at zero travel; free-u mass matrix includes chassis reaction.','Moving end-stop events and coupled complementarity remain fail-closed; no momentum clipping.','Extreme transforms with nonzero negative travel explicitly unsupported; zero-negative-travel keeps reviewed scalar fallback.'])
 out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n')
if __name__=='__main__':run(pathlib.Path(sys.argv[1]))
