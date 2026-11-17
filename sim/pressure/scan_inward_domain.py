"""Rebuilt-inventory static scan; no collision or capture certification."""
import json,pathlib,sys,hashlib
R=pathlib.Path(__file__).resolve().parents[2];sys.path[:0]=[str(R/'sim/acquisition')]
import inward_domain_gate as domain,free_geometry
FILES=['sim/pressure/scan_inward_domain.py','sim/pressure/inward_domain_gate.py','tests/test_pressure_inward_domain.py','sim/acquisition/obstacle_gate.py','sim/acquisition/free_geometry.py','sim/acquisition/free-obstacles.json','cad/assembly/intake_components.json','cad/assembly/belt_contact_geometry.py']
def scan():
 pins={f:hashlib.sha256((R/f).read_bytes()).hexdigest() for f in FILES}
 r=json.loads((R/'sim/acquisition/free-obstacles.json').read_text());cad=json.loads((R/'cad/assembly/intake_components.json').read_text());g=domain.make_gate(r,152.4,25.4,25.4);q=free_geometry.make_geometry(cad,g)
 rows=[dict(xMm=x,result=q([x/1000,0.])) for x in range(178,137,-1)]
 if any(hashlib.sha256((R/f).read_bytes()).hexdigest()!=h for f,h in pins.items()):raise ValueError('source changed')
 return dict(schema='inward-domain-static-scan/1',inputs=pins,rows=rows,acceptance=False,limits=['Centered samples only; unchanged conservative hard-object rejection and finite belt law.','New experimental lower bound138mm; not physical clearance or capture certification.'])
if __name__=='__main__':
 p=pathlib.Path(sys.argv[1]);p.write_text(json.dumps(scan(),indent=2)+'\n',newline='\n')
