"""Read-only same-endpoint reconstruction, not a trajectory resume."""
import pathlib,json,copy
import run_acquisition as r
R=r.R
REPORTS=['sim/pressure/acquisition-entry-evidence.json','sim/pressure/acquisition-entry-half-evidence.json']
def setup():
 obs,_,pins=r.run_free_planar.load(R/'sim/acquisition/free-obstacles.json')
 drive=json.loads((R/'sim/drivetrain/config.json').read_text());r.drive_source.validate(drive)
 note=json.loads((R/'sim/acquisition/free-planar-config.json').read_text())['model'];cad=json.loads((R/'cad/assembly/intake_components.json').read_text())
 pins.update({f:r.sha(R/f) for f in r.FILES+r.drive_source.FILES})
 gate=r.obstacle_gate.make_gate(obs,cad['note']['majorRadius'],cad['note']['tubeRadius'],cad['floorCenterZ'])
 p=r.PressureAcquisition(drive,note,r.free_geometry.make_geometry(cad,gate),dict(p=[.658,0.],v=[0.,0.],yaw=0.,spin=0.,w=[0.,0.]),cad)
 return p,pins

def compare():
 p,pins=setup();before=copy.deepcopy((p.drive.state,p.note,p.energy,p.t));rows=[]
 for name in REPORTS:
  data=json.loads((R/name).read_text())
  if data['inputs']!=pins or data['schema']!='fresh-pressure-acquisition/1':raise ValueError('source inventory mismatch')
  if data['outcome']!='HORIZON' or data['pressureConfig']['resolution']!=[1024,1024]:raise ValueError('scope mismatch')
  f=data['final']
  if abs(f['timeS']-.58)>1e-12:raise ValueError('endpoint mismatch')
  rates=p.acquisition_rates(f['note'],f['chassis'],f['busV'],f['commands']['pinch'],{})
  rows.append(dict(report=name,stepS=data['stepS'],timeS=f['timeS'],endRates=rates,maxAbsEnergyResidualJ=data['maxAbsEnergyResidualJ']))
 if before!=(p.drive.state,p.note,p.energy,p.t):raise AssertionError('reconstruction mutated state')
 inputs=dict(pins);inputs.update({f:r.sha(R/f) for f in REPORTS+['sim/pressure/compare_entry.py','tests/test_pressure_entry_comparison.py']})
 return dict(schema='pressure-entry-endpoint-comparison/1',inputs=inputs,rows=rows,acceptance=False,limits=['Pure end-state reconstruction; archived midpoint force samples occur at different times.','Same1024-square grid; temporal comparison does not establish spatial convergence or capture.'])
if __name__=='__main__':
 import sys
 out=pathlib.Path(sys.argv[1])
 if out.exists():raise ValueError('fresh output required')
 result=compare()
 if any(r.sha(R/f)!=h for f,h in result['inputs'].items()):raise ValueError('source changed')
 out.parent.mkdir(parents=True,exist_ok=True);tmp=out.with_suffix('.tmp');tmp.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');tmp.replace(out)
