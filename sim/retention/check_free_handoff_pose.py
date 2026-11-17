"""Preserve the free NOTE final state when testing the fixed CAD handoff boundary."""
import pathlib,sys,json,hashlib,math,argparse
R=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(R/'sim/component-launcher'))
import extract_geometry as e
import build_connected_candidate as connected
from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
from OCP.gp import gp_Ax2,gp_Pnt,gp_Dir
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def run(output):
 if output.exists():raise ValueError('Fresh output required')
 rp=R/'sim/retention/refinement-evidence.json';report=json.loads(rp.read_text())
 if any(sha(R/p)!=h for p,h in report['inputs'].items()):raise ValueError('Stale free-state inputs')
 case=report['cases'][-1]
 if case['stepS']!=.00000625 or case['outcome']!='WITHIN_NOMINAL_PATCH_DOMAIN_HORIZON':raise ValueError('Unsupported state')
 state=case['final'];phi=state['notePitchRad'];center=[state['noteCenterXZ'][0]*1000,0,state['noteCenterXZ'][1]*1000]
 c=json.loads((R/'cad/assembly/intake_components.json').read_text())
 if abs(state['cradlePitchRad']-math.radians(c['pitchDeg']))>1e-12:raise ValueError('CAD cradle pose differs')
 pins=dict(report['inputs'],**connected.input_pins(),**{p.relative_to(R).as_posix():sha(p) for p in [pathlib.Path(__file__),rp,R/'sim/component-launcher/extract_geometry.py']})
 assembly,joints=connected.build('handoff')
 if len(assembly)!=711 or len({n for n,s,k in assembly})!=711:raise ValueError('Assembly identity')
 note=BRepPrimAPI_MakeTorus(gp_Ax2(gp_Pnt(*center),gp_Dir(-math.sin(phi),0,math.cos(phi))),c['note']['majorRadius'],c['note']['tubeRadius']).Shape();bounds=e.b.bounds(note);hits=[];invalid=[]
 for n,s,k in assembly:
  if k=='service':continue
  if not e.solids(s) or not e.BRepCheck_Analyzer(s).IsValid():invalid.append(n);continue
  q=e.b.bounds(s)
  if min(min(bounds[j+3],q[j+3])-max(bounds[j],q[j]) for j in range(3))<=1e-7:continue
  v=e.common_volume(s,note)
  if v>1e-5:hits.append(dict(part=n,overlapMm3=v,group='pinch' if 'side pinch belt' in n else 'launcher' if 'nominal tread annulus' in n else 'unresolved'))
 if any(sha(R/p)!=h for p,h in pins.items()):raise ValueError('Changed source')
 result=dict(schema='frc-free-note-handoff-pose/1',inputs=pins,state=state,centerRobotMm=center,assemblyPartCount=len(assembly),invalidParts=invalid,hits=hits,acceptance=False,limits=['One final free-body pose only; no reset to aligned NOTE or zero velocity.','Original/source-sized CAD representations, including known underbounded bearing profile pending correction.','No transfer trajectory, contact force, continued retention, source-native full assembly or scoring acceptance.'])
 output.mkdir(parents=True);(output/'report.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');print(hits,invalid,flush=True);return result
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--output',type=pathlib.Path,required=True);a=p.parse_args();run(a.output)
