"""Original editable electrical envelope assembly. Millimetres; no vendor geometry."""
import argparse,hashlib,itertools,json,pathlib,sys
HERE=pathlib.Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from a2_primitives import box,STEPControl_Writer,STEPControl_Reader,STEPControl_AsIs,IFSelect_RetDone,BRepCheck_Analyzer

def load():return json.loads((HERE/'manifest.json').read_text())
def build(manifest=None,include_service=False):
 d=manifest or load()
 return [(c['id'],box(*[x-s/2 for x,s in zip(c['centerMm'],c['sizeMm'])],*c['sizeMm']),c['role']) for c in d['components'] if include_service or c['role']!='service']
def overlap(a,b):
 return [min(x+s/2,y+t/2)-max(x-s/2,y-t/2) for x,s,y,t in zip(a['centerMm'],a['sizeMm'],b['centerMm'],b['sizeMm'])]
def report(d):
 solid=[c for c in d['components'] if c['role']!='service']; conflicts=[]
 for a,b in itertools.combinations(solid,2):
  q=overlap(a,b)
  if min(q)>1e-6:conflicts.append({'a':a['id'],'b':b['id'],'overlapMm':q})
 known=[c for c in d['components'] if c['massKg'] is not None];m=sum(c['massKg']*c['quantity'] for c in known)
 return {'schema':'electrical-package-check/1','manifestSha256':hashlib.sha256((HERE/'manifest.json').read_bytes()).hexdigest(),'solidEnvelopeConflicts':conflicts,'knownComponentMassKg':m,'knownMassCenterMm':[sum(c['massKg']*c['quantity']*c['centerMm'][i] for c in known)/m for i in range(3)],'unknownMassIds':[c['id'] for c in solid if c['massKg'] is None],'fullRobotMassOrCoG':False,'mechanicalAcceptance':False,'limits':['Envelope overlap only, not native mating or fastener verification','Service volumes may overlap intentionally; no reach or thermal qualification','No full chassis/moving mechanism collision screen']}
def svg(d,out):
 colors={'electrical':'#e4a349','mount':'#8298a8','service':'#9ac6dc'}
 s=['<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="900" viewBox="0 0 1100 900">','<rect width="1100" height="900" fill="#f7fafc"/>','<text x="30" y="30" font-size="22">2024 electrical package: original sourced envelopes</text>','<text x="30" y="55" font-size="15">Top view. +X forward (up), +Y left. Dashed regions are service hypotheses, not parts.</text>']
 for c in sorted(d['components'],key=lambda c:c['role']!='service'):
  x,y,z=c['centerMm'];a,b,h=c['sizeMm'];px=430-y-b/2;py=460-x-a/2
  dash=' stroke-dasharray="5 4" fill-opacity=".12"' if c['role']=='service' else ' fill-opacity=".65"'
  s.append(f'<rect x="{px}" y="{py}" width="{b}" height="{a}" fill="{colors[c["role"]]}" stroke="#334155"{dash}/>')
 for i,c in enumerate(d['components']):s.append(f'<text x="760" y="{95+i*22}" font-size="12">{c["id"]}: Z {c["centerMm"][2]:.1f} mm</text>')
 s+=['<text x="30" y="875" font-size="14">Not a complete robot or wiring design. Deck attachments, restraint, harness and cooling remain open.</text>','</svg>'];out.write_text('\n'.join(s))
def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=pathlib.Path,default=HERE/'generated');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True);d=load();parts=build(d);w=STEPControl_Writer()
 for _,shape,_ in parts:
  assert BRepCheck_Analyzer(shape).IsValid();w.Transfer(shape,STEPControl_AsIs)
 dest=a.output/'electrical-envelopes.step';assert w.Write(str(dest))==IFSelect_RetDone
 r=STEPControl_Reader();assert r.ReadFile(str(dest))==IFSelect_RetDone;r.TransferRoots();assert r.NbRootsForTransfer()==len(parts)
 q=report(d);q['exportedRoots']=len(parts);q['reopenedRoots']=r.NbRootsForTransfer();q['stepSha256']=hashlib.sha256(dest.read_bytes()).hexdigest();(a.output/'verification.json').write_text(json.dumps(q,indent=2)+'\n');svg(d,a.output/'layout.svg');print(json.dumps(q))
if __name__=='__main__':main()
