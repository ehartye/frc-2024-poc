"""Full-NOTE handoff advance against fixed current mechanism; not dynamics."""
import pathlib,json,math,hashlib
import extract_geometry as e
import build_connected_candidate as connected
from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
from OCP.gp import gp_Ax2,gp_Pnt,gp_Dir

def run():
 pins=connected.input_pins();pins['sim/component-launcher/transfer_screen.py']=e.hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest();pins['sim/component-launcher/extract_geometry.py']=e.hashlib.sha256((pathlib.Path(__file__).parent/'extract_geometry.py').read_bytes()).hexdigest()
 c=json.loads((e.R/'cad/assembly/intake_components.json').read_text());angle=math.radians(c['pitchDeg']);parts=[];bad=[]
 assembly,joints=connected.build('handoff')
 if len(assembly)!=711 or len({n for n,s,k in assembly})!=711:raise ValueError('Expected distinct connected assembly711')
 for n,s,k in assembly:
  if k=='service':continue
  if not e.solids(s) or not e.BRepCheck_Analyzer(s).IsValid():bad.append(n);continue
  parts.append((n,s,e.b.bounds(s)))
 rows=[]
 for advance in range(0,481,10):
  center=[c['stageCenterX']+advance*math.cos(angle),0,c['handoffCenterZ']+advance*math.sin(angle)]
  note=BRepPrimAPI_MakeTorus(gp_Ax2(gp_Pnt(*center),gp_Dir(-math.sin(angle),0,math.cos(angle))),c['note']['majorRadius'],c['note']['tubeRadius']).Shape();bounds=e.b.bounds(note);hits=[]
  for n,s,q in parts:
   if min(min(bounds[j+3],q[j+3])-max(bounds[j],q[j]) for j in range(3))<=1e-7:continue
   v=e.common_volume(s,note)
   if v>1e-5:hits.append({'name':n,'overlapMm3':v,'contactGroup':'pinch' if 'side pinch belt' in n else 'launcher' if 'nominal tread annulus' in n else 'unintended'})
  rows.append({'advanceMm':advance,'centerRobotMm':center,'hits':hits,'pinchOverlapMm3':sum(x['overlapMm3'] for x in hits if x['contactGroup']=='pinch'),'launcherOverlapMm3':sum(x['overlapMm3'] for x in hits if x['contactGroup']=='launcher')});print(advance,len(hits),[x['name'] for x in hits if x['contactGroup']=='unintended'],flush=True)
 current=connected.input_pins();current.update({p:e.hashlib.sha256((e.R/p).read_bytes()).hexdigest() for p in pins if p.startswith('sim/')});assert pins==current
 report={'schema':'frc-current-note-transfer-screen/2','assemblyPartCount':len(assembly),'cadRevision':'34c0babb2a93ce4b372030f11daf6e529a98dc30','inputs':pins,'invalidOrNonSolidInputs':bad,'rows':rows,'acceptance':False,'limits':['Fixed cradle handoff pose and guide-aligned NOTE advance only','10mm finite samples, not continuous clearance or dynamic motion','Only designated side belts and launcher tread are intended-contact hypotheses','Original component representations, not exact vendor internals','Upstream traction, preload, power and NOTE attitude evolution uncalibrated','No field or scoring acceptance']}
 out=e.R/'artifacts/connected-transfer';out.mkdir(exist_ok=False,parents=True);(out/'report.json').write_bytes((json.dumps(report,indent=2)+'\n').encode())
if __name__=='__main__':run()
