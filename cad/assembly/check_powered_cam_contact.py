"""Reproducible standalone cam-port cases. No integration or servo calibration."""
import pathlib,json,hashlib,math
from powered_cam_contact import evaluate,command_target
ROOT=pathlib.Path(__file__).resolve().parents[2]
def run():
 cases=[('separated',[0,.02,math.pi/2,0]),('touching_approach',[0,.02,0,0]),('compressed_closing',[.0009,.02,.7,1.3]),('compressed_unloading',[.0009,-.02,.7,1.3]),('open_dead_center',[.0003,.05,0,0]),('closed_touching',[.0015,0,math.pi,0])]
 rows=[]
 for name,state in cases:
  result=evaluate(*state,stiffness_n_m=20000,damping_n_s_m=.5)
  rows.append({'case':name,'state_u_udot_theta_thetadot':state,'unchangedMechanicalInput':list(state),'stiffnessHypothesisNPerM':20000,'dampingHypothesisNsPerM':.5,'result':result})
 contract=json.loads((ROOT/'cad/assembly/powered-cam-interface.json').read_text());pins=dict(contract['geometrySourcePins'])
 for n in ['powered_cam_contact.py','powered-cam-interface.json','check_powered_cam_contact.py']:
  p=ROOT/'cad/assembly'/n;pins['cad/assembly/'+n]=hashlib.sha256(p.read_bytes()).hexdigest()
 for path,sha in pins.items():
  if hashlib.sha256((ROOT/path).read_bytes()).hexdigest()!=sha:raise ValueError('source changed')
 return {'inputs':pins,'rows':rows,'midpointCommand':command_target(1500,verified_default_270_mode=True,center_cam_angle_rad=math.pi/2,direction=1),'accepted':False,'limits':['Standalone contact port only; no state integration, capture or servo calibration','Stiffness/damping values are explicit example hypotheses','Mechanical contact remains a separate passive port after electrical faults','No inferred torque-current-speed curve, holding thermal rating or output inertia','No generalized force duplicated onto chassis; caller maps the internal pair exactly once']}
if __name__=='__main__':
 r=run();(ROOT/'cad/assembly/evidence/powered-cam-contact.json').write_text(json.dumps(r,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');print([(x['case'],x['result']['normal_force_n'],x['result']['power_identity_residual_w']) for x in r['rows']])
