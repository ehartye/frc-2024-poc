"""Source-bound free-y acquisition experiment; conservative unknown stops, no capture."""
import pathlib,json,hashlib,sys,copy,argparse
R=pathlib.Path(__file__).resolve().parents[2];sys.path[:0]=[str(R/'cad/assembly'),str(pathlib.Path(__file__).parent)]
import free_planar,free_geometry,obstacle_gate
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
OWN=['sim/acquisition/'+x for x in ['free_planar.py','free_geometry.py','obstacle_gate.py','run_free_planar.py','free-planar-config.json','export_obstacles.py']]

def load(obstacle_path):
 import build_connected_candidate as cad
 import audit_floor_entry
 obstacles=json.loads(obstacle_path.read_text());expected=set(cad.input_pins())|{'cad/components/a2-wheel-cartridge.json','cad/assembly/belt_contact_geometry.py','sim/acquisition/export_obstacles.py'}
 if obstacles.get('weldmentCoverage')!=[{'name':'intake candidate original cradle side weldment -1','remainingSolidCount':0,'boxCount':13},{'name':'intake candidate original cradle side weldment 1','remainingSolidCount':0,'boxCount':13}]:raise ValueError('native complete weldment coverage required')
 if len(obstacles['obstacles'])!=744:raise ValueError('complete conservative obstacle inventory')
 if set(obstacles['inputs'])!=expected:raise ValueError('complete obstacle source inventory required')
 reports=[obstacles]
 parity_path=R/'cad/assembly/evidence/belt-contact-geometry.json';parity=json.loads(parity_path.read_text());reports.append(parity)
 if parity.get('schema')!='analytical-belt-native-parity/1' or parity.get('checkedPoints')!=4000 or parity.get('mismatches')!=0:raise ValueError('native belt parity required')
 floor_path=R/'cad/assembly/evidence/floor-entry.json';floor=json.loads(floor_path.read_text());reports.append(floor)
 if floor['inputs']!=audit_floor_entry.pins():raise ValueError('rigid controls stale')
 if [(r['lateralOffsetMm'],r['advanceMm']) for r in floor['rows']]!=[(o,a) for o in [0,-5,5] for a in range(0,481,10)]:raise ValueError('rigid controls inventory')
 if set(parity['inputs'])!=set(cad.input_pins())|{'cad/assembly/belt_contact_geometry.py','cad/assembly/check_belt_contact_geometry.py'}:raise ValueError('parity inventory')
 pins={}
 for report in reports:
  for f,digest in report['inputs'].items():
   path=(R/f).resolve()
   if not path.is_relative_to(R.resolve()) or sha(path)!=digest:raise ValueError('stale source '+f)
   pins[f]=digest
 files=[R/f for f in OWN]+[obstacle_path,parity_path,floor_path]
 pins.update({p.resolve().relative_to(R.resolve()).as_posix():sha(p) for p in files})
 return obstacles,floor,pins

def run(obstacle_path,output):
 if output.exists():raise ValueError('fresh output required')
 obstacles,floor,pins=load(obstacle_path);cfg=json.loads((R/'sim/acquisition/free-planar-config.json').read_text());cad=json.loads((R/'cad/assembly/intake_components.json').read_text());note=cad['note']
 gate=obstacle_gate.make_gate(obstacles,note['majorRadius'],note['tubeRadius'],cad['floorCenterZ']);geometry=free_geometry.make_geometry(cad,gate)
 experiments=[('center',0.,{},cfg['duty']),('left',-.005,{},cfg['duty']),('right',.005,{},cfg['duty']),('left-half',-.005,{'stepS':cfg['model']['stepS']/2},cfg['duty']),('right-half',.005,{'stepS':cfg['model']['stepS']/2},cfg['duty']),('zero-belt-right',.005,{},0.),('soft-right',.005,{'normalStiffnessNpm':1000.},cfg['duty']),('stiff-right',.005,{'normalStiffnessNpm':4000.},cfg['duty']),('low-ground-right',.005,{'groundMu':.1},cfg['duty']),('aabb-control-right',.005,{},cfg['duty']),('stiffer-right',.005,{'normalStiffnessNpm':8000.},cfg['duty']),('stiffer-right-half',.005,{'normalStiffnessNpm':8000.,'stepS':cfg['model']['stepS']/2},cfg['duty']),('small-offset-right',.001,{},cfg['duty']),('mid-offset-right',.0025,{},cfg['duty'])]
 cases=[]
 for name,y,changes,duty in experiments:
  selected_geometry=free_geometry.make_geometry(cad,obstacle_gate.make_gate(obstacles,note['majorRadius'],note['tubeRadius'],cad['floorCenterZ'],refine_cylinders=False)) if name=='aabb-control-right' else geometry
  c=dict(cfg['model'],**changes);case=free_planar.simulate(c,selected_geometry,dict(p=[cfg['initialRobotRelativeX'],y]),base_velocity=cfg['approachMps'],duties=(duty,duty));case.update(name=name,config=c,initialOffsetM=y);cases.append(case)
  print(name,case['outcome'],case['final']['relativeRobotXY'],case['maxAbsEnergyResidualJ'],flush=True)
 if any(sha(R/f)!=v for f,v in pins.items()):raise ValueError('sources changed')
 controls=[{'offsetMm':o,'hardHitRows':[r['advanceMm'] for r in floor['rows'] if r['lateralOffsetMm']==o and any(h['contactGroup']!='pinch' for h in r['hits'])]} for o in [-5,0,5]]
 result=dict(schema='free-planar-acquisition-experiment/1',inputs=pins,cases=cases,rigidControls=controls,activeConservativeObstacleBoxes=gate.candidate_count,acceptance=False,limits=cfg['assumptions']+obstacles['limits']);output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');return result
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--obstacles',required=True,type=pathlib.Path);p.add_argument('--output',required=True,type=pathlib.Path);a=p.parse_args();run(a.obstacles,a.output)
