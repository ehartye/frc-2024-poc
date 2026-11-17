"""Pressure-only continuation state. No legacy model conversion or energy-reference shift."""
import copy,hashlib,json,math
import compare_entry as factory
R=factory.R
SCHEMA='pressure-acquisition-checkpoint/1'
def digest(v):return hashlib.sha256(json.dumps(v,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def identity(p,pins):
 return dict(inputs=dict(pins,**{f:factory.r.sha(R/f) for f in ['sim/pressure/checkpoint.py','sim/pressure/compare_entry.py']}),driveConfig=p.drive.c,noteConfig=p.c,cad=p.cad,pressure=dict(resolution=list(p.resolution),kArea=p.k_area,uPositiveM=0.,materialPhase='uniform-midpoint-fixed-v1'))
def shape(value,template):
 if isinstance(template,dict):
  if type(value)!=dict or value.keys()!=template.keys():raise ValueError('state keys')
  for k in template:shape(value[k],template[k])
 elif isinstance(template,list):
  if type(value)!=list or len(value)!=len(template):raise ValueError('state vector shape')
  for a,b in zip(value,template):shape(a,b)
 else:
  if type(value) not in (int,float) or not math.isfinite(value):raise ValueError('finite numeric state required')
def diagnostics(p):
 potential=p.prepared(p.drive.state,p.note,{})['potentialJ'];kinetic=p.drive.kinetic()+p.note_kinetic(p.note)
 return dict(potentialJ=potential,kineticJ=kinetic,residualJ=p.initial_energy+p.energy['sourceWork']-sum(v for k,v in p.energy.items() if k!='sourceWork')-kinetic-potential)
def export(p):
 fresh,pins=factory.setup()
 if identity(p,pins)!=identity(fresh,pins):raise ValueError('unsupported model identity')
 state=dict(chassis=copy.deepcopy(p.drive.state),note=copy.deepcopy(p.note),ledger=copy.deepcopy(p.energy),timeS=p.t,initialEnergyJ=p.initial_energy)
 shape(state,dict(chassis=fresh.drive.state,note=fresh.note,ledger=fresh.energy,timeS=0.,initialEnergyJ=0.))
 if p.t<0 or p.initial_energy!=0 or any(v<0 for v in p.energy.values()):raise ValueError('fresh-lineage energy/time')
 payload=dict(schema=SCHEMA,identity=identity(p,pins),state=state,diagnostics=diagnostics(p))
 return dict(payload,digest=digest(payload))
def restore(record):
 if type(record)!=dict or set(record)!=set(['schema','identity','state','diagnostics','digest']) or record['schema']!=SCHEMA:raise ValueError('pressure checkpoint schema required')
 payload={k:v for k,v in record.items() if k!='digest'}
 try:valid=digest(payload)==record['digest']
 except (ValueError,TypeError):raise ValueError('invalid checkpoint encoding')
 if not valid:raise ValueError('checkpoint digest mismatch')
 p,pins=factory.setup()
 if record['identity']!=identity(p,pins):raise ValueError('source/config/grid/phase identity mismatch')
 s=record['state'];shape(s,dict(chassis=p.drive.state,note=p.note,ledger=p.energy,timeS=0.,initialEnergyJ=0.))
 if s['timeS']<0 or s['initialEnergyJ']!=0 or any(v<0 for v in s['ledger'].values()):raise ValueError('fresh-lineage energy/time')
 p.drive.state=copy.deepcopy(s['chassis']);p.note=copy.deepcopy(s['note']);p.energy=copy.deepcopy(s['ledger']);p.t=s['timeS'];p.initial_energy=s['initialEnergyJ']
 if record['diagnostics']!=diagnostics(p):raise ValueError('potential/kinetic/ledger inconsistency')
 return p
