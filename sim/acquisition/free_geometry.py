"""Strict coplanar contact adapter. Hard-obstacle gate is mandatory, never inferred."""
import math,pathlib,sys
R=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(R/'cad/assembly'))
import belt_contact_geometry as belt

def make_geometry(config,hard_gate):
 if not callable(hard_gate):raise ValueError('source-bound hard-obstacle gate required')
 g=belt.geometry(config);major=config['note']['majorRadius'];tube=config['note']['tubeRadius'];origin=g['floorOrigin']
 def query(q):
  if len(q)!=2 or any(type(v) not in [int,float] or not math.isfinite(v) for v in q):raise ValueError('finite planar center')
  center=[q[0]*1000,q[1]*1000,origin[2]]
  gate=hard_gate(center,major,tube)
  if type(gate) is not dict or 'status' not in gate:raise ValueError('explicit hard-gate result required')
  if gate['status']!='SUPPORTED':return dict(status=gate['status'],contacts=[])
  local=[center[i]-origin[i] for i in range(3)];contacts=[]
  for motor,side in enumerate([-1,1]):
   witness=belt.coplanar_note_contact(local,major,tube,side,g)
   if not witness['supported']:return dict(status='UNSUPPORTED_BELT_GEOMETRY: '+witness['reason'],contacts=[])
   if witness['penetrationMm']<=0:continue
   contacts.append(dict(motor=motor,penetrationM=witness['penetrationMm']/1000,normal=witness['normalOnNote'][:2],tangent=witness['acquisitionUnitTangent'][:2],leverM=[v/1000 for v in witness['contactLeverFromNoteCenterMm'][:2]]))
  return dict(status='SUPPORTED',contacts=contacts)
 return query
