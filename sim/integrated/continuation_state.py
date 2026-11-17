"""Exact same-model checkpoint continuity; not a capture or phase-transition adapter."""
import copy,math
from coupled_acquisition import CoupledAcquisition

def _finite(value):
 if isinstance(value,dict):return all(_finite(v) for v in value.values())
 if isinstance(value,list):return all(_finite(v) for v in value)
 return type(value) in (int,float) and math.isfinite(value)

def checkpoint(plant,model_identity):
 if not model_identity or any(not isinstance(k,str) or not isinstance(v,str) or len(v)!=64 or any(ch not in '0123456789abcdef' for ch in v) for k,v in model_identity.items()):raise ValueError('explicit source identity required')
 if plant.initial_energy is None:raise ValueError('no initialized physical state')
 return copy.deepcopy(dict(schema='frc-coupled-checkpoint/1',modelIdentity=model_identity,driveConfig=plant.drive.c,noteConfig=plant.c,timeS=plant.t,drive=plant.drive.state,note=plant.note,energy=plant.energy,initialEnergyJ=plant.initial_energy))

def restore(saved,geometry,expected_identity,drive_config,note_config):
 s=copy.deepcopy(saved)
 if s.get('schema')!='frc-coupled-checkpoint/1' or not expected_identity or s.get('modelIdentity')!=expected_identity:raise ValueError('same model identity required')
 if s.get('driveConfig')!=drive_config or s.get('noteConfig')!=note_config:raise ValueError('same physical configuration required')
 if not all(_finite(s[k]) for k in ['timeS','drive','note','energy','initialEnergyJ']):raise ValueError('finite complete physical state required')
 if s['timeS']<0 or s['drive'].get('t')!=s['timeS']:raise ValueError('one shared clock required')
 if set(s['drive'])!={'t','x','y','heading','vx','vy','yaw_rate','wheel','wheel_angle'} or set(s['note'])!={'p','v','yaw','spin','w'}:raise ValueError('complete coordinate and rotor state required')
 if any(len(s['note'][k])!=2 for k in ['p','v','w']) or any(len(s['drive'][k])!=2 for k in ['wheel','wheel_angle']):raise ValueError('state dimension mismatch')
 p=CoupledAcquisition(s['driveConfig'],s['noteConfig'],geometry,s['note'])
 if set(s['energy'])!=set(p.energy):raise ValueError('complete cumulative energy ledger required')
 p.drive.state=s['drive'];p.note=s['note'];p.energy=s['energy'];p.t=s['timeS'];p.initial_energy=s['initialEnergyJ']
 return p
