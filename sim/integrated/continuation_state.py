"""Exact same-model checkpoint continuity; not a capture or phase-transition adapter."""
import copy,math
from coupled_acquisition import CoupledAcquisition

def _number(value):return type(value) in (int,float) and math.isfinite(value)
def _pair(value):return type(value) is list and len(value)==2 and all(_number(x) for x in value)

def checkpoint(plant,model_identity):
 if not model_identity or any(not isinstance(k,str) or not isinstance(v,str) or len(v)!=64 or any(ch not in '0123456789abcdef' for ch in v) for k,v in model_identity.items()):raise ValueError('explicit source identity required')
 if plant.initial_energy is None:raise ValueError('no initialized physical state')
 return copy.deepcopy(dict(schema='frc-coupled-checkpoint/1',modelIdentity=model_identity,driveConfig=plant.drive.c,noteConfig=plant.c,timeS=plant.t,drive=plant.drive.state,note=plant.note,energy=plant.energy,initialEnergyJ=plant.initial_energy))

def restore(saved,geometry,expected_identity,drive_config,note_config):
 if type(saved) is not dict:raise ValueError('checkpoint object required')
 s=copy.deepcopy(saved)
 required={'schema','modelIdentity','driveConfig','noteConfig','timeS','drive','note','energy','initialEnergyJ'}
 if set(s)!=required:raise ValueError('complete checkpoint fields required')
 if s['schema']!='frc-coupled-checkpoint/1' or not expected_identity or s['modelIdentity']!=expected_identity:raise ValueError('same model identity required')
 if s['driveConfig']!=drive_config or s['noteConfig']!=note_config:raise ValueError('same physical configuration required')
 if type(s['drive']) is not dict or type(s['note']) is not dict or type(s['energy']) is not dict:raise ValueError('physical state objects required')
 if set(s['drive'])!={'t','x','y','heading','vx','vy','yaw_rate','wheel','wheel_angle'} or set(s['note'])!={'p','v','yaw','spin','w'}:raise ValueError('complete coordinate and rotor state required')
 if not all(_number(s[k]) for k in ['timeS','initialEnergyJ']) or not all(_number(s['drive'][k]) for k in ['t','x','y','heading','vx','vy','yaw_rate']) or not all(_number(s['note'][k]) for k in ['yaw','spin']) or not all(_number(v) for v in s['energy'].values()):raise ValueError('finite numeric scalars required')
 if not all(_pair(s['note'][k]) for k in ['p','v','w']) or not all(_pair(s['drive'][k]) for k in ['wheel','wheel_angle']):raise ValueError('exact finite numeric pairs required')
 if s['timeS']<0 or s['drive']['t']!=s['timeS']:raise ValueError('one shared clock required')
 p=CoupledAcquisition(s['driveConfig'],s['noteConfig'],geometry,s['note'])
 if set(s['energy'])!=set(p.energy):raise ValueError('complete cumulative energy ledger required')
 p.drive.state=s['drive'];p.note=s['note'];p.energy=s['energy'];p.t=s['timeS'];p.initial_energy=s['initialEnergyJ']
 return p
