"""Explicit direct/transitive project-source inventory for the Fusion display runner."""
import hashlib,pathlib
LOCAL=('fusion_visual_provenance.py','fusion_visual_display.py','fusion_visual_drive.py','verify_fusion_visual_display.py','native_drive_interactive_v3.py','native_drive_interactive.py','native_drive.py','drivetrain_bridge.py','requirements-lock.txt')
EXTERNAL=('sim/drivetrain/config.json','sim/drivetrain/plant.py','cad/drivetrain/candidate.json')

def inventory(here,source_root):
 here=pathlib.Path(here);source_root=pathlib.Path(source_root)
 return {**{'audit/'+name:hashlib.sha256((here/name).read_bytes()).hexdigest() for name in LOCAL},**{'robot/'+name:hashlib.sha256((source_root/name).read_bytes()).hexdigest() for name in EXTERNAL}}

def verify_inventory(expected,here,source_root):
 actual=inventory(here,source_root)
 if expected!=actual:raise ValueError('runtime dependency inventory missing, stale, or changed')
 return True
