"""Offset the crossbar by 6 mm to clear the motor as the gate reaches full stroke."""
from pathlib import Path
p=Path(__file__).resolve().parent
src=(p.parent/'keeper-drive-01/fit.py').read_text(encoding='utf8')
src=src.replace('bridge=box(201,-104.88,273,5,209.76,5)','bridge=box(207,-104.88,273,5,209.76,5)').replace('link=box(198,-4,273,8,8,5)','link=box(198,-4,273,14,8,5)')
needle="bridge=fuse(bridge,local[side+' gate'])";assert src.count(needle)==1
src=src.replace(needle,"bridge=fuse(fuse(bridge,box(201,96.88 if side=='left' else -104.88,272,11,8,6)),local[side+' gate'])")
exec(compile(src,'clear motor crossbar','exec'))
