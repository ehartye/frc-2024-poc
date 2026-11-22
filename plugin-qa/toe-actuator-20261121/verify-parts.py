from pathlib import Path
import sys,zipfile,hashlib
sys.path.insert(0,str(Path.home()/'repos/agent-frc/scripts'));from mechanical_motion import bounds;from showcase_contacts import load_solids
from OCP.BRepCheck import BRepCheck_Analyzer
q=Path(__file__).parent
for n in ['REV-41-1503','REV-41-1332']:
 a,b=load_solids(q/(n+'.step'))
 for k,s in a.items():print(n,k,bounds(s),BRepCheck_Analyzer(s).IsValid())
z=zipfile.ZipFile(q/'REV-41-1600-ALL-PARTS.zip')
for num in [1602,1607,1608,1604]:
 p=next(p for p in z.namelist() if str(num)+'.STEP' in p);b=z.read(p);print(p,hashlib.sha256(b).hexdigest());(q/Path(p).name).write_bytes(b)
