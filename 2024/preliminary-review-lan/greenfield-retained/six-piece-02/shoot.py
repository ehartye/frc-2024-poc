"""Powered return-station test initialized from the exact carried NOTE pose and velocity."""
from pathlib import Path
import json,sys,hashlib,numpy as np
p=Path(__file__).resolve().parent;method=p.parent/'follower-01'
key,attempt,n=sys.argv[1:4];name=attempt+'-'+key+'-'+n
src=p/(name+'.json');a=json.loads(src.read_bytes())
assert a['id']==key and a['attempt']==attempt and a['held'] and a['seated']
script=(method/'fire.py').read_text();script=script[script.index("old=(p.parent/"):]
script=script.replace("extra=", "old=old.replace('1.43','1.53')\nextra=")
script=script.replace('incoming[\'approachM\']','incoming[\'approachM\']') # End chassis translation is zero in the carried body frame.
script=script.replace("'fire'+n","'scored-'+name").replace('previous.read_bytes()','src.read_bytes()')
assert abs(np.rad2deg(a['endFlapRad'])-90)<1
exec(compile(script,'same-field-NOTE station','exec'))
