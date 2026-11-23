"""Plan the one candidate firing station against the current guided robot and the registered field."""
from pathlib import Path
import json
p=Path(__file__).resolve().parent
source=(p.parent/'six-piece-02/route.py').read_text()
section="""live=read(p.parent.parent/'www/data.json');fieldRev=live['concepts']['field'];guide=live['concepts']['guided']
current=next(c for c in fieldRev['cases'] if c['key']=='field-r08')
allBodies={**live['bodies'],**fieldRev['add'],**guide['add']}
service=set(guide['fixtureOnlyNames']);omitted=set(current['removed'])|set(fieldRev['remove'])|service|{'Chambered NOTE'}
design['bodies']={n:v for n,v in allBodies.items() if n not in omitted}
cs['rig']={n:current['rig'].get(n,'arm' if n in guide['add'] else 'frame') for n in design['bodies']}
assert all(role in ('arm','slide','flap','frame') for role in cs['rig'].values())
design['step']=guide['step']
"""
assert source.count('vs={role:')==1
source=source.replace('vs={role:',section+'\nvs={role:').replace('station=np.array([1.53,5.55])','station=np.array([1.39,5.871])')
exec(compile(source,'route around current robot and marked NOTES','exec'))

