"""Publish only the active model and its matching receipts. Atomic JSON replacement drives hot reload."""
import hashlib,json
from pathlib import Path
p=Path(__file__).resolve().parent
if (p/'greenfield-active').exists():
    raise SystemExit('Greenfield review owns the LAN presentation')
src=p/'build-cache'/'data.json';data=json.loads(src.read_bytes())
step=data.pop('nominalSha')
assert hashlib.file_digest((p/'supported-layout.step').open('rb'),'sha256').hexdigest()==step
out=p/'supported-checked.json'
receipt=None
if out.exists():
    r=json.loads(out.read_bytes())
    if r['stepSha256']==step:receipt=r
data['step']=step;data['motion']=hashlib.sha256((p/'supported-paths.json').read_bytes()).hexdigest()
data['receipt']=hashlib.sha256(out.read_bytes()).hexdigest() if receipt else None
data['palette'].update(structure='#86A9C5',mechanism='#18BFB0',electrical='#B493FF',bumper='#3984F6',**{'reserved-space':'#F68448','game-piece':'#FFC84D'})
for n,b in data['bodies'].items():
    if 'nominal' in b:b.update(b.pop('nominal'))
    b.pop('added',None)
data['static']=data.pop('refinedParked');data['narrow']=data.pop('refinedNarrow')
for key in ('rings','originalCount','control','open'):data.pop(key,None)
obs={c['case']:c for c in receipt['cases']} if receipt else {}
for case in data['cases']:
    record=obs.get(case['id'],{})
    case['results']=record.get('pairs',[])
    case['status']=record.get('status','pending')
if receipt:
    ctrl=obs['displaced-front-cover']
    data['control']=next(row for row in ctrl['pairs'] if set(row['pair'])=={'Complete blue bumper envelope','Floor NOTE'})
data['support']={
    'Motor station 0':'Fixed left. Carrier, yoke, bearings and coupling are present in the detailed assembly.',
    'Motor station 1':'Fixed right. The proposed opposite-hand carrier has a measured 1,612 mm² web seat. Installed verification remains open.',
    'Motor station 2':'Moving upper. Preliminary frame, guide rails, four blocks, moving receiver, shaft, bearing carrier, spring envelope and opposed stops are shown. The independent 0–6 mm stroke has its own case. Loaded reaction and withdrawal remain open.',
    'Motor station 3':'Moving upper. Opposite-hand packaging proposal with guide rails, receiver, shaft, bearings, spring and stops. The independent 0–6 mm stroke has its own case. Right-hand attachment, stops and battery service require confirmation.'}
for i in range(4):data['support']['Wheel station '+str(i)]=data['support']['Motor station '+str(i)]
data['limits']={'inspectionKg':56.69904625}
temp=p/'www'/'data.tmp';temp.write_text(json.dumps(data,separators=(',',':'),allow_nan=False),encoding='utf8');temp.replace(p/'www'/'data.json')
print(len(data['bodies']),step[:8],data['preloadFit'],len(obs),flush=True)
