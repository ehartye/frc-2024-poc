"""Bind reviewed nominal geometry and the additional installed component mass estimates to the live view."""
from pathlib import Path
import hashlib,json
p=Path(__file__).parent
file=p/'www/data.json';d=json.loads(file.read_bytes())
receipt=p/'nominal-checked.json'
if receipt.exists() and json.loads(receipt.read_bytes())['stepSha256']==d['nominalSha']:
    r=json.loads(receipt.read_bytes());assert r['stepSha256']==d['nominalSha']
    d['nominalCases']={a['case']:{k:a[k] for k in ('status','pairs','counts','requiredContacts')} for a in r['cases']}
    d['nominalReceipt']=hashlib.sha256(receipt.read_bytes()).hexdigest()
d['palette'].update(structure='#86A9C5',mechanism='#18BFB0',electrical='#B493FF',bumper='#3984F6',**{'reserved-space':'#F68448','game-piece':'#FFC84D'})
tmp=file.with_suffix('.tmp');tmp.write_text(json.dumps(d,separators=(',',':')),encoding='utf8');tmp.replace(file)

