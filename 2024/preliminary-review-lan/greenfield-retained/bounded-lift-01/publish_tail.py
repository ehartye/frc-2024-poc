"""Add the fine field continuation to the same viewer without replacing the original replay."""
from pathlib import Path
import json,hashlib
here=Path(__file__).resolve().parent
def obj(p):return json.loads(p.read_bytes())
def h(p):return hashlib.sha256(p.read_bytes()).hexdigest()
case=next(x for x in obj(here/'tail-timing.json') if x['resolution']==96 and x['friction']=='.25')
receipt=obj(here/'run-dev-96.json');receipt.update(elapsedS=case['elapsed'],passMission=case['within120'])
last=0.
for part,ext in zip(receipt['parts'],case['parts']):
    assert ext['id']==part['id']
    shift=last-part['depart'];part['shotStart']+=shift;part['depart']=last;part['counterClock']=ext['elapsed']
    part['resetParallelEligible']=ext['overlap'];last=ext['nextDeparture']
    if ext['receipt']:
        sh=here/('split-tail_.25_0-dev-'+part['id']+'-96-6.88.json')
        assert h(sh)==obj(here/('tail-receipt_.25_0-dev-'+part['id']+'-96.json'))['extended']
        part['shot']=h(sh)
(here/'tail-run-dev-96.json').write_text(json.dumps(receipt,indent=2))
s=(here/'publish.py').read_text(encoding='utf8').replace('loaded-cycle-48','observed-reset-96').replace('run-dev-48','tail-run-dev-96').replace("k+'-48'","k+'-96'").replace("'dev-middle-48.json'","'dev-middle-96.json'").replace("sf=p/('split-outerlane-'+tag+'-6.88.json')","sf=p/(('split-tail_.25_0-' if extById[k]['receipt'] else 'split-outerlane-')+tag+'-6.88.json')").replace('Six NOTES · bounded loaded lift','Six NOTES · full post-shot observation').replace('at 0.5 ms','at 0.25 ms').replace('viewer-receipt.json','tail-viewer-receipt.json')
s=s.replace("g['focus']='Profile A and a bounded 4.8 s lift scored six in 118.46 s on the nominal coarse model. Inspect the match clock and field count here. Frozen starts, actual motor installation, sustained current and closed-loop tracking remain open.'","g['focus']='Full field observation puts the sixth nominal score at 115.46 s on the fine model and 115.63 s on the coarse model. This replay initializes belts at each pickup. The continuous-state mission and the 19-of-20 repeatability test are separate.'")
s=s.replace("g['defaultCase']=c['key']","g['defaultCase']='loaded-cycle-48'")
extById={row['id']:row for row in case['parts']}
exec(compile(s,'fine observed reset replay','exec'))

