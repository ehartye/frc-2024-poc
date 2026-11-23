from pathlib import Path
import json,hashlib
q=Path(__file__).parent;sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def data(f):return json.loads((q/f).read_bytes())
files=['support-only-v4.step','roster-v4.json','splits-v4.json','screen.json','screen-v2.json','screen-v3.json','screen-v4.json','loads-v4.json','loaded-cg-v4.json','three-witnesses.json','three-witnesses-lowered.json','shaft-demand.json','shaft-width.json','rack-land-radius.json','gearbox-mount.json','gearbox-mount-corrected.json','gearbox-stack-envelope.json','head-path-v3.json','loom-all-ring.json','loom-all-reservations.json','loom-neighbors.json','support.py','support-v4.py','revised.py','support-v3.py','handoff.md']
paths={f:sha(q/f) for f in files}
v=data('roster-v4.json');s=data('screen-v4.json');g=data('gearbox-stack-envelope.json')
assert len(v)==105 and all(r['solids']==1 and r['valid'] for r in v)
assert list(s)==['neutral','mid-withdraw','capture','home'] and all(len(a)==(6 if pose=='home' else 3) for pose,a in s.items())
assert all(left.endswith('original finger') for a in s.values() for left,right,common in a)
assert not data('loom-neighbors.json')['unmatched']
assert set(['96','192'])==set(data('head-path-v3.json'))
assert all(abs(mm)<1.e-4 for r in g for n,mm in r['padNominal']+r['shaftOutsidePilot']+r['head'])
helpers=Path.home()/'repos/agent-frc/scripts';receipt=dict(status='Select for native integration as a partial design',privateOnly=True,projectScope='2024 Blue 1 over-bumper head only, provisional part local poses. No authority change or promotion.',diagnosticStepSha256=paths['support-only-v4.step'],artifacts=paths,sourceArchives={str(f):sha(f)for f in [q/'M3X20.STEP',q/'bearing.step',q/'switch.step',q.parent/'rack-fit.py',q.parent/'fit-decision.json',q.parent/'wrist-shaft-check.json']},releasedHelpersAtInspection={f:sha(helpers/f) for f in ['mechanical_motion.py','showcase_contacts.py','stock_sections.py']},next='Register actual wrist torque connection and serviceable aluminum stocks in Fusion. Validate sensor trip force/overtravel, shaft form and lash, preload/fasteners, friction and real routing; replay actual constraints. Preserve all old negatives.')
(q/'support-receipt.json').write_text(json.dumps(receipt,indent=2));print(sha(q/'support-receipt.json'))
