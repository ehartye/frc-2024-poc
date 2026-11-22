"""Publish the fixed-coordinate evidence and preserve the current repeatability gate on the same viewer."""
from pathlib import Path
import json,hashlib
p=Path(__file__).resolve().parent;cr=p.parent/'counted-reset-power-02';web=p.parent.parent/'www'
read=lambda f:json.loads(f.read_bytes())
df=web/'data.json';d=read(df);g=d['concepts']['guided']
runs=[read(cr/f'run-R{i:02d}-96.json') for i in range(1,6)]
assert all(v['terminal'] for v in runs) and sum(v['passMission'] for v in runs)==3
short='Three of the first five frozen starts met the six individual-shot criteria. Two misses preclude 19 of 20 on this design. Both missed NOTES cleared the speaker opening. Shared field-counter reset is under test.'
late=read(p/'extended-R03-center-low.json')
returned=read(p/'extended-R02-inner.json')
pair=read(p/'pair-coupled.json')
assert late['originalShotSha']==runs[2]['parts'][-1]['shot'] and late['identicalSampledPoseError']<1.e-8 and 0<late['counterMissionClock']<120
assert returned['originalShotSha']==runs[1]['parts'][-1]['shot'] and returned['counterS'] is None and returned['finalQ'][2]<.026
assert pair['sharedClearRayBothS']<pair['distinctSecondEventS']
short=f"Four of the first five starts produced six individual shot events within 120 s when the field observer kept running. One sixth NOTE reached both beams at {late['counterMissionClock']:.3f} s. In the remaining start, the third NOTE returned to the carpet. Fifteen starts and the complete shared-counter check remain open."
g['focus']=short
for n in [48,96]:
    audit=read(p/f'view-contact-{n}.json')
    c=next(c for c in g['cases'] if c['key']==audit['case'])
    categories=[(['NOTE','Robot frame'],'sampled','Displayed NOTE positions use the recorded chassis-relative trajectory. The drive advance is accounted for once. The transfer from carry to shot matches the free NOTE position to within 0.001 mm on all six deliveries.'),(['NOTE','Cassette contact surfaces'],'open','The NOTE is drawn at full free size during contact. The collision sample finds up to %.2f mm into the modeled intake belt. Rigid guide, keeper and lip contacts still require contact and compliance review.'%audit['maxima'][0]['penetrationMm'])]
    rm=[v[0] for v in categories]+[['Shared field counter','Repeat deliveries']]
    c['results']=[r for r in c['results'] if r['pair'] not in rm]
    reason='The second NOTE displaced the first in the shared field simulation. Both beam pairs cleared and then registered a distinct second event. The remaining four shared events are open.' if n==48 else 'Two NOTES needed 10.032 s and 7.824 s to clear. Both cleared before the following arrival in isolation. Shared collection occupancy is open.'
    categories.append((['Shared field counter','Repeat deliveries'],'open',reason))
    for pair,status,why in categories:c['results'].append(dict(pair=pair,status=status,witness=dict(reason=why)))
    c['measure']=[v.replace('/6 scored','/6 individual events') for v in c['measure']]
    c['basis']='The free NOTE and displayed cassette use the simulated carry coordinates. Each of the six deliveries crossed the full opening and triggered both beams in its own field replay. Repeated scoring on a shared counter remains open. The torus depicts the unloaded foam shape. Pickup and tread contact use provisional compliance.'
g['status']='Preliminary design held · scoring repeatability'
temp=df.with_suffix('.clip-reviewed');temp.write_text(json.dumps(d,separators=(',',':')),encoding='utf8');temp.replace(df)
js=web/'review.js';s=js.read_text(encoding='utf8').replace('Preliminary design · frozen-start campaign in progress','Preliminary design held · scoring repeatability').replace('Six nominal scores at both simulation resolutions. Acceptance requires 19 of 20 fixed, perturbed starts.','Six individual events at both nominal resolutions. Three of five frozen starts passed the individual-shot criteria. The current design falls short of 19 of 20.')
s=s.replace('Three of five frozen starts passed the individual-shot criteria. The current design falls short of 19 of 20.', 'Four of five starts produced six individual events with the mission-length observer. Fifteen starts remain untested.')
js.write_text(s,encoding='utf8')
report=dict(freeze=hashlib.sha256((cr/'freeze.json').read_bytes()).hexdigest(),completed=[dict(start=r['name'],count=r['count'],passed=r['passMission'],elapsedS=r['elapsedS']) for r in runs],unrun=15,maxPossiblePass=18,liveDefault=g['defaultCase'])
report['missionClockAssessment']=dict(firstFiveIndividualPasses=4,ejectedNote=returned,lateSixth=late,unrun=15,maxPossibleIndividualPasses=19,sharedCounterSixQualified=False)
(p/'design-decision.json').write_text(json.dumps(report,indent=2))

