from pathlib import Path
import math,json,hashlib,csv
p=Path(__file__).resolve().parent;root=p.parent
def read(f):return json.loads(f.read_bytes())
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
season=Path('C:/Users/ehart/repos/frc-2024-poc')
narr=season/'kickoff/strategic-narrative.md';roster=season/'sim/pressure/evidence/speaker-repeatability-starts.csv'
d=read(p/'review-data.json');assert d['narrativeReview']['source']['sha256']==sha(narr)
cases={n:read(p/(n+'.json')) for n in ['controlled-front','controlled-center','controlled-rear','controlled-short','controlled-open','stiff-front','early']}
for n in ['controlled-front','controlled-center','controlled-rear','controlled-short']:assert cases[n]['passed']
for n in ['stiff-front','controlled-open','early']:assert not cases[n]['passed']
tr=read(p/'travel-detail.json');assert all(not row['unresolved'] for row in tr['rotation']) and all(not row['unchecked'] for row in tr['service'])
hp=root/'pickup-height-control-01'
pickup={n:read(hp/(n+'.json')) for n in ('nom','lowL','lowR','highL','highR','weak','dead')}
assert all(x['passScreen']==(n!='dead') for n,x in pickup.items())
tmax=max(cases[n]['peakCatchNm'] for n in cases if n.startswith('controlled') and cases[n]['passed'])
r=55*math.sqrt(1-(6/55)**2)
def signed_load(tq):
    force=tq*1000/r
    return dict(torqueNm=tq,armMm=r,dogTangentialN=force,pinAvgShearMPa=force/(math.pi*6**2),cantileverBeamMPa=32*force*6.8/(math.pi*12**3),sectorProjectedBearingMPa=force/(9.6*12))
out=dict(assembly=d['step'],requirements=dict(narrative=sha(narr),rosterFile=roster.name,rosterHash=sha(roster),count=len(list(csv.DictReader(roster.open()))),campaignCredit=0),
        boundaries='Phase layout. No supplier, thread, impact, fatigue or fabrication signoff.',
        CAD=dict(count=len(d['bodies']),rotationPairs=read(p/'travel-broad.json')['n'],servicePaths=len(tr['service']),rejected=['layout 01 sector overlaps service disk','layout 02 receiver rubs rotating service disk','rapid unload overloads rigid catch']),
        load=signed_load(tmax),overloadScreen=signed_load(350),
        catch={n:dict(sha256=sha(p/(n+'.json')),passed=x['passed'],proofTime=x['confirmAt'],cut=x['cutTime'],reserveSec=x['confirmMarginS'],floorN=x['maxFloorN'],driftDeg=x['measuredDriftDeg'],compressionMm=x.get('maxLimitCompressionMm')) for n,x in cases.items()},
        pickup={n:dict(sha256=sha(hp/(n+'.json')),tipMm=x['minimumBladeHeightMm'],motorLimitA=x['phaseCurrentLimitA'],seated=x['seated'],seatResidence=x['seatResidenceS']) for n,x in pickup.items()},
        next=['Locate carpet reference, sensor or follower, its available travel, power, service and response; the existing 5 mm model assumes the setting is attained.',
        'Bind floor approach, driven free-piece carry, power arbitration, physical score and return to one field clock with independent floor positions. Evaluate six distinct IDs in 120 s at 19/20 frozen starts.',
        'Rehearse acquisition of the chain on approach, full truss clearance, catch sensing and PARK fallback on the selected layout. Run complete preliminary phase checks before handoff.'])
(p/'phase-evidence.json').write_text(json.dumps(out,indent=2))
print(out['CAD'],out['load'],out['requirements'])

