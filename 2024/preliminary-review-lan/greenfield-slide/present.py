from travel import *
field=Path('R:/2024/six-speaker-20261118/field-source/blue-speaker/opening-and-counter.json')
target=(1000*np.array(json.loads(field.read_bytes())['nominalOpeningCornersM'])).tolist()
receipts=json.loads((HERE/'travel.json').read_bytes());parked=json.loads((HERE/'parked.json').read_bytes())
view=HERE/'sliding-pivot.step'
if not view.exists():write_step(view)
model={n:dict(**viewmesh(s),**meta[n],basis='Nominal occupied geometry of the preliminary article') for n,s in G.items()}
allocation={'Chassis':(5,7),'Pivot support':(3.8,6.1),'Pivoting cassette':(1.2,2.3),'Piece support':(.7,1.4),'Cassette drive':(2.1,4.2),'Electrical bay':(3.6,5.9)}
mass=[dict(id=g,kg=kg,members=[n for n in G if meta[n]['group']==g and n!='Battery and terminal case']) for g,kg in allocation.items()]
for g in sorted(set(m['group'] for m in meta.values() if m['group'].startswith('Swerve'))):
    mass.append(dict(id=g,kg=[2.4,3.1],members=[n for n in G if meta[n]['group']==g]))
mass+= [dict(id='Unplaced fasteners, harness and sensors',kg=[2.0,4.2],members=[]),dict(id='Unallocated design reserve',kg=[3.,6.],members=[])]
detail=[dict(name=n,kg=k,members=[v]) for n,k,v in [('battery',[5.5,6.2],'Battery and terminal case'),('one NOTE',[.23,.25],held),('finished bumper scenario',[4.6,6.6],'Blue bumper set')]]
inspection=np.array([m['kg'] for m in mass]).sum(axis=0); running=inspection+np.array([r['kg'] for r in detail]).sum(axis=0)
load_ids={'Drive and steering':[n for n in G if n.startswith('Swerve ')],'Pivot and translating support':['Pivot motor','Slide drive motor'],'Traction belts':[n for n in G if n.startswith('Belt motor ·')],'Pickup roller':['Nose motor'],'Computers and control':['roboRIO flat','radio','PDH']}
power=[]
for stage,amps in [('Aim and spin up',[(8,22),(5,18),(30,70),(0,2),(4,8)]),('Intake',[(16,42),(9,30),(8,26),(4,16),(4,8)]),('Concurrent acceleration and spinup',[(40,90),(12,34),(30,70),(4,16),(4,8)])]:
    for cond,v,r in [('weak',11.8,.027),('nominal',12.6,.018)]:
        lo,hi=np.array(amps).sum(axis=0).tolist();vmin=v-r*hi
        power.append(dict(case=stage+' / '+cond,stage=stage,batteryCondition=cond,openCircuitV=v,resistance=r,requiredMinV=8,loadedBusV=[vmin,v-r*lo],batterySideA=[lo,hi],marginV=vmin-8,status='pass' if vmin>=8 else 'fail',lines=[dict(name=k,amps=a,members=load_ids[k]) for k,a in zip(load_ids,amps)]))
rendercases=[]
for case in cases:
    rc=receipts[case]; frames={}
    for n in G:
        matrices=[poses(case,t/24)[group(n)][:3].ravel().tolist() for t in range(25)]
        if not np.allclose(matrices,identity[:3].ravel()):frames[n]=matrices
    rendercases.append(dict(id=case,stage='service' if case in cases[4:] else 'operating',basis='25 rigid poses. Ring tipping is a prescribed, two-surface geometric path.',gap=0,status='fail' if rc['results'] else 'sampled',results=rc['results'],frames=frames,removed=['Pack lead','Distribution lead',held] if case in cases[4:] else [],positions=[[965.2,5547.36]]*25))
identity_control=corridor()
path=HERE/'motion.json';dump(path,rendercases)
# Extent from actual stock at a known intermediate angle provides an independent failing witness.
witness_shape=G['Entry bearing bracket · left'];horizontal=mat(0,*pivot)@inverse
tip=bounds(place(witness_shape,horizontal))[3];limit=340+304.8
eng=json.loads((HERE/'engineering.json').read_bytes())
notes=[f'Greenfield P2.2 · 92 planning articles. A pair of outboard frames supports the cassette pivot on a translating carriage. The carriage retracts for the middle of the swing. Belts reverse for intake and launch. No game score is established.',
'P1 fixed-pivot control: the entry bracket alone exceeded forward extension by 177.2 mm. On P2 the local bounds of the actual moving cassette fit the rectangle used by the slide schedule. Its continuously bounded front reaches at most robot X634 against X644.8 permitted, leaving 10.8 mm nominal. A stalled slide reproduces the overreach. Stops, sensors and tracking tolerance remain open.',
'P2 first fit: each side beam entered the bumper by 106.72 mm³ and the nose reached 2.90 mm below carpet. Raising the pivot 14 mm cleared both at the 25 checked operating poses. The lower roller makes first contact after 16.13 mm of ideal straight approach. The modeled tip-up can lose powered nose contact by 52.95 mm. Capturing a free NOTE under load is open.',
'Battery withdrawn straight up 370 mm, cables disconnected. At the 125° launch pose it intersects the belt by 8644.65 mm³ and one runner by 365.97 mm³. At 80° the entire swept case clears by 5 mm excluding its support deck. An installed mechanical service prop, handle and wiring are still needed.',
'Shown field outline uses the official speaker corners; the NOTE has a 355.6 mm outer diameter and a 50.8 mm section. The simulation here prescribes the NOTE path. Power lines are battery-side scenarios, and the masses are subsystem planning allowances including separate integration and design reserves.',
f'The outboard A-frames retain {identity_control["minLateralGapMm"]:.1f} mm lateral clearance from the centered NOTE throughout motion. The deliberately displaced NOTE cuts the left A-frame by {identity_control["controlMm3"]:.1f} mm³. Other displayed operating results are 25-pose screens.']
notes.append('Powered contact check: at 18.75% of the tip-and-draw stroke all powered surfaces are at least 37.88 mm from the ring. A trial center belt bridges that pose but intersects the nose roller by 405.14 mm³. Floor capture remains the governing design change.')
data=dict(issue='Greenfield P2.2 · Sliding-pivot cassette',step=sha(view),receipt=sha(HERE/'travel.json'),motion=sha(path),bodies=model,cases=rendercases,palette=dict(structure='#86A9C5',mechanism='#18BFB0',electrical='#B493FF',bumper='#3984F6',**{'reserved-space':'#F68448','game-piece':'#FFC84D'}),mass=mass,details=detail,power=power,inspection=inspection.tolist(),running=running.tolist(),status='Held · floor capture',corners=target,support={},limits={'inspectionKg':56.69904625},**parked,notes=notes,focus='Front extension reserve 10.8 mm. At 19% of tip-and-draw the NOTE loses powered contact by 37.9 mm. Compare the two battery withdrawal poses.',required=[['Four wheels',[n for n in G if n.endswith(' tire')]],['Eight swerve motors',load_ids['Drive and steering']],['Control and distribution',['roboRIO flat','PDH','radio','main breaker','Battery and terminal case']],['Two supported A-frames',['Pivot A-frame · left','Pivot A-frame · right','Pivot torque shaft','Pivot bearing · left','Pivot bearing · right']],['Powered piece handling',load_ids['Traction belts']+load_ids['Pickup roller']]])
# Preserve whole-match scope even while the cassette is under review.
data['focus'] += ' Endgame is open: this layout has no climb allocation or PARK rehearsal.'
data['notes'].append('With 24 s left, Blue 1 compares the next Speaker delivery, PARK and solo ONSTAGE. A shared chain is conditional on a partner and two verified holds. Source: kickoff/strategic-narrative.md, End-of-match decision. This cassette study has no chain interface, frame load path, passive hold, climb mass or current allocation, or timed route to PARK. Inspection and power margins apply only to the scoped design.')
from review_intent import add_review
add_review(data,seat,held)
dump(path, data['cases']); data['motion']=sha(path)
out=HERE.parent/'www'/'data.tmp';dump(out,data);out.replace(out.with_name('data.json'))
print(data['issue'],len(G),'closed',sum(b['closed'] for b in model.values()),data['inspection'],tip-limit,flush=True)

