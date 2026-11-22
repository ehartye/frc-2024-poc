from travel import *
from component_intent import explain
from pathlib import Path
import json
source=HERE.parent/'greenfield-slide'/'sliding-pivot.step'
baseline=json.loads((HERE.parent/'www'/'data.json').read_bytes())
# Data from the prior view is used only for unchanged display palette and field aperture.
outstep=HERE/'retained-cassette.step'
rec=json.loads((HERE/'checks.json').read_bytes())
j=lambda file:json.loads((HERE/file).read_bytes())
assert rec['sourceStep']==sha(outstep)==j('stop-support.json')['step']
model={}
for n,s in G.items():
    start=intent[n] if n in intent else explain(n)
    model[n]=dict(**viewmesh(s),**meta[n],**start,basis='Closed preliminary package solid on the current chassis datum.')
assert all(x['closed'] for x in model.values())
results=j('travel.json');render=[]
for case in [*cases,*extra]:
    frames={}
    for name in G:
        data=[poses(case,t/24)[group(name)][:3].ravel().tolist() for t in range(25)]
        if not np.allclose(data,identity[:3].ravel()):frames[name]=data
    if case==reach:
        # Reel extends from its fixed tangent point to the underside of the crosshead.
        data=[]
        for t in range(25):
            m=identity.copy();m[2,2]=1+(142*t/24)/326;m[2,3]=235*(1-m[2,2]);data.append(m[:3].ravel().tolist())
        frames['Climb cable']=data
    record=results[case];notes='25 positions, occupied-volume checks. Free capture dynamics are shown in a separate case.'
    if case==release:notes='Pin opens first; a geometric exit path is tested. Exit velocity and scoring remain open.'
    render.append(dict(id=case,stage='service' if case in cases[4:] else 'operating',basis=notes,gap=0,status='fail' if record['results'] else 'sampled',results=record['results'],frames=frames,removed=[held,'Distribution lead','Pack lead'] if case in cases[4:] else [],positions=[[965.2,5547.36]]*25))
# Replay the free ring in the bare-center-belt trial from solver quaternions.
# Omit optional pressure rollers from this measured test as they are not in the CAD.
free=j('pickup-64-0.0005-1-0-0-0.75.json')
def quat(q):
    w,x,y,z=q;return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
base=poses(cases[2],0);frames={n:[base[group(n)][:3].ravel().tolist()]*25 for n in G if not np.allclose(base[group(n)],identity)}
path=[]
for t in np.linspace(0,len(free['trajectory'])-1,25):
    row=free['trajectory'][round(t)];m=np.eye(4);m[:3,:3]=quat(row[4:8]);m[:3,3]=1000*np.array(row[1:4]);path.append((m@invseat)[:3].ravel().tolist())
frames[held]=path
render.append(dict(id='Loose NOTE · powered center belt',stage='pickup dynamics',basis='2 s of free-body motion, finite floor ring and compliant contacts. Prescribed chassis advance and drive speeds.',status='fail',gap=0,frames=frames,removed=[],positions=[[965.2+min(95,250*(2*t/24)),5547.36] for t in range(25)],results=[dict(pair=[held,'Center intake belt'],status='fail',witness=dict(t=1,reason='Did not seat on the tested powered approach.'))]))
massRows={'Chassis':(5,7),'Pivot support':(3.86,6.23),'Pivoting cassette':(1.2,2.3),'Piece support':(.78,1.61),'Cassette drive':(2.27,4.56),'Electrical bay':(3.6,5.9),'Capture latch':(.22,.51),'Solo climber':(2.7,4.8)}
for m in meta.values():
    if m['group'].startswith('Swerve'):massRows[m['group']]=(2.4,3.1)
mass=[dict(id=g,kg=list(q),members=[n for n,m in meta.items() if m['group']==g and n!='Battery and terminal case']) for g,q in massRows.items()]
mass+=[dict(id='Unplaced fasteners, harness and sensors',kg=[2,4.2],members=[]),dict(id='Unallocated design reserve',kg=[3,6],members=[])]
inspection=np.array([r['kg'] for r in mass]).sum(axis=0)
detail=[dict(name=n,kg=k,members=[v]) for n,k,v in [('battery',[5.5,6.2],'Battery and terminal case'),('one NOTE',[.23,.25],held),('finished bumper scenario',[4.6,6.6],'Blue bumper set')]]
running=inspection+np.array([r['kg'] for r in detail]).sum(axis=0)
loads={'Drive and steering':[n for n in G if n.startswith('Swerve ')],'Pivot and slide':['Pivot motor','Slide drive motor'],'Side belts':[n for n in G if n.startswith('Belt motor ·')],'Center belt':['Nose motor'],'Latch coil':['Latch solenoid and guide'],'Solo climber':['Climb winch drive'],'Control':['roboRIO flat','PDH','radio']}
power=[]
for title,lines in [('Aim and spin up',[(8,22),(5,18),(30,70),(0,2),(0,3),(0,0),(4,8)]),('Intake',[(16,42),(9,30),(8,26),(4,16),(1,3),(0,0),(4,8)]),('Climb with shooter idle',[(0,4),(0,4),(0,0),(0,0),(0,0),(18,55),(4,8)]),('Simultaneous acceleration and spinup',[(40,90),(12,34),(30,70),(4,16),(0,3),(0,0),(4,8)])]:
    for pack,v,r in [('weak',11.8,.027),('nominal',12.6,.018)]:
        lo,hi=np.sum(lines,axis=0);p=v-r*hi
        power.append(dict(case=title+' / '+pack,stage=title,batteryCondition=pack,openCircuitV=v,resistance=r,requiredMinV=8,loadedBusV=[p,v-r*lo],batterySideA=[int(lo),int(hi)],marginV=p-8,status='pass' if p>=8 else 'fail',lines=[dict(name=g,amps=a,members=loads[g]) for g,a in zip(loads,lines)]))
narr=Path('C:/Users/ehart/repos/frc-2024-poc/kickoff/strategic-narrative.md')
review=dict(source=dict(file='kickoff/strategic-narrative.md',sha256=sha(narr)),objectives=[dict(objective=o,disposition='open',basis=b) for o,b in [
 ('Acquire and repeat close Speaker deliveries','The ring is mechanically captured once seated. The free-piece center-belt trial still pushes a floor NOTE away. No scored run on this assembly.'),
 ('Finish the next delivery or PARK','A timed route with peer traffic and the terminal bumper position is still required.'),
 ('Solo ONSTAGE and partner-dependent HARMONY','A two-guide climber now has a budget, rear frame attachment and reach. Loaded hanging attitude, passive hold and the partner branch remain unverified.'),
 ('Optional overhead entry','No overhead-entry trial on this revision.')]])
notes=['A rear stop backing carries a 3 mm pad. Two rear-driven screws clamp it to brackets. Each bracket seats on the crossbar and has its own through bolt and nut. Each end of the crossbar now bears on 80 mm² of the side beam. Joint capacity remains open.',
       'A spring-return pin through the NOTE bore seats in a lower socket. A closed loop ties it to the upper bridge. The raised escape test now meets 13,382 mm³ of roof; the forward displacement meets 1,455 mm³ of pin. The opening sequence retracts the pin 66 mm before discharge. Flexible-piece and latch-force measurements remain open.',
       'Center belt uses a relieved nose and supported rear idler. The separate powered free-ring case replaces the prescribed intake animation as evidence of acquisition. No powered capture or score is credited.',
       'At 80° a service pin indexes the rotating cassette to its truck. A 4° indexing error obstructs the pin. Battery extraction at 125° remains the retained failing case. The slide must also be restrained before hands-on service.',
       'A rear chain saddle and telescoping guides reserve a whole endgame subsystem. Loaded attitude, holding dog and joint strength remain open. No ONSTAGE points credited.',
       'Mass and bus voltage use design allocations. Assign staged current limits. Force, inertia, motor ratings and an actual battery discharge test are still required.']
dump(HERE/'motion.json',render)
support={n:'Open the inspector for each adjacent part. The stop, bracket, crossbar and side beam have measured bearing faces.' for n in G if n.startswith('Rear stop')}
data=dict(issue='Greenfield P3 · Retained cassette',step=sha(outstep),receipt=sha(HERE/'checks.json'),motion=sha(HERE/'motion.json'),bodies=model,cases=render,palette=baseline['palette'],mass=mass,details=detail,power=power,inspection=inspection.tolist(),running=running.tolist(),status='Held · loose-piece pickup',corners=baseline['corners'],support=support,limits={'inspectionKg':56.69904625},**j('parked.json'),notes=notes,focus='Inspect Rear stop bracket to follow the load into the side beams. Play Loose NOTE to see the failed powered approach. The seated ring has positive retention.',required=baseline['required'],narrativeReview=review)
data['required']=[x for x in data['required'] if x[0] not in ['Retained cassette','Endgame package']]+[['Retained cassette',[pin,'Capture bridge','Lower pin socket','Rear stop bracket · left','Rear stop bracket · right']],['Endgame package',['Chain saddle','Climber bridge','Climb winch drive','Winch hold pawl']]]
dest=HERE.parent/'www'/'data.json';tmp=dest.with_suffix('.next');dump(tmp,data);tmp.replace(dest);print(data['issue'],len(G),inspection,running)

