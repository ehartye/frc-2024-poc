"""Replay the completed bounded lift and the actual game-piece positions through the eligible stationary resets."""
from pathlib import Path
import json,math,copy,sys,numpy as np
p=Path(__file__).resolve().parent;root=p.parent
source=(root/'six-piece-02/publish.py').read_text(encoding='utf8')
exec(compile(source.split("plan=read(p/'plan.json')")[0],'presentation coordinates','exec'))
sys.path[:0]=[str(root/'loaded-actuation-01'),str(root/'follower-01')]
from smooth import times,table
dfile=root.parent/'www/data.json';d=read(dfile);g=d['concepts']['guided'];f=d['concepts']['field']
plan=read(p/'plan.json');receipt=read(p/'run-dev-48.json');assert receipt['passMission'] and receipt['count']==6 and plan['placedStep']==g['step']
c=copy.deepcopy(next(i for i in g['cases'] if i['key']=='lateral-nominal-48'))
c.update(id='Six NOTES · bounded loaded lift',key='loaded-cycle-48',time=[],measure=[],positions=[],results=[],series={k:[] for k in c['series']},basis='Profile A pickup, 4.8 s smooth lift, lateral firing station and free NOTE at 0.5 ms. This nominal development run is scored by the complete opening and both field counters. The empty cassette returns while parked after the firing delay where its swept space and residual power permit. Cassette tracking is commanded. Motor demand and sampled stock clearance are checked independently. Physical traction, pack condition and counter-wheel speed remain unmeasured.')
exec(compile(source[source.index('def append('):source.index('clock=0;count=0;')],'append common pose and time','exec'))
q0=read(p/'dev-middle-48.json')['pitchDegrees'];startX=read(root/'pickup-lip/layout.json')['pickPivot'][0]
xt,_=table(q0,122,startX)
def arm(t):
    tc=np.clip(t/4.8,0.,1.);a=q0+(122-q0)*tc*tc*(3-2*tc)
    return a,1000*np.interp(tc*3.4,times,xt)
for idx,(leg,pr) in enumerate(zip(plan['plan'],receipt['parts'])):
    k=leg['id'];tag='dev-'+k+'-48';cf=p/(tag+'.json');sf=p/('split-outerlane-'+tag+'-6.88.json')
    r=read(cf);s=read(sf);assert sha(cf)==pr['carry'] and sha(sf)==pr['shot'] and s['sourceCarry']==pr['carry']
    start=pr['depart'];q=leg['yaw'];out=np.array(leg['out']);ld=np.array(leg['loaded']);mark=np.array(leg['mark']);station=ld[-1]
    travel=sum(leg['outT']);pre=1.15 if idx==0 else 0
    for t in np.arange(0,travel+pre,.065):
        xy,_,_=moment(out,leg['outT'],t);a=q0 if idx else q0*np.clip((t-travel)/1.15,0,1)**2*(3-2*np.clip((t-travel)/1.15,0,1))
        append(start+t,xy,a,pose(0,*((mark-xy)*1000),25.4),startX if idx else scheduled(a),q,0,f'{clean(k)} · empty approach · {idx}/6 scored')
    carryStart=start+travel+pre;assert abs(carryStart+r['finish']-pr['shotStart'])<1e-5
    for row in r['trajectory'][::5]:
        t=row[0];rot=q;xy=out[-1]+yaw(q)[:2,0]*min(leg['advance'],t*.25)
        if t>2.82+r['seatDelay']:
            xy,j,u=moment(ld,leg['loadedT'],t-2.82-r['seatDelay'])
            if idx==0:rot=q*(1-u*u*(3-2*u)) if j==1 else (0 if j>1 else q)
        a,x=arm(t-r['startLift'])
        phase='floor pickup' if t<2.82+r['seatDelay'] else 'loaded return'
        append(carryStart+t,xy,a,yaw(rot)@ring(row[4:8],np.array(row[1:4])*1000),x,rot,math.degrees(row[-1]),f'{clean(k)} · {phase} · {idx}/6 scored')
    limit=pr['both'] if idx==5 else receipt['parts'][idx+1]['depart']-pr['shotStart']
    resetStart=s['feedTime']+1. if pr['resetParallelEligible'] else pr['both']
    samples=s['trace'];ts=np.array([v['t'] for v in samples]);stamp=np.unique(np.r_[np.arange(0,limit,.045),pr['both'],limit])
    for t in stamp:
        a,x=arm(4.8-max(0,t-resetStart)) if idx<5 else (122,185)
        v=samples[min(np.searchsorted(ts,t),len(samples)-1)]
        P=ring(v['q'][3:],(np.array(v['q'][:3])-[*station,0])*1000)
        count=idx+int(t>=pr['both']);state='shooting' if t<s['release']['t'] else 'free flight' if t<s['opening']['fullyThroughByS'] else 'inside speaker'
        if idx<5 and t>=resetStart:state+=' · cassette resetting'
        append(pr['shotStart']+t,station,a,P,x,0,90*np.clip((a-102)/18,0,1),f'{clean(k)} · {state} · {count}/6 scored')
    c['results'].append(dict(pair=[clean(k),'Speaker'],status='sampled',witness=dict(reason=f'Both counter pairs confirmed at {pr["counterClock"]:.3f} s. Full-opening margin {1000*pr["opening"]["minimumMarginM"]:.1f} mm.')))
c['results'].append(dict(pair=['Cassette and slide','Robot frame'],status='sampled',witness=dict(reason='The 4.8 s path cleared 1,208 close stationary/moving checks in 118 sampled states. Current and voltage use mass bounds. Loaded tracking, friction and between-pose contact remain open.')))
assert '6/6 scored' in c['measure'][-1] and abs(c['time'][-1]-receipt['elapsedS'])<1e-5
g['cases']=[v for v in g['cases'] if v['key']!=c['key']]+[c];g['defaultCase']=c['key']
g['focus']='Profile A and a bounded 4.8 s lift scored six in 118.46 s on the nominal coarse model. Inspect the match clock and field count here. Frozen starts, actual motor installation, sustained current and closed-loop tracking remain open.'
power=read(root/'loaded-actuation-01/purchased-power.json')
powerCase=next(i for i in g['power'] if i['case']=='Loaded lift and drive · 4.8 s candidate')
governing=min((x for x in power['cases'] if x['auxA']==25),key=lambda x:x['weakBusMinV'])
amps=governing['batteryBreakdownAtPeakA']
assert abs(sum(amps.values())-(11.8-governing['weakBusMinV'])/.027)<.0001
powerCase['lines']=[dict(name='Concurrent accessories',amps=[25,25],members=[]),dict(name='Swerve traction at governing instant',amps=[0,amps['fourDrives']],members=[n for n in c['rig'] if n.startswith('Swerve drive motor')]),dict(name='Loaded cassette pivot at governing instant',amps=[0,amps['pivot']],members=['Pivot motor']),dict(name='Screw-driven slide at governing instant',amps=[0,amps['slide']],members=['Slide drive motor'])]
powerCase['basis']='At the minimum bus voltage the center-low return draws %.1f A total. Loads are synchronized battery-side values at this instant. See the separate commissioning ranges for the motor phase currents.'%sum(amps.values())
tmp=dfile.with_suffix('.bounded-next');tmp.write_text(json.dumps(d,separators=(',',':')),encoding='utf8');tmp.replace(dfile)
(p/'viewer-receipt.json').write_text(json.dumps(dict(step=g['step'],source=sha(p/'run-dev-48.json'),frames=len(c['time']),key=c['key'],seconds=c['time'][-1],last=c['measure'][-1],power=sha(root/'loaded-actuation-01/purchased-power.json'),interference=sha(p/'clearance.json')),indent=2))
print(c['key'],len(c['time']),c['time'][-1])

