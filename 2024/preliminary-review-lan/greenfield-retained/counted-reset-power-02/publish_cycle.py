"""Animate the actual six free NOTES using the recorded carry, reset and field observation clocks."""
from pathlib import Path
import copy, hashlib, json, math, sys
import numpy as np
p=Path(__file__).resolve().parent
root=p.parent
n=int(sys.argv[1])
source=(root/'six-piece-02/publish.py').read_text(encoding='utf8')
exec(compile(source.split("plan=read(p/'plan.json')")[0],'shared viewer frames','exec'))
sys.path.insert(0,str(root/'loaded-actuation-01'))
from smooth import times,table
docfile=root.parent/'www/data.json'
doc=read(docfile);g=doc['concepts']['guided']
run=read(p/f'run-dev-{n}.json');plan=read(p/'plan.json')
assert run['passMission'] and run['terminal'] and run['count']==6 and g['step']==plan['placedStep']
c=copy.deepcopy(next(x for x in g['cases'] if x['key']=='loaded-cycle-48'))
c.update(key=f'continuous-cycle-{n}',id=f'Six NOTES · continuous state · {n} segments',time=[],measure=[],positions=[],results=[],series={k:[] for k in c['series']},basis='Six distinct free floor NOTES on one clock. Roller speeds and position, flap angle and flap velocity carry between pickups. Each score requires the full opening and both field counter pairs. The chassis, pivot and slide follow commanded paths. Carpet grip and field counter speed await physical calibration.')
exec(compile(source[source.index('def append('):source.index('clock=0;count=0;')],'shared pose recorder','exec'))
q0=read(p/f'dev-middle-{n}.json')['pitchDegrees'];x0=read(root/'pickup-lip/layout.json')['pickPivot'][0]
xt,_=table(q0,122,x0)
def arm(t):
    u=np.clip(t/4.8,0,1)
    return q0+(122-q0)*u*u*(3-2*u),1000*np.interp(3.4*u,times,xt)
def empty(item):
    f=p/f"empty-dev-{item['id']}-{n}.json"
    assert sha(f)==item['emptyMotion']
    return np.array(read(f)['trace'])
def flap(e,t):return math.degrees(np.interp(t,e[:,0],e[:,2]))
joins=[]
for i,(leg,part) in enumerate(zip(plan['plan'],run['parts'])):
    k=part['id'];assert k==leg['id']
    label=clean(k); tag=f'dev-{k}-{n}'
    cf=p/(tag+'.json');sf=p/('split-tail_.25_0-'+tag+'-6.88.json')
    r=read(cf);s=read(sf);e=empty(part)
    assert sha(cf)==part['carry']==s['sourceCarry'] and sha(sf)==part['shot']
    q=leg['yaw'];out=np.array(leg['out']);loaded=np.array(leg['loaded']);mark=np.array(leg['mark']);station=loaded[-1]
    start=part['depart'];travel=sum(leg['outT']);pre=1.15 if i==0 else 0;carryStart=start+travel+pre
    et0=0 if i==0 else start-run['parts'][i-1]['shotStart']
    assert abs(carryStart+r['finish']-part['shotStart'])<1e-5
    for t in np.arange(0,travel+pre,.060):
        xy,_,_=moment(out,leg['outT'],t);angle=float(np.interp(et0+t,e[:,0],e[:,1]))
        append(start+t,xy,angle,pose(0,*((mark-xy)*1000),25.4),x0 if i else scheduled(angle),q,flap(e,et0+t),f'{label} · approaching floor NOTE · {i}/6 scored')
    for row in r['trajectory'][::4]+r['trajectory'][-1:]:
        t=row[0];rot=q;advance=min(leg['advance'],.25*t);xy=out[-1]+yaw(q)[:2,0]*advance
        if t>=2.82+r['seatDelay']:
            xy,j,u=moment(loaded,leg['loadedT'],t-2.82-r['seatDelay'])
            if i==0:rot=q*(1-u*u*(3-2*u)) if j==1 else (0 if j>1 else q)
        a,x=arm(t-r['startLift']);local=np.array(row[1:4])
        append(carryStart+t,xy,a,yaw(rot)@ring(row[4:8],1000*local),x,rot,math.degrees(row[-1]),f'{label} · '+('pickup' if t<2.82+r['seatDelay'] else 'loaded return')+f' · {i}/6 scored')
    stationExit=np.array(r['endPieceQpos'][:3])-[leg['advance'],0,0]+[*station,0]
    join=float(np.linalg.norm(stationExit-np.array(s['initialState'][:3])))
    assert join<1e-6,join
    traceJoin=float(np.linalg.norm(np.array(r['trajectory'][-1][1:4])+[*station,0]-np.array(s['initialState'][:3])))
    assert traceJoin<2.e-5,traceJoin
    joins.append(max(join,traceJoin))
    limit=part['both'] if i==5 else run['parts'][i+1]['depart']-part['shotStart']
    trigger=s['feedTime']+1
    if i<5:
        assert part['resetParallelEligible'] and abs(part['resetEnd']-run['parts'][i+1]['depart'])<1e-6
        nextEmpty=empty(run['parts'][i+1])
    ts=np.array([v['t'] for v in s['trace']])
    for t in np.unique(np.r_[np.arange(0,limit,.045),part['both'],limit]):
        v=s['trace'][np.argmin(abs(ts-t))];P=ring(v['q'][3:],(np.array(v['q'][:3])-[*station,0])*1000)
        a,x=arm(4.8-max(0,t-trigger)) if i<5 else (122,185)
        angle=flap(nextEmpty,t) if i<5 else math.degrees(r['endFlapRad'])
        label2='feeding' if t<s['release']['t'] else 'free flight' if t<s['opening']['fullyThroughByS'] else 'inside speaker'
        if i<5 and t>=trigger:label2+=' · empty cassette returning'
        append(part['shotStart']+t,station,a,P,x,0,angle,f'{label} · {label2} · {i+int(t>=part["both"])}/6 scored')
    c['results'].append(dict(pair=[label,'Speaker'],status='sampled',witness=dict(reason=f'Full opening margin {1000*part["opening"]["minimumMarginM"]:.1f} mm. Both field counters confirmed at {part["counterClock"]:.3f} s.')))
assert abs(c['time'][-1]-run['elapsedS'])<1e-5 and '6/6 scored' in c['measure'][-1]
c['results'].append(dict(pair=['Empty cassette reset','Weak battery'],status='sampled',witness=dict(reason='With 35 A of other loads throughout reset the bus stays above 9.82 V and the limiting motor has 2.09 V available beyond its demand. The earlier 30 A scheduling limit prevented overlap.')))
g['cases']=[x for x in g['cases'] if x['key']!=c['key']]+[c]
g['defaultCase']=c['key']
g['focus']=f"Six floor NOTES scored in {run['elapsedS']:.2f} s on the {n}-segment nominal simulation. The separate acceptance test requires 19 of 20 perturbed starts."
tmp=docfile.with_suffix('.new-cycle');tmp.write_text(json.dumps(doc,separators=(',',':')),encoding='utf8');tmp.replace(docfile)
(p/f'viewer-{n}.json').write_text(json.dumps(dict(run=sha(p/f'run-dev-{n}.json'),key=c['key'],frames=len(c['time']),clock=c['time'][-1],final=c['measure'][-1],shotHandoffPositionErrorsM=joins),indent=2))
print(c['key'],c['time'][-1],max(joins))

