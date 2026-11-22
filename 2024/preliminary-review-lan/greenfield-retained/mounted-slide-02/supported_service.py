"""Check the installed drive and its tools with the empty cassette positively restrained at the battery-service pose."""
from pathlib import Path
import json,sys,math,numpy as np,hashlib
p=Path(__file__).resolve().parent;root=p.parent
sys.path.insert(0,str(root/'assembly-recheck-01'))
from sweep import load_solids,name,bounds,common,place,mat,distance
from layout import box,cyl,write_step,G,X
def read(f):return json.loads(f.read_bytes())
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
def shapes(f):return {name(n):s for n,s in load_solids(f)[0].items()}
def m4(flat):return np.vstack([np.array(flat).reshape(3,4),[0,0,0,1]])
d=read(root.parent/'www/data.json');g=d['concepts']['guided'];s=next(k for k in g['cases'] if k['key']=='guided-service')
idx=next(i for i,t in enumerate(s['time']) if 2.58<t<2.65)
assert 'power isolated' in s['measure'][idx]
base=shapes(root/'guided-hoist-01/service-locked-robot.step')
stock={n:place(v,m4(s['series'][s['rig'][n]][idx])) for n,v in base.items() if n not in ['Slide drive motor','Slide bearing mount 220','Chambered NOTE'] and n not in s['removed']}
# Three removable service tools are separate from the ordinary 232-body archive.
toolfile=root/'guided-hoist-01/removable-truck-lock.step';tset=shapes(toolfile)
for n in g['fixtureOnlyNames']:
    stock[n]=place(tset[n],m4(s['series'][s['rig'][n]][idx]))
fixed=shapes(p/'slide-cartridge.step');fixed['Slide drive motor']=fixed.pop('SOLID');del fixed['Trial coupling blank']
drive=shapes(p/'drive-service.step')
bb=lambda s:np.array(bounds(s))
def screen(subjects,other):
    count=0;hits=[];O={k:bb(v) for k,v in other.items()}
    for k,v in subjects.items():
        b=bb(v)
        for j,h in other.items():
            if not np.all((b[3:]+.0001>=O[j][:3])&(O[j][3:]+.0001>=b[:3])):continue
            count+=1;vol=common(v,h)
            if vol>.001:hits.append([k,j,vol])
    return dict(subjects=len(subjects),obstacles=len(other),narrow=count,hits=hits)
report=read(p/'drive-service.json');checks={};D=np.array([0.,1,1])/math.sqrt(2)
def slab(v,delta):
    b=bb(v);lo=b[:3]+np.minimum(delta,0);hi=b[3:]+np.maximum(delta,0);return box(*lo,*(hi-lo))
field=stock | fixed; moving=['Slide drive motor','Removable cap and ears','M3 on cap 0','M3 on cap 1']
checks['installed hardware vs supported robot']=screen(fixed|drive,stock)
ports={};pull={};hold={};hand={}
for i,wire in enumerate(report['path']):
    e=np.array(wire['end']);r=wire['connectorReservationDiameterMm']/2
    pull[str(i)]=cyl(e+22*D,D,r,45);hold[str(i)]=place(drive['socket '+str(i)],mat(0,*(D*24)))
    hand[str(i)]=cyl(e+22*D,D,10,48)
checks['unplug all four']=screen(pull,field);checks['finger approach']=screen(hand,field)
for deg in [-180,-90,-10,0,10,90]:
    a=math.radians(deg);c=math.cos(a);f=math.sin(a);R=np.array([[1,0,0],[0,c,-f],[0,f,c]]);M=np.eye(4);M[:3,:3]=R;center=np.array([0,275,428]);M[:3,3]=center-R@center
    ports[str(deg)]=screen({str(x):place(cyl((x,281.05,432.50),(0,0,1),1.65,75),M) for x in [238.4,247.6]},field|{k:v for k,v in drive.items() if k.startswith(('lead','plug','socket'))})
checks['cartridge retaining screws and drivers']=screen({str(y):cyl((247,y,433),(0,0,1),3.5,88) for y in (253,297)},stock|{k:v for k,v in drive.items() if k.startswith(('lead','plug','socket'))})
# For the complete motor use a continuous clearance proof via the bounding box of the complete translated source.
for k in moving+list(k for k in drive if k.startswith(('lead','plug'))):
    v=(fixed|drive)[k];lo=bb(v)
    checks.setdefault('draw vs supported stock',{})[k]=screen({k:slab(v,np.array([45.,0,0]))},stock|{'held '+j:h for j,h in hold.items()})
r=dict(registered=g['step'],fixture=report['step'],serviceIndex=idx,servicePose=s['measure'][idx],rootMembers=len(stock),checks=checks,hubClockToolChecks=ports,source=sha(toolfile))
(p/'supported-service.json').write_text(json.dumps(r,indent=2))
print({k:v['hits'] for k,v in checks.items() if 'hits' in v});print({k:v['hits'] for k,v in checks['draw vs supported stock'].items() if v['hits']});print({k:v['hits'] for k,v in ports.items()})

