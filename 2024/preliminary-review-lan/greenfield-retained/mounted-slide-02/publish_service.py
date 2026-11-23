"""Publish a case-specific motor service scene on the existing preliminary review."""
from pathlib import Path
import copy,json,math,numpy as np
p=Path(__file__).resolve().parent;web=p.parent.parent/'www'
def read(f):return json.loads(f.read_bytes())
def save(f,obj):
    tmp=f.with_suffix('.service-staging');tmp.write_text(json.dumps(obj,separators=(',',':')),encoding='utf8');tmp.replace(f)
m=read(p/'motion.json');r=read(p/'supported-service.json');stack=read(p/'drive-service.json')
assert not m['hits'] and all(not v['hits'] for k,v in r['checks'].items() if 'hits' in v)
assert all(not v['hits'] for v in r['checks']['draw vs supported stock'].values())
assert all(not r['hubClockToolChecks'][str(d)]['hits'] for d in [-10,0,10])
assert all(not r['positive'] for r in stack['checks'].values() if 'positive' in r)
mesh=read(p/'display-mesh.json');assert all(v['closed'] for v in mesh.values())
webdata=web/'data.json';doc=read(webdata);g=doc['concepts']['guided'];s=next(c for c in g['cases'] if c['key']=='guided-service');idx=r['serviceIndex']
# Purposeful names for selectable ports. The purchased model has three equal-gauge leads and one larger sensor bundle.
rename=lambda n: (('Encoder' if n[-1]=='0' else 'Phase '+n[-1])+' '+n[:-2]) if n.startswith(('lead ','socket ','plug ')) else n
old=list(mesh);mesh={rename(k):v for k,v in mesh.items()}
case=copy.deepcopy(s);case.update(id='Slide motor cartridge service',key='slide-motor-service',time=[],positions=[],measure=[],frames={},series={},stage='power-off service',fixture='',gap=None,status='sampled',basis='Empty cassette supported at −40° with slide and rotation locks engaged, power isolated. Pull the four chassis-side connectors clear, loosen the motor-side shaft clamp at the indexed service clock, release the two cap retainers, then withdraw the bench-assembled motor 45 mm. Connector volumes are reservations at measured motor wire exits.')
newRig={}
for n in old:
    group='cartridge' if n in ('Slide drive motor','Removable cap and ears') or n.startswith(('lead ','plug ','M3 on cap')) else 'sockets' if n.startswith('socket ') else 'retainers' if n.startswith('M4 cap retention') else 'frame'
    newRig[rename(n)]=group
case['rig']={**s['rig'],**newRig};case['series']={k:[] for k in set(case['rig'].values())}
clamp=lambda t:float(np.clip(t,0,1))
I=[1,0,0,0,0,1,0,0,0,0,1,0]
def off(x=0,y=0,z=0):return [1,0,0,x,0,1,0,y,0,0,1,z]
for i in range(181):
    t=i/30;unplug=24*clamp((t-.3)/1.2);unscrew=20*clamp((t-2.6)/1.1);draw=45*clamp((t-3.9)/1.5)
    moving={'cartridge':off(draw),'sockets':off(0,unplug/math.sqrt(2),unplug/math.sqrt(2)),'retainers':off(0,0,unscrew)}
    for k in case['series']:
        tr=moving.get(k,s['series'][k][idx] if k in s['series'] else I)
        case['series'][k].append(tr)
    case['positions'].append(s['positions'][idx]);case['time'].append(t)
    phase='Isolate power and verify both locks' if t<.3 else 'Unplug three phases and encoder' if t<1.5 else 'Loosen motor-side clamp with indexed hub' if t<2.6 else 'Remove both top retaining screws' if t<3.9 else 'Withdraw the motor with its cap'
    case['measure'].append(f'{phase} · cartridge {draw:.0f} / 45 mm')
case['results']=[dict(pair=['Double pinch hub','Receiver and cap screws'],status='pass',witness=dict(reason='Reduced head height and two relocated receiver screws clear the full rotating cylinder. The first layout had 56.53 mm³ of hub interference with a receiver screw.')),dict(pair=['Clamp driver','Cassette and stationary rail receiver'],status='pass',witness=dict(reason='Both 75 mm driver corridors clear at −10°, 0° and +10° about the service clock. At +90° they enter the feed belt and the receiver.')),dict(pair=['Slide drive motor','Chassis and disconnected sockets'],status='pass',witness=dict(reason='Continuous straight 45 mm removal volume clears the supported robot. The connected-pin control resists a 1 mm sideways pull at all four ports.')),dict(pair=['Round shaft and clamp','Installed torque capacity'],status='open',witness=dict(reason='Projected peak torque is 0.364 N·m. Clamp grip, real connector fit, cable restraint and the selected screw/nut life still require verification.'))]
# Charge all the proposed package on top of the upper budget, deliberately retaining the predecessor motor allowance.
dry=(.142+read(p/'fit.json')['capAndReceiverMassKg']+read(p/'fit.json')['extraHardwareKg'])
others=sum(v['volume']*(7.85e-6 if n.startswith(('Spindle','M2.5')) else 2.7e-6) for n,v in mesh.items() if n=='Double pinch hub' or n.startswith(('Spindle','M2.5')))
base=dry+others;mass=dict(id='Slide motor service package, gross addition',members=sorted(mesh.keys()-{'Slide bearing mount 220'}),kg=[base+.04,base+.09])
trial=dict(case=case,add=mesh,mass=mass,step=stack['step'],budgetBasis='Gross motor, machined stock and fasteners plus 40–90 g wiring and connector allowance. Prior group budget retained. Cold assembly fit is documented for the installed source model.')
doc['slideService']=trial;save(webdata,doc)
js=web/'review.js';code=js.read_text(encoding='utf8')
if 'const slideService=data.slideService' not in code:
    marker=' if(variant){const bodies='
    assert code.count(marker)==1
    code=code.replace(marker,""" const slideService=data.slideService;
 if(conceptKey==='guided' && slideService){
   variant={...variant,cases:[...variant.cases,slideService.case]};
   if(requested===slideService.case.key){
     const gross=slideService.mass.kg, combine=a=>a.map((v,i)=>v+gross[i]);
     const originalMass=variant.mass.map(row=>({...row,members:row.members.filter(n=>!slideService.mass.members.includes(n))}));
     variant={...variant,add:{...variant.add,...slideService.add},mass:[...originalMass,slideService.mass],
       inspection:combine(variant.inspection),running:combine(variant.running),issue:'Guided cassette · slide motor service',
       step:slideService.step,focus:slideService.case.basis,validationText:'Service-specific installed package. Connector envelopes and the real wire exits are included. See the checks for the fully supported maintenance position.'};
   }
 }
"""+marker)
    marker="$('motion').onchange=activate;"
    assert code.count(marker)==1
    code=code.replace(marker,"""$('motion').onchange=()=>{const next=current().key;const service=slideService?.case.key;if(service && (requested===service)!==(next===service)){const u=new URL(location.href);u.searchParams.set('concept','guided');u.searchParams.set('case',next);location.assign(u.href);return;}activate();};""")
    js.write_text(code,encoding='utf8')
(p/'service-view-receipt.json').write_text(json.dumps(dict(case=case['key'],frames=len(case['time']),bodies=len(mesh),materialMassKg=base,grossKg=mass['kg'],highInspectionMarginKg=56.699-g['inspection'][1]-mass['kg'][1],end=case['measure'][-1]),indent=2))

