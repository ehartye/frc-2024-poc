"""Display service-only stock in the service sequence and preserve the mobile truck geometry at its real pose."""
from pathlib import Path
import math,json,hashlib,numpy as np,sys
p=Path(__file__).resolve().parent;root=p.parent;web=root.parent/'www/data.json'
read=lambda f:json.loads(f.read_bytes())
doc=read(web);g=doc['concepts']['guided'];f=doc['concepts']['field'];bodies={**doc['bodies'],**f['add']};step=read(p/'truck-lock.json');assert not step['contacts']
meshes=read(p/'truck-lock-mesh.json');tools=[n for n in meshes if n.startswith('Service slide')]
for n,v in meshes.items():
    isLink=n=='Service slide link';service=n in tools
    source=bodies.get(n,bodies['Battery service pin'])
    row={**source,**v}
    if service:
        row.update(group='Battery service tools',material='Machined aluminum plate' if isLink else 'Steel shoulder screw, M5 tip',purpose='Restrain fore/aft cassette travel while the battery is being handled.' if isLink else 'Positively locate one end of the removable service link.',rationale='The rotation pin leaves a separate slide free. Two separated shoulder seats carry load into the right frame.',basis='Service-only fixture, stored off the robot. The shank carries shear and the tapped end retains the screw. Validate strength, fit and tool grip before physical use.',serviceOnly=True)
    else:row['basis']=source['basis']+' A new Ø6.4 mm shoulder seat and M5 pilot locate the removable truck restraint.'
    g['add'][n]=row
g['step']=step['assembly']
case=next(c for c in g['cases'] if c['key']=='guided-service');base=next(c for c in f['cases'] if c['key']=='field-r08')
sys.path.insert(0,str(root/'follower-01'));from stroke import scheduled
def M(a=0,x=0,y=0,z=0):
    s=math.sin(math.radians(a));c=math.cos(math.radians(a));return np.array([[c,0,-s,x],[0,1,0,y],[s,0,c,z],[0,0,0,1.]])
def flat(m):return m[:3].round(8).ravel().tolist()
rig={n:base['rig'].get(n,'arm' if n in g['add'] else 'frame') for n in bodies.keys()|g['add'].keys()}
rig.update({'Battery service pin':'servicePin','Battery and terminal case':'withdrawal','Catch dog':'slide','Service slide link':'link','Service slide shoulder screw · truck':'truckScrew','Service slide shoulder screw · frame':'frameScrew'})
case.update(rig=rig,time=[],measure=[],positions=[],series={k:[] for k in set(rig.values())},basis='Support the empty cassette at −40° while installing both restraints. The disk pin holds rotation. The removable link and its two shoulder screws hold the translating truck. Isolate power and disconnect the leads before the 370 mm battery lift. This view shows insertion and extraction envelopes; operator hand clearance and physical proof load are open.')
ip=np.linalg.inv(M(125,185,0,501));truck=M(0,scheduled(-40)-185);arm=M(-40,scheduled(-40),0,501)@ip
clamp=lambda t: min(1,max(t,0))
for i in range(151):
    t=i/30;pinMm=12*clamp(t/.6);linkMm=-75*(1-clamp((t-.6)/.6));fmm=-100*(1-clamp((t-1.2)/.65));tmm=-100*(1-clamp((t-1.85)/.65));lift=370*clamp((t-2.75)/2.25)
    m=dict(frame=M(),arm=arm,slide=truck,pin=truck,servicePin=truck@M(0,0,-pinMm,0),withdrawal=M(0,0,0,lift),piece=M(),keeper=arm,flap=arm,link=M(0,0,linkMm,0),truckScrew=M(0,0,tmm,0),frameScrew=M(0,0,fmm,0))
    for k in case['series']:case['series'][k].append(flat(m[k]))
    phase='Index the rotation disk' if t<.6 else 'Seat the truck restraint' if t<1.2 else 'Install the frame screw' if t<1.85 else 'Install the truck screw' if t<2.5 else 'Slide restrained · power isolated'
    case['time'].append(t);case['positions'].append([2300,5550]);case['measure'].append(f'{phase} · pack withdrawal {lift:.0f} / 370 mm')
case['results']=[r for r in case['results'] if r['pair'][0]!='Translating truck']+[dict(pair=['Translating truck','Frame'],status='sampled',witness=dict(reason='Removable link seats on the actual right A-frame and translating block. Two shoulder screws carry shear. ±2 mm misalignment obstructs each tested truck screw. The 75 mm link insertion and straight driver corridor clear. The pack remains at least 81 mm from the tool. Physical proof load and operator handling remain open.'))]
g['fixtureOnlyNames']=tools
tmp=web.with_suffix('.lock-next');tmp.write_text(json.dumps(doc,separators=(',',':')),encoding='utf8');tmp.replace(web)

