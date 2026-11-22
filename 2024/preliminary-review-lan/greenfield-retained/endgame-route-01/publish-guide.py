"""Display the free-chassis trial and the articulated chain on the same field camera and clock."""
from pathlib import Path
import numpy as np,copy,json,sys,math
p=Path(__file__).resolve().parent;root=p.parent
def read(f):return json.loads(f.read_bytes())
web=root.parent/'www/data.json';doc=read(web);layout=doc['concepts']['field']
cases=layout['cases'];original=next(k for k in cases if k['key']=='field-climb')
source=read(p/'original-climb.json');trial=sys.argv[1] if len(sys.argv)>1 else 'long-front';row=read(p/(trial+'.json'));coarse=read(p/'full-hooks.json')
nms=set(coarse['hookGroup']);assert len(nms)==7
anchor=source['positions'][30]
def rigid(q):
    w,x,y,z=q;return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def M(a=0,x=0,y=0,z=0):
    ang=np.deg2rad(a);c,s=np.cos(ang),np.sin(ang);m=np.eye(4);m[:3,:3]=[[c,0,-s],[0,1,0],[s,0,c]];m[:3,3]=[x,y,z];return m
def flat(m):return m[:3].round(8).ravel().tolist()
park=M(125,185,0,501);ip=np.linalg.inv(park);axis=M(0,457.5,0,96)
c=copy.deepcopy(original);key='field-guide-short' if trial=='long-front' else 'field-guide';c.update(key=key,id=f'Guided-pull climb · {row["strokeM"]*1000:.0f} mm',basis=f'Simulated {row["strokeM"]*1000:.0f} mm guided retraction and a freely moving robot. Both hooks and the physical chain interact. The pitch lock, slide drive and slide stop are idealized. The moving guide and its transmission still require packaging and mass reconciliation.',results=[])
entry=original['time'][-1]-source['time'][-1]+source['time'][30]
ix=next(i for i,t in enumerate(c['time']) if t>=entry-.000001)
for n in ['time','positions','chain','measure']:c[n]=c[n][:ix]
for n in c['series']:c['series'][n]=c['series'][n][:ix]
c['series']['hookSlide']=list(c['series']['arm'])
for n in nms:
    if n!='Common keeper yoke and sliding rack':c['rig'][n]='hookSlide'
for r in row['trace']:
    t=r['t'];fr=np.eye(4);fr[:3,:3]=rigid(r['q']);fr[:3,3]=np.array(r['p'])*1000
    arm=fr@M(r['angle'],185,0,501);slide=arm@M(0,-1000*r['draw']);m=dict(frame=fr,piece=np.eye(4),pin=fr,arm=arm@ip,flap=arm@axis@M(0)@np.linalg.inv(axis)@ip,hookSlide=slide@ip,keeper=slide@M(0,0,0,r['gate']*1000)@ip)
    for n in c['series']:c['series'][n].append(flat(m[n]))
    c['time'].append(entry+t);c['positions'].append(anchor);c['chain'].append((np.array(r['chain'])[::2]*1000).tolist())
    desc='Power cut' if t>=5 else 'Taking load' if t>.35 else 'Capture'
    c['measure'].append(f"{desc} · {r['draw']*1000:.0f} mm stroke · {r['ground']:.1f} N floor load · {r['z']*1000:.1f} mm tire / bumper clearance")
peak=row['maxDisabledFloorN']
if row['disabledMinTireOrBumperM']<.010 and peak<=.1:
    c['results'].append(dict(pair=['Front tire','10 mm hold allowance'],status='fail',witness=dict(reason=f"Minimum sampled clearance {row['disabledMinTireOrBumperM']*1000:.1f} mm across the loaded hold. The design allowance is 10 mm.")))
    c['measure'][-1]+=f" · minimum hold clearance {row['disabledMinTireOrBumperM']*1000:.1f} mm"
if peak>.1:
    c['measure'][-1]+=f' · hold failed earlier ({peak:.0f} N floor load)'
    c['results'].append(dict(pair=['Front tire','Carpet'],status='fail',witness=dict(reason=f'Floor contact recurred during the five-second disabled hold; peak {peak:.1f} N over the reported interval.')))
c['results'].append(dict(pair=['Hoist carriage','Inspection budget'],status='fail',witness=dict(reason='The 0.750 kg provisional addition exceeds the 0.199 kg margin. Replacement mass and an installable drive are open.')))
if trial=='equipped-front':
    obj=next(o for o in layout['narrativeReview']['objectives'] if 'ONSTAGE' in o['objective'])
    obj.update(disposition='Guided pull under review',basis='The original inward swing hits a fixed stage rail. Both simulated CG extremes lifted on the 220 mm guided pull. The adverse hold had only 4.8 mm floor clearance against the 10 mm design allowance. The provisional carriage exceeds the mass margin by 551 g.')
    layout['focus']='Profile A remains the pickup baseline. Compare R08 before and after seat confirmation, the rejected inward swing, the guided climb and the PARK fallback.'
layout['cases']=[s for s in cases if s['key']!=key]+[c];tmp=web.with_suffix('.guided-next');tmp.write_text(json.dumps(doc,separators=(',',':')),encoding='utf8');tmp.replace(web)
print(c['key'],c['time'][-1],c['measure'][-1])
