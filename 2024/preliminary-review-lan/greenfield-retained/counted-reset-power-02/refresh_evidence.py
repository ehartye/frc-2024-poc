"""Bind the current climb display to the charged free-body trials and show the counter limitation."""
from pathlib import Path
import math,json,numpy as np
p=Path(__file__).resolve().parent;root=p.parent
read=lambda f:json.loads(f.read_bytes())
df=root.parent/'www/data.json';doc=read(df);g=doc['concepts']['guided']
src=(root/'guided-hoist-01/publish-review.py').read_text(encoding='utf8')
exec(compile(src[src.index('def M('):src.index(';hooks=read')],'original measured poses','exec'))
hold=src[src.index("    for r in row['trace']:"):src.index("    for status,pair,reason")]
hold='\n'.join(s[4:] for s in hold.splitlines())
now=read(p/'endgame-clock.json');co,fine=now
for side in ['front','rear']:
    c=next(c for c in g['cases'] if c['key']=='score-to-climb-'+side)
    row=read(p/f'climb-gross-{side}.json')
    i=next(i for i,msg in enumerate(c['measure']) if msg.startswith('Capture and lock'))
    entry=c['time'][i];anchor=c['positions'][i]
    for k in ['time','positions','measure','chain']:c[k]=c[k][:i]
    for k in c['series']:c['series'][k]=c['series'][k][:i]
    exec(compile(hold,'charged solved poses','exec'))
    for j in range(i,len(c['time'])):c['measure'][j]+=f" · {c['time'][j]:.2f} s from sixth score"
    c['floorCheckMm']=1000*row['disabledMinTireOrBumperM']
    c['results']=[r for r in c['results'] if r['pair']!=['Tires and bumper','Carpet']]+[dict(pair=['Tires and bumper','Carpet'],status='sampled',witness=dict(reason=f"At {row['modeledRobotMass']:.3f} kg, including the complete additional slide package, minimum disabled clearance was {c['floorCheckMm']:.2f} mm. Both hooks carried positive load and no tire or bumper contact carried weight."))]
    c['basis']='Travel from the actual scoring station through the archived traffic. The climb uses the 71.764 kg dynamics trial with both hook loads and five seconds of disabled hold. The 24 s terminal decision and the six-piece practice run have separate clocks. Waiting for the sixth count before departing misses a combined 135 s TELEOP by 2.09 to 3.51 s. Starting the drive one second after the final feed leaves only 0.18 s in this timing model. Pin strength and unexpected delays remain open.'
for n in [48,96]:
    c=next(c for c in g['cases'] if c['key']==f'continuous-cycle-{n}')
    key=['Shared field counter','Repeat deliveries'];rows=read(p/'counter-clear.json');bad=[r['id'] for r in rows if r['n']==n and r['sustainedBothBeamsClearFromSampleS'] is None]
    c['results']=[r for r in c['results'] if r['pair']!=key]+[dict(pair=key,status='open',witness=dict(reason='The final seven-second sample still had an obscured beam for '+', '.join(bad)+'. Each shot triggered independently. Shared counter clearance through the next arrival is under test.'))]
    c['basis']=c['basis'].split(' Six independent')[0]+' Six independent field responses are shown. A common collector that can clear every shot is still under test.'
g['focus']='Profile A produced six individual counter events in 116.50 and 115.08 seconds. Shared counter clearance and repeatability on the 20 frozen starts govern the preliminary design decision.'
tmp=df.with_suffix('.stage-review');tmp.write_text(json.dumps(doc,separators=(',',':')),encoding='utf8');tmp.replace(df)

