"""Replay the two evaluated starts beside the nominal campaign, with their true initial coordinates."""
from pathlib import Path
import csv,json,math,numpy as np
p=Path(__file__).resolve().parent
roster=Path('R:/sim/pressure/evidence/speaker-repeatability-starts.csv')
stored=json.loads((p/'campaign-status.json').read_bytes());assert stored['terminal']
template=(p/'publish.py').read_text(encoding='utf8')
for attempt in stored['attempts']:
    evaluated=json.loads((p/('attempt-'+attempt+'.json')).read_bytes())
    obs=next(r for r in csv.DictReader(roster.open()) if r['attempt']==attempt)
    tag=attempt.lower()
    s=template.replace("'six-note-indexed'",repr('six-note-'+tag)).replace("Six-NOTE field rehearsal · indexed, fine",'Repeatability · '+attempt)
    s=s.replace("for idx,leg in enumerate(plan['plan']):","for idx,leg in enumerate(plan['plan'][:len(evaluated['pieces'])]):")
    s=s.replace("tag='dev-'+name","tag=attempt+'-'+name")
    s=s.replace("mark=np.array(leg['mark']);","mark=np.array(leg['mark'])+[float(obs[name+'_x_mm'])/1000,float(obs[name+'_y_mm'])/1000];")
    s=s.replace("pre=1.15 if idx==0 else 3.4","""if idx==0:
        out[0]+=[float(obs['robot_x_mm'])/1000,float(obs['robot_y_mm'])/1000];L=np.linalg.norm(out[1]-out[0]);td=max(1.5*L/2.7,math.sqrt(6*L/2.6),math.sqrt(6*abs(math.radians(float(obs['robot_heading_deg'])))/2.8));leg['outT']=[td]
    pre=1.15 if idx==0 else 3.4""")
    s=s.replace("P,scheduled(a),q,90*np.clip", "P,scheduled(a),q+(math.radians(float(obs['robot_heading_deg']))*(1-min(1,t/td)**2*(3-2*min(1,t/td))) if idx==0 else 0),90*np.clip")
    s=s.replace("(p/'mission.json').write_text","assert abs(clock-evaluated['endS'])<.001\n(p/('mission-'+attempt+'.json')).write_text")
    exec(compile(s,'repeatability viewer','exec'))
www=p.parent.parent/'www/data.json';d=json.loads(www.read_bytes())
for row in d['narrativeReview']['objectives']:
    if 'six distinct' in row['objective']:
        row.update(disposition='Reopen preliminary scoring design',basis='Nominal fine replay scored six in 115.07 s. R01 and R02 both missed a required sensor event after full opening passage. With 18 starts remaining, this controller cannot reach 19/20.')
    if 'ONSTAGE' in row['objective']:
        row.update(disposition='Capture geometry under revision',basis='The disabled-hold model started with ideal gated points. The original closed keepers intersect the as-installed FIRST chain by 188.0 and 186.2 mm³ at that registration. A new approach reference and two guided gates are under test.')
d['focus']='The indexed scorer missed a required count on each of the first two varied setups. Select R01 or R02 to see the floor pickup, loaded return and field rebound.'
new=www.with_suffix('.next');new.write_text(json.dumps(d,separators=(',',':')),encoding='utf8');new.replace(www)

