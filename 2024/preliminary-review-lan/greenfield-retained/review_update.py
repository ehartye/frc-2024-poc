from travel import *
base=HERE.parent/'www'/'data.json';data=json.loads(base.read_bytes())
variant=json.loads((HERE/'comparison-view.json').read_bytes())
load=json.loads((HERE/'loaded-pose.json').read_bytes())
variant['notes'][-1]=f"A finer screen covered {load['samples']:,} assumed centers of gravity. A real bumper vertex would sit {abs(load['lowestStockVertexMm']):.1f} mm below the carpet at CG {load['centerMm']}. Hook placement and loaded attitude require a revision before crediting a hang."
variant['focus']='Inspect the windowed panel and the shared lift crosspiece. All three sampled geometric cases are clear. A loaded single-point hang can still tip a bumper onto the floor.'
data['concepts']={'shared':variant}
def read(p):return json.loads((HERE/p).read_bytes())
d=read('park-rehearsal.json'); arm=mat(0,pivot_x(0),0,501)@inverse;sm=mat(0,pivot_x(0)-185,0,0)
frames={n:[(arm if n in moving else sm)[:3].ravel().tolist()]*25 for n in [*moving,*slide]}
new=[dict(id='PARK · final 24 seconds',stage='endgame',basis='481 peer samples, swept fixed-heading robot outside all five peer envelopes and stage legs. Starts at TELEOP 111 seconds.',gap=0,status='sampled',results=[],frames=frames,positions=d['viewPositionsMm'],removed=[held])]
holds=read('free-hold.json')
def quaternion(q):
    w,x,y,z=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],[2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],[2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
for angle in (-50,125):
    row=next(r for r in holds['cases'] if r['angleDeg']==angle and r['n']==96 and r['latchAndRoof'])
    frame=mat(angle,pivot_x(angle),0,501); initial=poses(cases[3],0 if angle==-50 else 1)
    mats={n:[initial[group(n)][:3].ravel().tolist()]*25 for n in G if n!=held and not np.allclose(initial[group(n)],identity)}
    path=[]
    for i in np.linspace(0,len(row['poses'])-1,25):
        p=row['poses'][round(i)];m=mat(0,*[1000*x for x in p[:3]]);m[:3,:3]=quaternion(p[3:]);path.append((frame@m@invseat)[:3].ravel().tolist())
    mats[held]=path
    new.append(dict(id=f'Free NOTE · gravity hold {angle}°',stage='power-off retention',basis='Independent 96-segment free ring, gravity, 0.5 ms integration, frictionless contacts. Roof and pin installed. No motion prescribed for NOTE.',gap=0,status='sampled',results=[],frames=mats,positions=[[965.2,5547.36]]*25,removed=[]))
data['cases']=[c for c in data['cases'] if c['id'] not in {c['id'] for c in new}]+new
data['notes']=[n for n in data['notes'] if not n.startswith(('Free ring:','PARK route:'))]+['Free ring: zero-friction holds at −50, 0, 45, 90 and 125° on two numerical refinements. The maximum shift with the pin and bridge was 26.43 mm. At the two extreme angles the original open geometry allowed more than 1 metre of travel. The physical foam NOTE and latch loads still require testing.',
f"PARK route: {d['routeS']:.2f} s of 24 s at the declared capped speed and acceleration, against all five recorded peers. The straight approach intersects two stage legs and B2. The waypoint solution finishes at the documented Blue parking pose. Stage overhead and field occupancy remain project assumptions."]
for row in data['narrativeReview']['objectives']:
    if 'PARK' in row['objective']:row['basis']='The empty robot reaches the published Blue parking pose in 10.53 s against the five recorded peers. Comparison to completing another delivery depends on a working pickup and shot.'
dump(HERE/'motion.json',data['cases']);data['motion']=sha(HERE/'motion.json')
dest=base.with_suffix('.next');dump(dest,data);dest.replace(base)
www=base.parent;js=www/'review.js';s=js.read_text(encoding='utf8')
if 'const conceptKey=' not in s:
    s=s.replace('data=await response.json();entries=',"""data=await response.json();
 const conceptKey=new URLSearchParams(location.search).get('concept'),variant=data.concepts?.[conceptKey];
 if(variant){const bodies={...data.bodies};for(const n of variant.remove)delete bodies[n];Object.assign(bodies,variant.add);
 data={...data,...variant,bodies};}
 const switcher=$('conceptSwitch');switcher.href=variant?'/':'/?concept=shared';switcher.textContent=variant?'Return to retained cassette':'View passive cover and shared lift';
 entries=""")
    s=s.replace("($('labels').checked||selected||marked.length)?labelRail(width):0","$('labels').checked?labelRail(width):0")
    tmp=js.with_suffix('.next');tmp.write_text(s,encoding='utf8');tmp.replace(js)
html=www/'index.html';s=html.read_text(encoding='utf8')
if 'id="conceptSwitch"' not in s:
    tmp=html.with_suffix('.next');tmp.write_text(s.replace('<b>Assembly layout</b>','<b>Assembly layout</b><a id="conceptSwitch" href="/?concept=shared">View passive cover and shared lift</a>'),encoding='utf8');tmp.replace(html)
ref=www/'presentation.md';s=ref.read_text(encoding='utf8')
if 'one callout margin' not in s:
    ref.write_text(s+'\n## Component callouts\n\nKeep names outside the assembly in one callout margin. Sort the labels and material anchors in the same vertical order. Route leaders through a common margin and approach every part horizontally. Page coincident anchors into separate sets. Orbit, pan and scrub with labels enabled and verify that every visible component can be reached and inspected. Turning labels off restores the full canvas.\n',encoding='utf8')

