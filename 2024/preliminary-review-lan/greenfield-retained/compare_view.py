"""Build an alternate selectable rough layout while retaining the issued baseline."""
from travel import *
from component_intent import explain
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
base=HERE.parent/'www'/'data.json';data=json.loads(base.read_bytes())
assert data['step']==sha(HERE/'retained-cassette.step')
remove=[n for n in G if meta[n]['group'] in ['Capture latch','Solo climber']]
for n in remove:
    del G[n];del meta[n]
    if n in moving:moving.remove(n)
new=[]
def article(n,sh,g,pur,why,material='6061 aluminum'):
    local(n,sh,g,'mechanism',material);specify(n,pur,why);new.append(n)
for y,side in [(218,'left'),(-226,'right')]:
    for s in (95,330):
        article(f'Lid post {s} · '+side,box(s,y,14,12,8,62),'Passive cover','Support the keeper rails from the side beam.','The post carries NOTE contact into the cassette instead of the thin panel edge.')
    # Angle shelf laps the underside of the panel.
    ledge=box(36,208 if side=='left' else -218,76,372,10,4)
    sidewall=box(36,y,76,372,8,12)
    rail=cut(fuse(cut(sidewall,box(172,y,75,16,8,15)),ledge),cyl((399.5,y,84),Y,2,8))
    article('Lid edge rail · '+side,rail,'Passive cover','Seat the panel and support the flap axle.','The ledge bears under the cover along its edge and the upright wall carries the pivot.')
    article('Climb yoke post · '+side,box(172,y,14,16,8,166),'Shared cassette lift','Carry the chain reaction into the cassette side beam.','Places the crosspiece near the main pivot to limit required torque.')
panel=cut(box(36,-218,80,372,436,2),box(60,-45,79,349,90,5))
article('Windowed keeper panel',panel,'Passive cover','Keep the NOTE on the runners when inverted.','The center window leaves the article visible and the mouth accessible.','2 mm polycarbonate allocation')
article('Flap axle',cyl((399.5,-226,84),Y,2,452),'Passive cover','Mount the retention flap across the cassette.','Both upper rails carry the small axle.','Steel')
flap=fuse(box(398,-37,32,3,74,52),cyl((399.5,-37,84),Y,3,74))
flap=cut(flap,cyl((399.5,-37,84),Y,2,74))
article('Mouth retention flap',flap,'Passive cover','Catch the ring if it moves toward the open mouth.','Fold inward on admission; park above the rim for launch. The return spring, stop and release cam need design.','Polymer flap with sleeve')
article('Shared lift crosspiece',box(172,-226,180,16,452,12),'Shared cassette lift','Provide a mounting point for a swiveling, gated chain hook.','The existing cassette provides reach; the new crosspiece carries load across both sides. Hook, brake and reinforcement remain to be sized.')
AAB.clear();AAB.update({n:bounds(sh) for n,sh in G.items()});pcheck=parked_check()
# Run the matching release with the actual flap and windowed panel.
tor=place(torus,mat(0,215,0,50.4));normalSweep=place(fuse(fuse(torus,ann((0,0,0),Z,177.8,127,80)),place(torus,mat(0,0,0,80))),mat(0,215,0,50.4))
props=GProp_GProps();BRepGProp.VolumeProperties_s(panel,props)
pulled=place(tor,mat(0,50,0,0));hinge=mat(0,399.5,0,84);opened=place(flap,hinge@mat(90,0,0,0)@np.linalg.inv(hinge))
results=dict(parked=pcheck,normalBlockMm3=common(panel,normalSweep),panelMassKg=props.Mass()*1.2e-6,
 seatedPanelClearanceMm=distance(tor,panel),closedFlap50mmCommon=common(pulled,flap),openFlap50mmCommon=common(pulled,opened),openFlapClearanceMm=distance(pulled,opened))
# Selected secondary view: only phased changes have frames. Full conflict reporting,
# including stock across the enlarged swing range.
motion=[]; mb=mat(park,*pivot)
for mode in ['Inspect cover and shared lift','Open mouth for discharge','Empty cassette · chain take-up']:
    rows={};frames={};narrow=0;seen=set();unresolved=[]
    for i in range(25):
        t=i/24;angle=park if mode!='Empty cassette · chain take-up' else 70.43494+(165-70.43494)*t
        arm=mat(angle,pivot_x(angle),0,501)@inverse;sm=mat(0,pivot_x(angle)-185,0,0)
        fp=arm@mb@hinge@mat(90*t if mode=='Open mouth for discharge' else 0,0,0,0)@np.linalg.inv(hinge)@inverse
        matrices={n:(fp if n=='Mouth retention flap' else arm if n in moving or n==held else sm if n in slide else identity) for n in G}
        valid=[n for n in G if n!=held or mode!='Empty cassette · chain take-up']
        for n in valid:frames.setdefault(n,[]).append(matrices[n][:3].ravel().tolist())
        groups={n:('flap' if n=='Mouth retention flap' else 'arm' if n in moving or n==held else 'slide' if n in slide else 'fixed') for n in valid}
        boxes={n:transformed_bounds(AAB[n],matrices[n]) for n in valid};cache={}
        def shaped(n):
            if n not in cache:cache[n]=posegeom(G[n],matrices[n])
            return cache[n]
        for a,b in itertools.combinations(valid,2):
            if groups[a]==groups[b] or not close(boxes[a],boxes[b]):continue
            k=(a,b,tuple(np.round((np.linalg.inv(matrices[a])@matrices[b])[:3].ravel(),6)))
            if k in seen:continue
            seen.add(k)
            if not close(bounds(shaped(a)),bounds(shaped(b))):continue
            narrow+=1
            try:v=common(shaped(a),shaped(b))
            except Exception as e:unresolved.append(dict(pair=[a,b],status='open',witness=dict(t=t,reason=str(e))));continue
            if v>.001 and v>rows.get((a,b),{}).get('volumeMm3',0):rows[a,b]=dict(t=t,volumeMm3=v)
    found=[dict(pair=list(k),status='fail',witness=v) for k,v in rows.items()]+unresolved
    motion.append(dict(id=mode,stage='architecture comparison',basis='25 registered poses. Empty lift illustrates load-point travel; the field chain and a supporting hook are not installed.',gap=0,status='fail' if found else 'sampled',results=found,frames={n:f for n,f in frames.items() if not np.allclose(f,identity[:3].ravel())},positions=[[965.2,5547.36]]*25,removed=[held] if mode=='Empty cassette · chain take-up' else [],narrow=narrow))
view=HERE/'shared-cassette-comparison.step';write_step(view);results['step']=sha(view);results['motionResults']=[dict(case=c['id'],narrow=c['narrow'],results=c['results']) for c in motion]
dump(HERE/'shared-view-check.json',results)
load=json.loads((HERE/'shared-arm.json').read_bytes())
additions={n:dict(**viewmesh(G[n]),**meta[n],**intent[n],basis='Preliminary alternative on the common chassis datum.') for n in new}
mass=[dict(row) for row in data['mass'] if row['id'] not in ['Capture latch','Solo climber']]
mass.extend([dict(id='Passive cover',kg=[.62,1.2],members=[n for n in new if meta[n]['group']=='Passive cover']),
             dict(id='Shared cassette lift',kg=[1.4,3.8],members=[n for n in new if meta[n]['group']=='Shared cassette lift'])])
ins=np.array([r['kg'] for r in mass]).sum(axis=0)
power=[]
for row in data['power']:
    lines=[dict(x) for x in row['lines'] if x['name'] not in ('Latch coil','Solo climber')]
    if row['stage']=='Climb with shooter idle':
        for line in lines:
            if line['name']=='Pivot and slide':line['amps']=[20,65]
    lo,hi=np.sum([x['amps'] for x in lines],axis=0);v=row['openCircuitV'];r=row['resistance'];vmin=v-r*hi
    power.append(dict(row,lines=lines,batterySideA=[int(lo),int(hi)],loadedBusV=[vmin,v-r*lo],marginV=vmin-8,status='pass' if vmin>=8 else 'fail'))
variant=dict(issue='P3 comparison · Passive cover and shared lift',remove=remove,add=additions,step=sha(view),receipt=sha(HERE/'shared-view-check.json'),cases=motion,mass=mass,inspection=ins.tolist(),running=(ins+np.array([r['kg'] for r in data['details']]).sum(axis=0)).tolist(),power=power,status='Comparison · load path open',focus='The empty cassette swings to take chain load. Inspect the upper crosspiece, panel and flap. A gated hook, positive hold and load-rated shoulder remain open.',**pcheck,
    notes=[f"Windowed polycarbonate panel alone: {results['panelMassKg']:.3f} kg at planning density, {results['seatedPanelClearanceMm']:.1f} mm above the seated NOTE. The cover arrests outward lift but leaves an open exit along the bed. A passive flap is a separate mechanism.",
    f"Two posts connect the climb crosspiece to the cassette beams. At the 68.56 kg comparison load the peak static torque from robot weight alone is {load['peakStaticNm']:.1f} N·m. These are geometric screens before purchasing a drive, brake and actual hook.",
    'The shared-lift mass and current figures are allocations. Eight CG corners were screened; loaded chain movement and intermediate CG values remain open.'],
    required=[row for row in data['required'] if row[0] not in ('Retained cassette','Endgame package')]+[['Passive keeper',['Windowed keeper panel','Mouth retention flap','Flap axle']],['Shared lift support',['Climb yoke post · left','Climb yoke post · right','Shared lift crosspiece']]],
    narrativeReview=dict(data['narrativeReview'],objectives=[dict(r,basis='Empty cassette reaches a trial point on the chain. Gated hook, loaded hold and installed drive are open.') if 'ONSTAGE' in r['objective'] else r for r in data['narrativeReview']['objectives']]))
dump(HERE/'comparison-view.json',variant);print(results,flush=True)

