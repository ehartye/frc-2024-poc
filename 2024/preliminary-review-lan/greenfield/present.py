from travel import *
field=Path('C:/Users/ehart/Documents/FRC Native/2024/six-speaker-20261118/field-source/blue-speaker/opening-and-counter.json')
target=(1000*np.array(json.loads(field.read_bytes())['nominalOpeningCornersM'])).tolist()
receipts=json.loads((HERE/'travel.json').read_bytes());parked=json.loads((HERE/'parked.json').read_bytes())
view=HERE/'pivot-cassette.step'
if not view.exists():write_step(view)
model={n:dict(**viewmesh(s),**meta[n],basis='Nominal occupied geometry of the preliminary article') for n,s in G.items()}
allocation={'Chassis':(5,7),'Pivot support':(2.4,4.2),'Pivoting cassette':(1.2,2.3),'Piece support':(.7,1.4),'Cassette drive':(2.1,4.2),'Electrical bay':(3.6,5.9)}
mass=[dict(id=g,kg=kg,members=[n for n in G if meta[n]['group']==g and n!='Battery and terminal case']) for g,kg in allocation.items()]
for g in sorted(set(m['group'] for m in meta.values() if m['group'].startswith('Swerve'))):
    mass.append(dict(id=g,kg=[2.4,3.1],members=[n for n in G if meta[n]['group']==g]))
mass+= [dict(id='Unplaced fasteners, harness and sensors',kg=[2.0,4.2],members=[]),dict(id='Unallocated design reserve',kg=[3.,6.],members=[])]
detail=[dict(name=n,kg=k,members=[v]) for n,k,v in [('battery',[5.5,6.2],'Battery and terminal case'),('one NOTE',[.23,.25],held),('finished bumper scenario',[4.6,6.6],'Blue bumper set')]]
inspection=np.array([m['kg'] for m in mass]).sum(axis=0); running=inspection+np.array([r['kg'] for r in detail]).sum(axis=0)
load_ids={'Drive and steering':[n for n in G if n.startswith('Swerve ')],'Pivot':['Pivot motor'],'Traction belts':[n for n in G if n.startswith('Belt motor ·')],'Pickup roller':['Nose motor'],'Computers and control':['roboRIO flat','radio','PDH']}
power=[]
for stage,amps in [('Aim and spin up',[(8,22),(5,15),(30,70),(0,2),(4,8)]),('Intake',[(16,42),(6,24),(8,26),(4,16),(4,8)]),('Concurrent acceleration and spinup',[(40,90),(8,28),(30,70),(4,16),(4,8)])]:
    for cond,v,r in [('weak',11.8,.027),('nominal',12.6,.018)]:
        lo,hi=np.array(amps).sum(axis=0).tolist();vmin=v-r*hi
        power.append(dict(case=stage+' / '+cond,stage=stage,batteryCondition=cond,openCircuitV=v,resistance=r,requiredMinV=8,loadedBusV=[vmin,v-r*lo],batterySideA=[lo,hi],marginV=vmin-8,status='pass' if vmin>=8 else 'fail',lines=[dict(name=k,amps=a,members=load_ids[k]) for k,a in zip(load_ids,amps)]))
rendercases=[]
for case in cases:
    rc=receipts[case]; frames={}
    for n in G:
        matrices=[poses(case,t/24)[group(n)][:3].ravel().tolist() for t in range(25)]
        if not np.allclose(matrices,identity[:3].ravel()):frames[n]=matrices
    rendercases.append(dict(id=case,stage='service' if case==cases[4] else 'operating',basis='25 rigid poses. Ring tipping is a prescribed, two-surface geometric path.',gap=0,status='fail' if rc['results'] else 'sampled',results=rc['results'],frames=frames,removed=['Pack lead','Distribution lead'] if case==cases[4] else [],positions=[[965.2,5547.36]]*25))
identity_control=corridor()
path=HERE/'motion.json';dump(path,rendercases)
# Extent from actual stock at a known intermediate angle provides an independent failing witness.
witness_shape=G['Entry bearing bracket · left'];horizontal=mat(0,*pivot)@inverse
tip=bounds(place(witness_shape,horizontal))[3];limit=340+304.8
notes=[f'Greenfield P1.2 · one pivoting, reversible belt cassette · {len(G)} solids. Launch and floor capture share a channel. The two A-frames are outboard of the NOTE throughout its centered path. Bearings, axles, pulleys, motor envelopes and runners are shown.',
f'REJECT full-length rigid swing. At horizontal the entry bearing bracket reaches robot X {tip:.1f} mm, {(tip-limit):.1f} mm past the 304.8 mm forward extension allowance measured from the X340 frame. Parked fit alone missed this.',
'Opening and nominal foam torus use official field-source geometry. Floor raster is 250 mm. The drive pose is a fixed planning datum. Animations prescribe the NOTE orientation. Foam deformation, traction, motor response and any scored event have yet to be demonstrated.',
'Five battery-pull clashes at the aiming pose. Leads are disconnected in this case. Independent current allocations are engineering scenarios, pending motor selection, inertia and current measurements. Mass bands are provisional subsystem allowances; the aggregate includes unplaced integration reserve.',
f'Continuous lateral separation of each A-frame from the centered NOTE is at least {identity_control["minLateralGapMm"]:.1f} mm. A deliberately offset NOTE intersects the actual left A-frame by {identity_control["controlMm3"]:.1f} mm³. All other motion results apply to sampled poses unless identified separately.']
data=dict(issue='Greenfield P1.2 · Swing-through belt cassette',step=sha(view),receipt=sha(HERE/'travel.json'),motion=sha(path),bodies=model,cases=rendercases,palette=dict(structure='#86A9C5',mechanism='#18BFB0',electrical='#B493FF',bumper='#3984F6',**{'reserved-space':'#F68448','game-piece':'#FFC84D'}),mass=mass,details=detail,power=power,inspection=inspection.tolist(),running=running.tolist(),status='Held · extension limit',corners=target,support={},limits={'inspectionKg':56.69904625},**parked,notes=notes,focus='Horizontal sweep exceeds front extension by at least '+str(round(tip-limit))+' mm. Select Deploy to floor to inspect the swing.',required=[['Four wheels',[n for n in G if n.endswith(' tire')]],['Eight swerve motors',load_ids['Drive and steering']],['Control and distribution',['roboRIO flat','PDH','radio','main breaker','Battery and terminal case']],['Two supported A-frames',['Pivot A-frame · left','Pivot A-frame · right','Pivot torque shaft','Pivot bearing · left','Pivot bearing · right']],['Powered piece handling',load_ids['Traction belts']+load_ids['Pickup roller']]])
out=HERE.parent/'www'/'data.tmp';dump(out,data);out.replace(out.with_name('data.json'))
print(data['issue'],len(G),'closed',sum(b['closed'] for b in model.values()),data['inspection'],tip-limit,flush=True)

