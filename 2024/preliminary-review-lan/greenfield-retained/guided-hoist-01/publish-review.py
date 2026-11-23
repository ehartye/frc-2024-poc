"""Publish the preliminary hoist and solved free-chassis trials to the existing LAN review."""
from pathlib import Path
import numpy as np,copy,json,math
p=Path(__file__).resolve().parent;root=p.parent
read=lambda f:json.loads(f.read_bytes())
web=root.parent/'www/data.json'
doc=read(web);field=doc['concepts']['field'];original=next(x for x in field['cases'] if x['key']=='field-climb');end=root/'endgame-route-01'
geometry=read(p/'inboard.json');mesh=read(p/'inboard-mesh.json');budget=read(p/'return-budget.json')
mass=budget['mass'];baseline={**doc['bodies'],**field['add']}
# Human-readable functions correspond to the preview's selectable solids.
guide={
'Shared lift crosspiece':('Tie both hook shoes together.','The two hooks must follow the same travel.'),
'Hook guide channel':('Guide the captured hook carriage along the cassette.','A straight pull clears the fixed field rail during the lift.'),
'Captive hook wear block':('Keep the shoe captured in its channel.','Bearing on opposed faces controls twist under an uneven chain load.'),
'Guide rear anchor':('Attach the channel to the cassette structure.','The hook reaction must reach both side beams.'),
'Pivot cable drum':('Wind one hoist rope on the pivot shaft.','Paired drums reuse the existing pivot motor for the climb.'),
'Hoist redirect ear':('Hold the rear redirect axle.','The load turns around the back of each cassette rail.'),
'Hoist rear redirect':('Turn the hoist line toward the sliding carriage.','The inboard location clears the power lead through cassette rotation.'),
'Catch sector':('Index the cassette at chain capture.','The pivot must hold pitch independently while its drive winds the cable.'),
'Capture-angle dog bushing':('Support the pitch-lock pin.','A replaceable bearing surface carries the local sector load.'),
'Hook slider liner':('Provide a sliding surface between channel and hook shoe.','The wear surface can be replaced without remaking the rail.'),
'Slide stop':('Secure the loaded carriage at full retract.','A positive stop supports the hang after battery power is removed.'),
'Hoist rope':('Transfer drum pull to the captured hook.','Two lines share the crosspiece. The visible span represents the tensioned centerline.'),
'Pivot torque shaft':('Deliver pivot motor torque to both drums.','The shaft continues to turn while the cassette is locked.'),
'Pivot drive socket':('Drive the cassette during scoring.','The socket separates from the shaft clutch for hoisting.'),
'Sliding pivot clutch':('Switch the shared shaft between pivot and winch duty.','The 4.2 mm release frees the shaft while the pitch dog holds the arm.'),
'Cassette cheek':('Carry the pivot socket and cassette reactions.','The relieved cheek clears the clutch stroke.'),
'Redirect cross-tie':('Connect the inboard redirect to its channel.','Both cable routing and battery service depend on where the redirect is supported.'),
'Hook anchor compression tube':('Transmit the forward eye load to the crosspiece.','The forward attachment leaves room for the splice through full travel.'),
'Hook anchor root':('Seat the compression tube on the moving structure.','Keeps the cable attachment tied to the hook carriage.'),
'Thimble pin shelf':('Support the removable eye pin.','A replaceable end termination needs a physical bearing seat.'),
'Hoist thimble pin':('Retain the rope eye on the shelf.','The removable pin permits fitting and inspecting the splice.'),
'Hoist eye thimble':('Support the bend in the loaded rope eye.','A 27 mm bearing surface limits curvature in the termination.'),
'Spliced hoist eye':('Loop the 3 mm rope around the thimble.','The manufacturer-rated spliced construction needs enough flat eye length.'),
'Hoist splice allowance':('Reserve the 180 mm splice bury.','The thickened splice must stay out of the rear sheave throughout travel.')}
added={};details=list(doc['details'])
for n,b in mesh.items():
    row=geometry['rows'][n];purpose,why=next(val for k,val in guide.items() if n.startswith(k))
    mat={970:'UHMWPE rope',1420:'Acetal wear stock',7850:'Steel',2700:'Aluminum alloy'}.get(row['densityKgM3'],'Planning stock, '+str(row['densityKgM3'])+' kg/m³')
    added[n]=dict(b,material=mat,group='Guided chain hoist',category='electrical' if 'actuator' in n else 'structure' if any(x in n for x in ('channel','compression','web','cross-tie','cheek','crosspiece')) else 'mechanism',purpose=purpose,rationale=why,basis='Reopened dimensioned preliminary STEP. Occupied stock estimate '+format(row['kg']*1000,'.1f')+' g. Purchased hardware and shop allowances are listed in the hoist budget.')
    details.append(dict(name=n+' · modeled stock',kg=[row['kg']]*2,members=[n]))
source=read(end/'original-climb.json');entry=original['time'][-1]-source['time'][-1]+source['time'][30];anchor=source['positions'][30]
def M(a=0,x=0,y=0,z=0):
    t=math.radians(a);c=math.cos(t);s=math.sin(t);return np.array([[c,0,-s,x],[0,1,0,y],[s,0,c,z],[0,0,0,1.]])
def rigid(q):
    w,x,y,z=q;return [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
def flat(m):return m[:3].round(8).ravel().tolist()
ip=np.linalg.inv(M(125,185,0,501));hooks=read(end/'full-hooks.json')['hookGroup']
cases=[];power=[];inherit=['field-96','field-48','field-r01','field-r08','field-seat-recovery','field-park']
for hand in ['front','rear']:
    row=read(p/('return-'+hand+'.json'));trial=budget['trials'][hand];c=copy.deepcopy(original)
    title=('Forward' if hand=='front' else 'Rearward')+' CG · guided climb and power cut'
    c.update(id=title,key='guided-'+hand,stage='physics playback',status='review',basis='230 mm travel. Both hooks contact the free chain and the robot lifts under motor torque. The pin and pitch lock are ideal constraints. Drum take-up assumes 0.30 N·m total passive spring torque. Pin and clutch engagement timing before reeling is illustrative. Rope rendering shows the tensioned path.',results=[],floorCheckMm=trial['floorMm'])
    ix=next(i for i,t in enumerate(c['time']) if t>=entry-.000001)
    for n in ['time','positions','chain','measure']:c[n]=c[n][:ix]
    for n in c['series']:c['series'][n]=c['series'][n][:ix]
    for n in ['carriage','clutch','drum','stopLeft','stopRight','cord']:c['series'][n]=list(c['series']['arm'])
    for n in hooks:
        if n!='Common keeper yoke and sliding rack':c['rig'][n]='carriage'
    for n,r in geometry['role'].items():c['rig'][n]=('stopLeft' if n.endswith('left') else 'stopRight') if r=='stop' else 'arm' if r=='cassette' else r
    for r in row['trace']:
        t=r['t'];mm=1000*r['draw'];fr=np.eye(4);fr[:3,:3]=rigid(r['q']);fr[:3,3]=np.array(r['p'])*1000
        arm=fr@M(r['angle'],185,0,501);slide=arm@M(0,-mm);drum=M(math.degrees(r['coil']/.0295))
        factor=(245-mm)/245;cord=np.eye(4);cord[0,0]=factor;cord[0,3]=-131*(1-factor)
        stop=8 if r['latchedNow'] else 0
        transforms=dict(frame=fr,arm=arm@ip,pin=fr@M(0,0,-10.8*min(t/.13,1),0),piece=np.eye(4),flap=arm@ip,keeper=slide@M(0,0,0,1000*r['gate'])@ip,carriage=slide@ip,drum=arm@drum@ip,clutch=arm@M(0,0,-4.2*np.clip((t-.15)/.20,0,1),0)@drum@ip,stopLeft=arm@M(0,0,-stop,0)@ip,stopRight=arm@M(0,0,stop,0)@ip,cord=arm@cord@ip)
        for n in c['series']:c['series'][n].append(flat(transforms[n]))
        c['time'].append(entry+t);c['positions'].append(anchor);c['chain'].append((np.array(r['chain'])[::2]*1000).round(3).tolist())
        state='Power cut · mechanical stop' if t>=row['cutS'] else 'Seated on stop' if r['latchedNow'] else 'Reeling' if t>=.45 else 'Capture and lock'
        c['measure'].append(f"{state} · {mm:.0f} mm travel · {r['ground']:.1f} N floor reaction · {1000*r['z']:.1f} mm clearance")
    for status,pair,reason in [('pass',['Tires and bumper','Carpet'],f"Minimum {trial['floorMm']:.2f} mm during five seconds disabled, against 10 mm required. Floor reaction stayed at zero."),('open',['Rope eye','Hook carriage'],'Installed splice, drum dead end, paired line tension and long-term bend strength require a loaded trial.'),('fail',['Battery withdrawal at 80°','Right hook guide'],'The existing service index obstructs a 370 mm vertical battery pull. The −40° alternative clears, but needs a positive index before use.')]:
        c['results'].append(dict(pair=pair,status=status,witness=dict(reason=reason)))
    cases.append(c)
    power.append(dict(case=title+' / weak',stage=title,batteryCondition='weak',openCircuitV=11.8,resistance=.027,requiredMinV=8,loadedBusV=[row['weakBusMin'],11.125],batterySideA=[25,row['peakBatteryA']],marginV=row['weakBusMin']-8,status='pass',lines=[dict(name='Controls and concurrent loads',amps=[25,25],members=[]),dict(name='Shared pivot motor during retraction',amps=[0,row['peakBatteryA']-25],members=['Pivot motor','Pivot reducer'])]))
roster=[r for r in field['mass'] if r['id'] not in mass['replacedBudget']]
roster.append(dict(id='Guided climb, keeper and take-up',kg=mass['packageRangeKg'],members=sorted(set(added)|set(hooks)|set(mass['retainedBodyKg']))))
narr=copy.deepcopy(field['narrativeReview'])
for r in narr['objectives']:
    if 'ONSTAGE' in r['objective']:r.update(disposition='Both CG trials held after power cut',basis='The 230 mm guided pull held for five seconds with at least 15.0 mm floor clearance. Battery service, mechanical load and paired cable installation remain open.')
out=dict(inherit=inherit,add=added,cases=cases,details=details,mass=roster,power=field['power']+power,narrativeReview=narr,inspection=mass['inspectionRangeKg'],running=mass['runningRangeKg'],issue='Shared cassette · guided climb',status='Preliminary design held · scoring repeatability and service',step=geometry['whole'],focus='Inspect the guided climb under both CG conditions. Profile A remains the default pickup edge.',validationText='232 occupied solids. Eight internal hoist states and 100 chassis poses screened. Five free-NOTE traces checked. Battery withdrawal at 80° interferes with the right guide. Physical splice, assembly tolerance and strength are open.',notes=['Two chain hooks capture before the shared pivot motor reels. After both slide stops seat, the drive unloads the cable and a passive take-up keeps it tensioned. The spring package is a purchasing allowance.','Scoring replays preserve the original free-piece simulation. The revised mass and inertia still need a matched scoring campaign. Frozen scoring results: six passes, two failures, twelve unrun.','The 55.963 kg upper inspection estimate includes 6 kg of general reserve and 0.20 kg of take-up hardware.'])
doc['concepts']['guided']=out
tmp=web.with_suffix('.guided-next');tmp.write_text(json.dumps(doc,separators=(',',':')),encoding='utf8');tmp.replace(web)
print(len(added),len(cases),web.stat().st_size)
