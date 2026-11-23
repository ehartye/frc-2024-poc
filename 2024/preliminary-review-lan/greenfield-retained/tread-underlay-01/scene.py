"""Bind scored field traces, unilateral chain contact and the latest editable preliminary layout in the LAN viewer."""
from pathlib import Path
import sys,json,math,csv,hashlib,copy,numpy as np
from scipy.spatial.transform import Rotation
p=Path(__file__).resolve().parent;root=p.parent;web=root.parent/'www';sf=root/'split-field-02'
def read(f):return json.loads(f.read_bytes())
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
doc=read(web/'data.json');old=doc['concepts']['split'];pkg=read(root/'split-cycle-01/package.json')
parts=copy.deepcopy(old['add']);gates=read(root/'keeper-drive-02/result.json');forward=read(root/'forward-drive-02/result.json');gas=read(root/'forward-drive-02/budget.json');tread=read(p/'result.json');sweep=read(root/'keeper-drive-02/sweep-result.json');approach=read(root/'keeper-drive-02/root-entry.json')
assert not sweep['positive'] and not sweep['errors'] and not approach['positiveChain'] and not approach['otherFieldCandidates']
assert gates['step']==sweep['parent']==approach['parent']==tread['parent'] and sha(p/'field-review.step')==tread['step']
remove=list(dict.fromkeys(old['remove']+gates['removed']))
def mesh(folder):
    for n,g in read(root/folder/'mesh.json').items():
        b=copy.deepcopy(parts.get(n,doc['bodies'].get(n,{})));b.update(g)
        if folder=='frame-pocket-01':b.update(basis='Machined 6061 stock. Continuous webs, pocket end radii, intact mounting lands and joint mass allowances are in the frame takeoff.')
        if n.startswith('Entry bearing bracket'):b.update(purpose='Carry the nose bearing and forward launch motor on a shared bracket.',rationale='The upper motor location clears the front bumper at pickup. The integrated web preserves the entry-bearing bore.')
        if folder=='keeper-drive-02':
            pu,ra={'Common keeper yoke and sliding rack':('Close both hook throats on one stroke.','A spring returns the connected slides to two lower pockets when opening torque is removed.'),'Keeper drive mast':('Carry the opening actuator reaction into the lift crosspiece.','The mast sits behind the chain during the capture stroke.'),'Keeper pinion output':('Transmit motor torque to the gate pinion.','One shaft drives the two gates through a common rack.'),'Keeper return pinion':('Turn the actuator rotation into 44 mm of gate travel.','The selected pitch radius sets the opening torque budget.'),'Keeper motor and return spring':('Open the hook gates and bias them closed.','The rack can close without holding motor torque.')}.get(n,('Carry one side of the chain load.','The throat and keeper pocket were sized on the measured installed field-chain section.'))
            b.update(category='electrical' if n.startswith('Keeper motor') else 'mechanism',group='Chain capture',purpose=pu,rationale=ra,material='Motor and spring envelope' if n.startswith('Keeper motor') else '6061 allocation' if n=='Keeper drive mast' else 'Steel',basis='Unilateral hook contact. Motor, spring and gear are preliminary envelopes.')
        if folder=='tread-underlay-01':b['basis']='The hard backing ends at radius 17.25 mm. The compliant tread extends to 24 mm. Actual compound and force curve remain open.'
        parts[n]=b
for folder in ['frame-pocket-01','forward-drive-02','keeper-drive-02','tread-underlay-01']:mesh(folder)
plan=read(sf/'plan.json');roster=list(csv.DictReader(Path('R:/sim/pressure/evidence/speaker-repeatability-starts.csv').open()))
def rigs(c,climbing=False):
    for n in remove:c['rig'].pop(n,None)
    for n in parts:
        if n.startswith(('Bay crossmember','Chassis sill','Chassis perimeter')):c['rig'][n]='frame'
        else:c['rig'][n]=('keeper' if climbing and n=='Common keeper yoke and sliding rack' else 'arm')
    return c
# Reuse the paired simulation's viewer transforms, with independent attempted starts on the original field.
template=(root/'six-piece-02/publish.py').read_text(encoding='utf8').split('res=dict(planHash')[0]
template=template.replace("assert d['step']==plan['placedStep'] and ","assert ")
template=template.replace("tag='dev-'+name+'-96'","tag=attempt+'-'+name+'-'+str(nn)").replace("('indexed-'+tag+'.json')","('split-ready-'+tag+'-6.88.json')")
template=template.replace("for idx,leg in enumerate(plan['plan']):","for idx,leg in enumerate(plan['plan'][:len(expected['parts'])]):")
template=template.replace("mark=np.array(leg['mark']);","mark=np.array(leg['mark'])+[float(obs.get(name+'_x_mm',0))/1000,float(obs.get(name+'_y_mm',0))/1000];")
template=template.replace("pre=1.15 if idx==0 else 3.4","""if idx==0:
        out[0]+=[float(obs.get('robot_x_mm',0))/1000,float(obs.get('robot_y_mm',0))/1000];L=np.linalg.norm(out[1]-out[0]);td=max(1.5*L/2.7,math.sqrt(6*L/2.6),math.sqrt(6*abs(math.radians(float(obs.get('robot_heading_deg',0))))/2.8));leg['outT']=[td]
    pre=1.15 if idx==0 else 3.4""").replace("P,scheduled(a),q,90*np.clip","P,scheduled(a),q+(math.radians(float(obs.get('robot_heading_deg',0)))*(1-min(1,t/td)**2*(3-2*min(1,t/td))) if idx==0 else 0),90*np.clip")
cases=[]
for attempt,nn in [('dev',96),('dev',48),('R01',96),('R02',96)]:
    expected=read(sf/f'attempt-{attempt}-{nn}.json');obs=next((x for x in roster if x['attempt']==attempt),{})
    env=dict(__file__=str(sf/'publish.py'),attempt=attempt,nn=nn,expected=expected,obs=obs);exec(compile(template,'measured route animation','exec'),env)
    assert abs(env['clock']-expected['elapsed'])<.001 and env['count']==expected['score']
    c=env['c'];k='field-'+(str(nn) if attempt=='dev' else attempt.lower())
    c.update(key=k,id=(f'Six-NOTE route · {nn} segments' if attempt=='dev' else 'Repeatability · '+attempt),basis='Independent floor pieces under gravity, finite belt forces, loaded chassis motion and free flights over the registered field. The simulator uses commanded arm and chassis motion. Whole opening passage and both counters determine the score.')
    cases.append(rigs(c))
def M(a=0,x=0,y=0,z=0):
    m=np.eye(4);m[:3,:3]=Rotation.from_euler('y',-a,degrees=True).as_matrix();m[:3,3]=[x,y,z];return m
def flat(m):return m[:3].round(7).flatten().tolist()
park=M(125,185,0,501);ip=np.linalg.inv(park);hinge=M(0,457.5,0,96)
source=read(root/'chain-contact-02/front48.json');assert source['passed'] and not any(source['enabledHookEqualities'])
c=copy.deepcopy(next(k for k in doc['cases'] if k['key']=='climb'));rigs(c,True);c.update(id='Chain entry, lift and power cutoff',key='field-climb',fixture='climb',positions=[],time=[],chain=[],measure=[],series={k:[] for k in ['frame','arm','pin','flap','piece','keeper']},basis='The approach uses measured chain sections. The loaded replay uses an articulated chain and CAD-derived hook contact surfaces. The chassis and chain move freely; the actuator and spring are estimated. Motor torque is zero for the five-second disabled hold.',results=[])
anchor=np.array([5753.154153941144,4104.32]);rest=np.array(source['trace'][0]['chain'])[::2]*1000
def add(t,xy,frame,angle,elev,chain,txt,dep=0.):
    a=frame@M(angle,185,0,501);poses=dict(frame=frame,arm=a@ip,pin=frame@M(0,0,-dep,0),flap=a@hinge@M(90*np.clip((angle-102)/18,0,1))@np.linalg.inv(hinge)@ip,piece=np.eye(4),keeper=a@M(0,0,0,elev)@ip)
    c['time'].append(t);c['positions'].append(list(xy));c['measure'].append(txt);c['chain'].append(chain.tolist())
    for n in c['series']:c['series'][n].append(flat(poses[n]))
# First 30 exact approach samples; dynamic simulation starts at seated entry with the gates open.
for i in range(30):
    a=approach['poses'][str(i)];add(i*.04,[a['x'],anchor[1]],np.eye(4),a['th'],44,rest,'Approach · both keepers open')
extra=1.20
for r,g in zip(source['trace'],source['gateTrace']):
    assert abs(r['t']-g[0])<.0001
    fr=np.eye(4);fr[:3,:3]=Rotation.from_quat([*r['q'][1:],r['q'][0]]).as_matrix();fr[:3,3]=np.array(r['p'])*1000
    t=r['t'];txt=(f"Disabled · {r['minFloorM']*1000:.0f} mm tire / bumper clearance · {r['floorN']:.1f} N floor load" if t>=8 else f"Load transfer · {r['torqueNm']:.0f} N·m motor · {r['catchNm']:.0f} N·m stop" if t>3 else 'Both gates seated · hoist under load' if t>.55 else 'Spring closing the common keeper')
    dep=10.8*np.clip((t-source['commandTime']-.03)*source['assumedStrokeRate']/10.8,0,1) if source['commandTime'] is not None else 0
    add(extra+t,anchor,fr,r['angle'],g[1]*1000,np.array(r['chain'])[::2]*1000,txt,dep)
c['results']=[dict(pair=['Chain hooks','Installed field chain'],status='sampled',witness={'reason':f"{approach['exactChainChecks']} CAD checks during entry. At disable both sides carry load; minimum tire/bumper clearance {source['minFloorMm']:.1f} mm."}),dict(pair=['Open gate','Lift permission'],status='sampled',witness={'reason':'Holding the yoke open prevents the lift. No hook-to-chain joints are active.'})];cases.append(c)
parkCase=rigs(copy.deepcopy(next(k for k in doc['cases'] if k['key']=='park-route')))
parkCase.update(key='field-park',id='PARK fallback route',basis=parkCase['basis']+' Earlier field route with the arm lowered. Clearance with the latest keeper and motor brackets remains to be rechecked.')
cases.append(parkCase)
new=copy.deepcopy(old);new.update(add=parts,remove=remove,cases=cases,issue='Shared cassette · field review',step=tread['step'],status='Preliminary design · in review',static=[],unresolved=[],narrow=sweep['kernelComparisons'],notes=['Mesh and beam estimates use the latest staged geometry. Grip, wear, deflection and assembly tolerances require physical measurements.'])
new['support'].update({n:b['rationale'] for n,b in parts.items()})
extraGate=np.array(gates['extraIncludingActuatorKg']);ins=np.array(gas['inspection'])+extraGate+[tread['netNominalKg'],0.];run=np.array(gas['running'])+extraGate+[tread['netNominalKg'],0.]
new['inspection']=ins.tolist();new['running']=run.tolist()
for r in new['mass']:
    if r['id']=='Chassis':r['kg']=gas['chassisRange']
new['mass'] += [dict(id='Forward drive bridges and shafts',kg=gas['gripAndWireIncrementKg'],members=forward['modified']),dict(id='Common keeper and return drive',kg=extraGate.tolist(),members=list(gates['rig'])),dict(id='Backing and tread material change',kg=[tread['netNominalKg'],0.],members=tread['changed'])]
new['focus']='Check scoring repeatability, rigid-backing clearance and the loaded chain hold. The held-open keeper inhibits lift.'
new['narrativeReview']=copy.deepcopy(doc['narrativeReview'])
for n in new['narrativeReview']['objectives']:
    if 'six distinct' in n['objective']:n.update(disposition='Repeatability under review',basis='Both nominal resolutions scored six. R01 scored five. The remaining frozen setups determine the gate.')
    if 'ONSTAGE' in n['objective']:n.update(disposition='Contact and disable checked',basis='The registered chain approach and free chassis lift are shown in Chain entry, lift and power cutoff. Fastener loads, tolerances and physical verification remain open.')
new['power']=copy.deepcopy(old['power'])
for pow in new['power']:
    if pow['case'].startswith('Split feeder'):
        vals=[part['bus'] for n in ('dev-48','dev-96','R01-96','R02-96') for part in read(sf/('attempt-'+n+'.json'))['parts'] if 'bus' in part]
        vv=min(vals);pow['loadedBusV'][0]=vv;pow['batterySideA'][1]=(11.8-vv)/.027;pow['marginV']=vv-8;pow['lines'][1]['amps'][1]=pow['batterySideA'][1]-27
new['validationText']=f"{sweep['kernelComparisons']:,} narrow-phase geometry comparisons over 122 sampled robot poses. {approach['exactChainChecks']} field-chain entry comparisons. The live contact model and its held-open control cover the climb. Continuous clearance between sampled poses remains open."
duty=read(p/'four-wheel-duty.json')
for row in new['power']:
    if row['case'].startswith('Loaded drive'):
        aux=row['lines'][0]['amps'][0];case=next(r for r in duty['groups'] if r['gear']==5.5 and r['loadedAuxA']==aux)
        bus=case['minBusV'];amps=(11.8-bus)/.027
        row.update(loadedBusV=[bus,11.8-.027*aux],batterySideA=[aux,amps],marginV=bus-8,status='pass' if not case['failed'] else 'hold')
        row['lines'][1]['amps']=[0,amps-aux]
new['required']=copy.deepcopy(doc['required'])
for row in new['required']:
    if 'piece' in row[0].lower():row[1]=[n for n,b in parts.items() if b['category']=='electrical' and n.startswith(('Launch Belt motor','Feed motor'))]+['Nose motor']
new['defaultPickupBlade']={**doc['defaultPickupBlade'],'assembly':tread['step']}
doc.setdefault('concepts',{})['field']=new
tmp=web/'field.next';tmp.write_text(json.dumps(doc,separators=(',',':')),encoding='utf8');tmp.replace(web/'data.json')
(p/'review.json').write_text(json.dumps(dict(step=tread['step'],cases=[k['key'] for k in cases],highMarginKg=56.69904625-ins[1],inspection=ins.tolist(),running=run.tolist(),sweep=sweep['kernelComparisons'],entry=approach['exactChainChecks'],sourceBoundDisabled=source['minFloorMm']),indent=2))
print(ins,run)

