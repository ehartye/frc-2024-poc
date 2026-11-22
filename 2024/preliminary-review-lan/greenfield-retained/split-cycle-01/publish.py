from pathlib import Path
import json,copy,hashlib
here=Path(__file__).resolve().parent;development=here.parent/'six-piece-02';web=here.parent.parent/'www'
def read(p):return json.loads(p.read_bytes())
project=read(web/'data.json')
fit=read(here/'package.json'); budget=read(here/'budget.json'); wheels=read(here/'four-wheel-duty.json')
assert fit['parent']==project['step'] and not fit['overlaps'] and budget['geometry']==fit['layout']
stock=read(here/'meshes.json')
why={'traction':('Transmit force to the NOTE.','Separate feed and launch surfaces let the launch belts reach speed before the NOTE advances.'),'retained launch drive':('Turn and support the launch pulley.','Relocating the earlier drive keeps the high-speed stage independent of the infeed.'),'pulley support':('Locate a return pulley and carry its radial load.','The forward idler mounts on the transverse cassette member to leave the nose roller clear.'),'feed drive':('Power or support the shared feed transmission.','One transverse shaft and two opposed gearboxes drive both feed belts from one controlled motor.')}
for n,v in stock.items():
    cat=fit['categories'][n];pu,ra=why[cat];material=('Elastomer' if 'tread' in n or 'belt' in n else 'Steel shaft envelope' if ('shaft' in n.lower() or 'axle' in n.lower()) else 'Motor assembly envelope' if n in ['Feed motor','Launch Belt motor · left','Launch Belt motor · right'] else 'Mechanical assembly allocation')
    v.update(category='electrical' if material.startswith('Motor') else 'mechanism',group='Metering drive' if n.startswith('Feed') else 'Launch drive',material=material,purpose=pu,rationale=ra,basis='Preliminary machined-stock or component envelope. Tooth geometry and selected purchased mass remain open.')
base=(development/'publish.py').read_text()
# Only borrow the viewer time/coordinates conversion. Derive credits and times from the actual new trace.
base=base[:base.index("res=dict(planHash")].replace("read(p/'plan.json')","read(development/'plan.json')").replace("tag='dev-'+name+'-96'","tag=attempt+'-'+name+'-'+str(nn)").replace("s=read(p/('indexed-'+tag+'.json'))","s=read(p/('split-ready-'+tag+'-8.05.json'))")
base=base.replace("    name=leg['id'];","    if one is not None and leg['id']!=one:continue\n    name=leg['id'];").replace("mark=np.array(leg['mark']);q=leg['yaw'];","q=leg['yaw'];mark=np.array(leg['mark'])+yaw(q)[:2,:2]@np.array(r['placedNoteOffset']);")
base=base.replace("p/('indexed-'+tag+'.json')","p/('split-ready-'+tag+'-8.05.json')")
cases=[];runs=[]
for attempt,nn,one in [('dev',96,None),('dev',48,None),('R02',96,'center-high'),('R01',96,'inner')]:
    env=dict(__file__=str(here/'publish.py'),development=development,attempt=attempt,nn=nn,one=one)
    exec(compile(base,'current mission animator','exec'),env)
    c=env['c'];full=one is None
    c.update(key='split-'+(str(nn) if full else attempt.lower()),id=(f'Six-NOTE route · {nn} segments' if full else ('High center rebound · R02' if attempt=='R02' else 'Inner wing recovery · R01')),basis='Floor NOTE with six free degrees of freedom. Short belts, feed shaft and launcher keep their carried speeds through handoff; rotor speed gates feeding. Chassis trajectory is commanded. Field mesh, gravity and counter detections determine the outcome. Case clock starts at this route or individual leg.')
    for n in fit['removed']:c['rig'].pop(n,None)
    for n in stock:c['rig'][n]='arm'
    if not full:
        count=env['count'];c['measure']=[v.replace('/6 scored','/1 scored') for v in c['measure']]
    cases.append(c)
    runs.append(dict(case=c['key'],seconds=env['clock'],scored=env['count'],total=len(env['report']),trace=env['report']))
(here/'route-review.json').write_text(json.dumps(runs,indent=2))
weak=min(read(f)['volts'] for f in here.glob('result*.json'))
power=[dict(case='Split feeder and launch belts / weak',stage='Score with feeder',batteryCondition='weak',openCircuitV=11.8,resistance=.027,requiredMinV=8,loadedBusV=[weak,11.071],batterySideA=[27,(11.8-weak)/.027],marginV=weak-8,status='pass',lines=[dict(name='Concurrent accessories',amps=[27,27],members=[]),dict(name='Feed plus two launch drives, battery side',amps=[0,(11.8-weak)/.027-27],members=[n for n in stock if stock[n]['category']=='electrical'])])]
for r in wheels['groups']:
    if r['gear']!=5.5:continue
    aux=r['loadedAuxA'];minV=r['minBusV'];power.append(dict(case=f"Loaded drive / {aux} A auxiliaries",stage='Return to speaker',batteryCondition='weak',openCircuitV=11.8,resistance=.027,requiredMinV=8,loadedBusV=[minV,11.8-.027*aux],batterySideA=[aux,(11.8-minV)/.027],marginV=minV-8,status='pass' if not r['failed'] else 'fail',lines=[dict(name='Concurrent mechanism and electrical loads',amps=[aux,aux],members=[]),dict(name='Four drive motors, battery side',amps=[0,(11.8-minV)/.027-aux],members=[n for n in project['bodies'] if 'Swerve drive motor' in n])]))
mass=copy.deepcopy(project['mass'])
for row in mass:
    row['members']=[n for n in row.get('members',[]) if n not in fit['removed']]
mass.append(dict(id='Split transmission, net added stock and electrical allowance',kg=budget['deltaKg'],members=list(stock)))
v=dict(remove=fit['removed'],add=stock,cases=cases,power=power,inspection=budget['inspectionKg'],running=budget['runningKg'],mass=mass,support={**project['support'],**{n:stock[n]['rationale'] for n in stock}},issue='Split feeder and launcher',status='Preliminary trial · held',focus='Both nominal routes scored six. The high-center variation rebounds to the field and the high inspection estimate is 0.305 kg overweight.',step=fit['layout'],unresolved=['High mass estimate','Repeatability at the speaker','Moving-stock clearance and service'],static=[],narrow=fit['parkedCandidates'])
project.setdefault('concepts',{})['split']=v
tmp=web/'split.next';tmp.write_text(json.dumps(project,separators=(',',':')),encoding='utf8');tmp.replace(web/'data.json')
print([(x['case'],round(x['seconds'],2),x['scored']) for x in runs])

