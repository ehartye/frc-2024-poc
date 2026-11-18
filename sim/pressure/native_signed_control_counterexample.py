"""Counterfactual replay: clip negative pinch duties from the attained native index."""
import argparse,copy,hashlib,json,pathlib,sys
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
p=argparse.ArgumentParser()
for n in ['project','binding','trajectory','output']:p.add_argument('--'+n,type=pathlib.Path,required=True)
p.add_argument('--dt',type=float,required=True);a=p.parse_args();assert a.dt in [.0005,.00025] and not a.output.exists();sys.path[:0]=[str(a.project/'sim/pressure'),str(a.project/'sim/integrated')]
from native_signed_pressure import NativeSignedPressure
from speed_approach import observe
load=lambda p:json.loads(p.read_bytes());source=load(a.trajectory);initial=source['index'];assert initial;binding=load(a.binding);dc=load(a.project/'sim/drivetrain/config.json');nc=load(a.project/'sim/acquisition/free-planar-config.json')['model'];cad=load(a.project/'cad/assembly/intake_components.json');model=NativeSignedPressure(dc,nc,cad,binding)
model.drive.state=copy.deepcopy(initial['chassis']);model.note=copy.deepcopy(initial['note']);model.energy=copy.deepcopy(initial['ledger']);model.initial_energy=initial['initialEnergyJ'];model.t=initial['timeS'];events=[x for x in source['events'] if x['phase']=='transport'];assert abs(events[0]['timeS']-model.t)<1e-9;index=0;rows=[];outcome='HORIZON'
for step in range(round(.5/a.dt)):
 while index+1<len(events) and events[index+1]['timeS']<=model.t+1e-9:index+=1
 command=copy.deepcopy(events[index]['command']);command['pinchDemands']=[max(0.,x) for x in command['pinchDemands']]
 row=model.step(a.dt,command['driveDemands'],command['pinchDemands'],[dc['massKg']*9.80665/2]*2,modes=command['modes']);rows.append(row);o=observe(model.t,model.drive.state,model.note,row['busV'])
 if not .180<=o['relativeX']<=.190 or abs(o['relativeVx'])>.01:outcome='COUNTERFACTUAL_CORRIDOR_EXIT';break
r=dict(schema='native-signed-clipped-control/1',sourceTrajectorySha256=sha(a.trajectory),scriptSha256=sha(pathlib.Path(__file__)),bindingSha256=sha(a.binding),stepS=a.dt,initial=initial,trace=rows,outcome=outcome,durationS=model.t-initial['timeS'],maximumRelativeSpeedMps=max(abs(x['note']['v'][0]-x['chassis']['vx']) for x in rows),acceptance=False,limits=['Counterfactual from attained native index; exact recorded commands replayed except negative pinch commands clipped to coast','Open-loop command replay after divergence; not a universal impossibility result or hardware prediction'])
a.output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps({k:r[k] for k in ['outcome','durationS','maximumRelativeSpeedMps']}))
