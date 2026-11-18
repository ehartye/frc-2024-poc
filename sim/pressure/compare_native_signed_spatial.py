"""Local contact quadrature comparison at actual fresh-run states, not full trajectories."""
import argparse,hashlib,json,math,pathlib,sys
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
p=argparse.ArgumentParser()
for n in ['project','binding','trajectory','contract','output']:p.add_argument('--'+n,type=pathlib.Path,required=True)
a=p.parse_args();assert not a.output.exists();sys.path[:0]=[str(a.project/'sim/pressure'),str(a.project/'sim/integrated')]
import native_signed_pressure as native
load=lambda p:json.loads(p.read_bytes());r=load(a.trajectory);binding=load(a.binding);contract=load(a.contract);cad=load(a.project/'cad/assembly/intake_components.json');nc=load(a.project/'sim/acquisition/free-planar-config.json')['model'];dc=load(a.project/'sim/drivetrain/config.json')
selected={'firstContact':next(x for x in r['trace'] if x['contacts'])}
if r['index']:selected['index']=min(r['trace'],key=lambda x:abs(x['timeS']-r['index']['timeS']))
transport=[x for x in r['trace'] if x['phase']=='transport']
if transport:selected['maximumTransportReaction']=max(transport,key=lambda x:math.hypot(*x['reactionForceWorldN']))
rows=[]
for name,state in selected.items():
 values=[]
 for resolution in contract['levels']:
  m=native.NativeSignedPressure(dc,nc,cad,binding,resolution=(resolution,resolution));q=m.acquisition_rates(state['note'],state['chassis'],state['busV'],[0.,0.],{});values.append(dict(resolution=resolution,normalN=[c['normalMagnitudeN'] for c in q['contacts']],reactionN=q['reactionForceWorldN'],potentialJ=q['springJ']))
 comparisons=[]
 for one,two in zip(values,values[1:]):
  errors=[]
  for k in ['normalN','reactionN']:
   errors.extend(abs(x-y)/(contract['absoluteForceFloorN']+contract['relativeTolerance']*max(abs(x),abs(y))) for x,y in zip(one[k],two[k]))
  errors.append(abs(one['potentialJ']-two['potentialJ'])/(contract['absolutePotentialFloorJ']+contract['relativeTolerance']*max(one['potentialJ'],two['potentialJ'])))
  comparisons.append(dict(levels=[one['resolution'],two['resolution']],maximumNormalizedDifference=max(errors),withinLocalBounds=max(errors)<=1))
 rows.append(dict(name=name,timeS=state['timeS'],values=values,comparisons=comparisons))
out=dict(schema='native-signed-local-spatial/1',trajectorySha256=sha(a.trajectory),bindingSha256=sha(a.binding),contractSha256=sha(a.contract),rows=rows,localFineBoundsPassed=all(x['comparisons'][-1]['withinLocalBounds'] for x in rows),acceptance=False,limits=['Only prescribed states from the attained trajectory; no full spatial trajectory refinement','Motor currents set to coast only for this instantaneous contact-law query; no trajectory state is changed'])
a.output.write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps(out,indent=2))
