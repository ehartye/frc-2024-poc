"""Additive, explicitly hypothetical four-drive energy/contact experiment."""
import json,pathlib,math,hashlib,copy,argparse,csv
R=pathlib.Path(__file__).resolve().parents[2]

def engagement(x,rows):
 peak=max(v['summedNominalOverlapMm3'] for v in rows)
 for a,b in zip(rows,rows[1:]):
  xa=a['noteCenterFromWheelStationMm']/1000;xb=b['noteCenterFromWheelStationMm']/1000
  if xa<=x<=xb:return ((b['summedNominalOverlapMm3']-a['summedNominalOverlapMm3'])*(x-xa)/(xb-xa)+a['summedNominalOverlapMm3'])/peak
 return 0.

def bus(c,omega,duty):
 # Solve ideal PWM bus current = sum(duty * phase current); one shared supply.
 lo=0.;hi=c['busOpenCircuitV'];r=c['windingResistanceOhm'];kv=c['backEmfRadPerSV']
 def current(v):return [max(0.,min(c['phaseCurrentLimitA'],(duty*v-w/kv)/r)) for w in omega]
 for _ in range(40):
  v=(lo+hi)/2;amps=current(v);ib=sum((w/kv+a*r)*a for w,a in zip(omega,amps))/max(v,1e-12)+c.get('otherRobotCurrentA',0.)
  if v+c['sourceResistanceOhm']*ib>c['busOpenCircuitV']:hi=v
  else:lo=v
 v=(lo+hi)/2;i=current(v);return v,i,sum((w/kv+a*r)*a for w,a in zip(omega,i))/max(v,1e-12)+c.get('otherRobotCurrentA',0.)

def simulate(c,geometry,case='nominal'):
 if c['motors']!=4 or c['ratio']!=1:raise ValueError('this experiment requires four independent 1:1 drives')
 if c['stepS']<=0 or c['radiusM']<=0 or c['rotatingInertiaEachKgM2']<=0:raise ValueError('invalid positive parameter')
 rows=geometry['contactSampling']['rejected central wheel' if case=='central' else 'two lane candidate'];n=c['motors'];dt=c['stepS'];j=c['rotatingInertiaEachKgM2'];r=c['radiusM'];kt=1/c['backEmfRadPerSV'];omega=[0.]*n;initial=0.;source=heat=supplyloss=sliploss=otherwork=0.;minbus=c['busOpenCircuitV'];trace=[];t=0.;phase='spinup';x=c['startMm']/1000;v=0.;releaseT=None;outcome='timeout';feed0=x;peakI=0.;active=0.;lastContact=None;maxContactGap=0.;contactGap=0.;seenContact=False
 for tick in range(math.ceil((c['maxSpinupS']+c['maxFeedS'])/dt)):
  if phase=='spinup' and min(omega)>=c['readyRpm']*2*math.pi/60:
   phase='feed';releaseT=t;v=c['feedEntrySpeedMps'];initial=.5*c['pieceMassKg']*v*v
  if phase=='spinup' and t>=c['maxSpinupS']:outcome='not ready';break
  if phase=='feed' and t-releaseT>=c['maxFeedS']:outcome='feed timeout';break
  b,amps,ib=bus(c,omega,c['duty']);minbus=min(minbus,b);peakI=max(peakI,ib);wgt=engagement(x,rows) if phase=='feed' else 0.;forces=[]
  for w in omega:
   slip=r*w-v
   # Smooth sign avoids numerical chatter; normal is an independent hypothesis.
   forces.append(c['friction']*c['normalEachN']*wgt*math.tanh(slip/.05))
  if wgt>0:
   active+=dt;seenContact=True;lastContact={'xFromStationM':x,'speedMps':v,'timeS':t};contactGap=0.
  elif seenContact and phase=='feed':
   contactGap+=dt;maxContactGap=max(maxContactGap,contactGap)
  otherwork+=b*c.get('otherRobotCurrentA',0.)*dt;source+=c['busOpenCircuitV']*ib*dt;supplyloss+=c['sourceResistanceOhm']*ib*ib*dt;heat+=sum(a*a*c['windingResistanceOhm'] for a in amps)*dt
  sliploss+=sum(f*(r*w-v) for f,w in zip(forces,omega))*dt
  newomega=[w+dt*(kt*a-f*r)/j for w,a,f in zip(omega,amps,forces)]
  if phase=='feed':
   acc=sum(forces)/c['pieceMassKg']-9.80665*math.sin(math.radians(c['pitchDegrees']));nv=v+dt*acc;x+=dt*(v+nv)/2;v=nv
  omega=newomega;t+=dt
  if tick%100==0:trace.append({'timeS':t,'phase':phase,'noteXFromStationM':x,'pieceSpeedMps':v,'busV':b,'busCurrentA':ib,'minRpm':min(omega)*60/(2*math.pi),'engagementWeight':wgt})
  if phase=='feed' and v<0:outcome='reversed before exit';break
  if phase=='feed' and x>=c['releaseBoundaryMm']/1000:outcome='passed contact-table boundary';break
 kinetic=.5*j*sum(w*w for w in omega)+.5*c['pieceMassKg']*v*v;potential=c['pieceMassKg']*9.80665*math.sin(math.radians(c['pitchDegrees']))*(x-feed0);residual=source+initial-kinetic-potential-heat-supplyloss-sliploss-otherwork
 return {'case':case,'outcome':outcome,'elapsedS':t,'feedStartS':releaseT,'endSpeedMps':v,'endXFromStationM':x,'activeContactTimeS':active,'lastPositiveEngagementSample':lastContact,'longestPostContactZeroEngagementS':maxContactGap,'minimumSharedBusV':minbus,'peakSharedBusCurrentA':peakI,'energyJ':{'batterySource':source,'prescribedFeedKinetic':initial,'finalKinetic':kinetic,'piecePotentialChange':potential,'windingHeat':heat,'sourceResistanceLoss':supplyloss,'slipLoss':sliploss,'otherRobotLoadWork':otherwork,'balanceResidual':residual},'acceptance':False},trace

def run(out):
 c=json.loads((R/'sim/fourdrive/config.json').read_text());gp=R/c['sourceGeometry'];g=json.loads(gp.read_text());cases=[]
 for name,changes,mode in [('nominal',{},'nominal'),('9V',{'busOpenCircuitV':9},'nominal'),('low friction',{'friction':.3},'nominal'),('high inertia',{'rotatingInertiaEachKgM2':.0006},'nominal'),('central geometry control',{},'central'),('zero traction',{'friction':0},'nominal'),('4V insufficient speed',{'busOpenCircuitV':4},'nominal')]:
  cc=dict(c,**changes);result,trace=simulate(cc,g,mode);result['case']=name;result['overrides']=changes;cases.append(result)
  if name=='nominal':
   out.mkdir(parents=True,exist_ok=True)
   with (out/'nominal-trace.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=list(trace[0]));w.writeheader();w.writerows(trace)
 report={'schema':'frc-fourdrive-results/1','plantSha256':hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),'configSha256':hashlib.sha256((R/'sim/fourdrive/config.json').read_bytes()).hexdigest(),'geometrySha256':hashlib.sha256(gp.read_bytes()).hexdigest(),'cases':cases,'limits':c['assumptions'],'integration':'End boundary is end of passage screen, not release. Last positive engagement is only a sampled-model release estimate, not old guide exit. Recompute physical release pose and target intersection only after contact and mount agreement.'};(out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(cases,indent=2))
if __name__=='__main__':run(R/'artifacts/fourdrive')
