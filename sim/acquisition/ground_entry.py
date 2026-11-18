"""Centered ground approach experiment; no lateral recentering or lift retention claim."""
import pathlib,json,hashlib,math,sys,argparse,copy
R=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(R/'cad/assembly'))
import audit_floor_entry as geometry
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def inputs():return geometry.pins()
def load_geometry():
 p=R/'cad/assembly/evidence/floor-entry.json';report=json.loads(p.read_text());expected=inputs()
 if report['inputs']!=expected or report['assemblyPartCount']!=711:raise ValueError('Stale or incomplete floor geometry')
 if [(r['lateralOffsetMm'],r['advanceMm']) for r in report['rows']]!=[(o,a) for o in [0,-5,5] for a in range(0,481,10)]:raise ValueError('Full nominal and offset inventory required')
 rows=[r for r in report['rows'] if r['lateralOffsetMm']==0]
 if any(h['contactGroup']!='pinch' for r in rows for h in r['hits']):raise ValueError('Unresolved centered path')
 if not all(any(h['contactGroup']=='unresolved' for r in report['rows'] if r['lateralOffsetMm']==o for h in r['hits']) for o in [-5,5]):raise ValueError('Offset counterexamples must be preserved')
 return rows,report

def run_case(c,rows,name):
 c=copy.deepcopy(c)
 for k,v in c.items():
  if k!='assumptions' and (type(v) not in [int,float] or not math.isfinite(v) or v<0):raise ValueError('Finite nonnegative config:'+k)
 for k in ['stepS','horizonS','radiusM','rotorInertiaKgM2','kvRadpsPerV','motorResistanceOhm','pieceMassKg','sourceV','currentLimitA','baseDecelerationMps2','slipRegularizationMps']:
  if c[k]<=0:raise ValueError('Positive config:'+k)
 if c['duty']>1 or c['entryWindowMinX']>c['entryWindowMaxX']:raise ValueError('Config domain')
 if name=='coast':c['brakeDampingNmPerRadps']=0.
 elif name=='low-ground-friction':c['groundMu']=.1
 elif name=='fast-approach':c['approachMps']=.8
 elif name=='zero-belt':c['duty']=0.
 elif name=='early-brake':c['brakeTriggerRobotX']=.25
 elif name=='half-step':c['stepS']/=2
 elif name!='braked-hypothesis':raise ValueError('Case')
 dt=c['stepS'];r=c['radiusM'];j=c['rotorInertiaKgM2'];kv=c['kvRadpsPerV'];res=c['motorResistanceOhm'];mass=c['pieceMassKg'];volts=c['sourceV'];sr=c['sourceResistanceOhm']
 centers=[r0['centerRobotMm'][0]/1000 for r0 in rows]
 totals=[sum(h['overlapMm3'] for h in row['hits']) for row in rows];peak=max(totals)
 def weight(q):
  for k in range(len(rows)-1):
   hi,lo=centers[k],centers[k+1]
   if lo<=q<=hi:
    a=(hi-q)/(hi-lo);return ((1-a)*totals[k]+a*totals[k+1])/peak
  return 0.
 xr=0.;xp=centers[0];vr=c['approachMps'];vp=0.;w=[0.,0.];t=0.;braking=False;dwell=0.;trace=[];events=[];outcome='HORIZON';tick=0
 energy=dict(electricSource=0.,windingHeat=0.,sourceHeat=0.,slipHeat=0.,groundHeat=0.,brakeHeat=0.,prescribedBaseWork=0.)
 maxres=0.;minq=xp;minbus=volts
 while t<c['horizonS']:
  q=xp-xr
  if q<centers[-1]-1e-9 or q>centers[0]+1e-9:outcome='LEFT_SAMPLED_ENTRY_DOMAIN';break
  if not braking and q<=c['brakeTriggerRobotX']:
   braking=True;events.append(dict(timeS=t,event='APPROACH_BRAKE_TRIGGER',robotX=xr,pieceWorldX=xp,pieceWorldV=vp,relativeX=q))
  duty=0. if braking else c['duty'];beta=c['brakeDampingNmPerRadps'] if braking else 0.
  def current(v):return [0. if duty==0 else max(0.,min(c['currentLimitA'],(duty*v-om/kv)/res)) for om in w]
  lo,hi=0.,volts
  for _ in range(40):
   bus=(lo+hi)/2;amps=current(bus);ib=sum((om/kv+i*res)*i for om,i in zip(w,amps))/max(bus,1e-12)
   if bus+sr*ib>volts:hi=bus
   else:lo=bus
  bus=(lo+hi)/2;amps=current(bus);ib=sum((om/kv+i*res)*i for om,i in zip(w,amps))/max(bus,1e-12)
  occupancy=weight(q);belt=[vr-r*om for om in w]
  force=[c['beltMu']*c['normalEachN']*occupancy*math.tanh((v-vp)/c['slipRegularizationMps']) for v in belt]
  ground=-c['groundMu']*mass*9.80665*math.tanh(vp/c['slipRegularizationMps'])
  energy['electricSource']+=volts*ib*dt;energy['windingHeat']+=sum(i*i*res for i in amps)*dt;energy['sourceHeat']+=sr*ib*ib*dt
  energy['slipHeat']+=sum(f*(v-vp) for f,v in zip(force,belt))*dt;energy['groundHeat']+=-ground*vp*dt;energy['brakeHeat']+=beta*sum(om*om for om in w)*dt
  energy['prescribedBaseWork']+=sum(force)*vr*dt
  w=[om+dt*(i/kv+f*r-beta*om)/j for om,i,f in zip(w,amps,force)]
  nv=vp+dt*(sum(force)+ground)/mass;xp+=dt*(vp+nv)/2;vp=nv
  nvr=max(0.,vr-c['baseDecelerationMps2']*dt) if braking else vr;xr+=dt*(vr+nvr)/2;vr=nvr;t+=dt;minq=min(minq,xp-xr);minbus=min(minbus,bus)
  kinetic=.5*mass*vp*vp+.5*j*sum(om*om for om in w)
  residual=energy['electricSource']+energy['prescribedBaseWork']-sum(energy[k] for k in ['windingHeat','sourceHeat','slipHeat','groundHeat','brakeHeat'])-kinetic;maxres=max(maxres,abs(residual))
  row=dict(timeS=t,robotWorldX=xr,robotSpeedMps=vr,pieceWorldX=xp,pieceWorldSpeedMps=vp,relativeRobotX=xp-xr,relativeSpeedMps=vp-vr,omegaRadps=w[:],busV=bus,duty=duty,braking=braking,occupancy=occupancy,energyJ=dict(energy,kinetic=kinetic,residual=residual))
  if tick%20==0:trace.append(row)
  if braking and c['entryWindowMinX']<=xp-xr<=c['entryWindowMaxX'] and abs(vp-vr)<c['dwellSpeedMps'] and vr==0:dwell+=dt
  else:dwell=0.
  if dwell>=c['dwellS']:outcome='CENTERED_GROUND_DWELL_ONLY';break
  tick+=1
 if not trace or trace[-1]['timeS']!=row['timeS']:trace.append(row)
 return dict(case=name,outcome=outcome,events=events,trace=trace,final=row,minimumRelativeX=minq,minimumBusV=minbus,maxAbsEnergyResidualJ=maxres,acceptance=False)

def run(output):
 if output.exists():raise ValueError('Fresh output required')
 rows,g=load_geometry();cp=R/'sim/acquisition/config.json';c=json.loads(cp.read_text());files=[pathlib.Path(__file__),cp,R/'cad/assembly/evidence/floor-entry.json'];pins=dict(inputs(),**{p.relative_to(R).as_posix():sha(p) for p in files})
 cases=[run_case(c,rows,n) for n in ['braked-hypothesis','coast','low-ground-friction','fast-approach','zero-belt','early-brake','half-step']]
 if any(sha(R/p)!=h for p,h in pins.items()):raise ValueError('Inputs changed')
 report=dict(schema='frc-centered-ground-entry/1',inputs=pins,cases=cases,acceptance=False,limits=c['assumptions']);output.mkdir(parents=True);(output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');return report
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--output',type=pathlib.Path,required=True);v=a.parse_args();print([(c['case'],c['outcome'],c['final']['relativeRobotX']) for c in run(v.output)['cases']])
