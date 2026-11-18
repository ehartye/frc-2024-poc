import hashlib,json,pathlib
load=lambda p:json.loads(pathlib.Path(p).read_bytes());left=load('native-static-warm-cycle-half/receipt.json');right=load('native-static-warm-cycle-quarter/receipt.json');keys=[('chassis','vx'),('note','v'),('noteVz',),('vPivot',)]
def value(s,k):
 x=s
 for q in k:x=x[q]
 return x[0] if isinstance(x,list) else x
worst=None
for i,a in enumerate(left['trace'][:len(right['trace'])//2]):
 b=right['trace'][2*i+1]
 for k in keys:
  delta=abs(value(a['state'],k)-value(b['state'],k))
  if worst is None or delta>worst['differenceMps']:worst=dict(timeS=a['timeS'],field='/'.join(k),differenceMps=delta,coarseValue=value(a['state'],k),fineValue=value(b['state'],k),coarsePhase=a['phase'],finePhase=b['phase'])
aligned=[]
for phase in ['acquisition','lift','index']:
 a=[r for r in left['trace'] if r['phase']==phase];b=[r for r in right['trace'] if r['phase']==phase];maximum=0.;field=None;t=None
 for i,row in enumerate(a[:len(b)//2]):
  for k in keys:
   delta=abs(value(row['state'],k)-value(b[2*i+1]['state'],k))
   if delta>maximum:maximum=delta;field='/'.join(k);t=(i+1)*left['stepS']
 aligned.append(dict(phase=phase,maximumVelocityDifferenceMps=maximum,field=field,phaseElapsedS=t))
out=dict(schema='same-law-cycle-clock-diagnostic/1',producerSha256=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),sources=[hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest() for p in ['native-static-warm-cycle-half/receipt.json','native-static-warm-cycle-quarter/receipt.json']],maximumSharedTimeVelocityDifference=worst,phaseRelativeDiagnostics=aligned,originalSharedTimeVelocityGatePassed=False,limits=['Phase-relative comparisons are diagnostic and do not replace the originally declared failed absolute-time velocity bound','No trajectory, controller event or threshold was changed'])
pathlib.Path('native-static-cycle-timing-diagnosis.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
