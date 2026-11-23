from pathlib import Path
import numpy as np,json,hashlib
q=Path(__file__).parent
c=Path.home()/'repos/frc-2024-poc-worktrees/engineering-iterations/iterations/002/design/over-bumper-pickup'
ratio=76/21;r=.01125;kt=.97/100;ke=(12-1.4*.12)/(11000*np.pi/30)
out={}
for n in [96,192]:
 d=q/f'replay-{n}'; orig=c/f'head-clear-{n}'
 if not (d/'toe.npy').exists():continue
 a=np.load(d/'toe.npy')
 rows={}
 for name,lo,hi in [('extend',.32,.56),('hold',.56,3.70),('withdraw',3.70,3.94),('settle',3.94,4.75)]:
  b=a[(a[:,1]>=lo)&(a[:,1]<hi)]
  x=b[:,2::3];v=b[:,3::3];F=b[:,4::3]
  maxF=np.argmax(abs(F),axis=0);maxV=np.argmax(abs(v),axis=0); ix=np.arange(3)
  force=np.stack([abs(F[:,0]+F[:,1])+2,abs(F[:,2])+1],axis=1)
  vel=np.stack([np.maximum(abs(v[:,0]),abs(v[:,1])),abs(v[:,2])],axis=1)
  I=1.4+r*force/(kt*ratio*.65);V=.12*I+ke*(ratio/r)*vel
  rows[name]=dict(peakN=abs(F[maxF,ix]).tolist(),speedAtPeakN=v[maxF,ix].tolist(),topSpeed=abs(v[maxV,ix]).tolist(),forceAtTopSpeed=F[maxV,ix].tolist(),peakAbsWatts=abs(F*v).max(axis=0).tolist(),worstSideMismatchMM=1000*max(abs(x[:,0]-x[:,1])),trialPeakMotorAmps=I.max(axis=0).tolist(),trialPeakVolts=V.max(axis=0).tolist(),trialTwoControllersInputAmpsAt8p884V=max((I*V).sum(axis=1)/(.90*8.884)))
 eq={s: json.loads((d/s).read_text())==json.loads((orig/s).read_text()) for s in ['run.json','floor/cycle.json']}
 out[n]=dict(replayEqualsIssued=eq,signalsSha256=hashlib.sha256((d/'toe.npy').read_bytes()).hexdigest(),phases=rows)
print(json.dumps(out,indent=1));(q/'phase-demand.json').write_text(json.dumps(out,indent=2))

