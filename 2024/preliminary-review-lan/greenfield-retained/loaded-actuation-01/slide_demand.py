"""Measure the command applied to the free-piece trials; qualify smoothing separately."""
from pathlib import Path
import hashlib, json, math, sys
import numpy as np
from scipy.ndimage import convolve1d
p=Path(__file__).resolve().parent;root=p.parent
sys.path.insert(0,str(root/'follower-01'))
from stroke import scheduled
def read(f):return json.loads(f.read_bytes())
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
trial=root/'lateral-station-01/dev-middle-48.json'
source=read(trial);a0=source['pitchDegrees'];a1=source['endPitchDegrees']
x0=read(root/'pickup-lip/layout.json')['pickPivot'][0]
def law(t):
    u=np.clip(t/3.4,0.,1.);blend=u*u*(3.-2*u);a=a0+(a1-a0)*blend
    return a,(scheduled(a)+(x0-scheduled(a0))*(1-blend))*.001
def measure(dt,smooth):
    t=np.arange(-.5,3.901,dt);q,x=law(t)
    if smooth:
        k=np.arange(-smooth/2,smooth/2+dt/2,dt);w=1+np.cos(2*np.pi*k/smooth)
        alt=convolve1d(x,w/w.sum(),mode='nearest')
        # Keep the full entry and shot holds and their exact boundaries.
        z=np.clip(np.minimum(t,3.4-t)/.20,0,1);h=z**3*(10-15*z+6*z*z)
        xx=x+(alt-x)*h
    else:xx=x
    speed=np.gradient(xx,dt);acc=np.gradient(speed,dt);j=np.argmax(abs(acc))
    return dict(dtS=dt,smoothingWindowS=smooth,peakSpeedMps=max(abs(speed)),peakAccelMps2=max(abs(acc)),peakAngleDeg=q[j],peakTimeS=t[j],maximumRetreatMm=max(x-xx)*1000,maximumAdvanceMm=max(xx-x)*1000,endErrorMm=1000*max(abs(xx[0]-x[0]),abs(xx[-1]-x[-1])),maxScrewRPM8mmLead=max(abs(speed))*60/.008)
if __name__=='__main__':
    rows=[measure(dt,w) for w in [0,.08,.16] for dt in [.002,.001,.0005,.00025]]
    report=dict(sourceGeometry=sha(root/'guided-hoist-01/service-locked-robot.step'),sourceCarry=sha(trial),law=sha(root/'follower-01/stroke.py'),actualDriveScript=sha(root/'follower-01/carry_cam.py'),profile='3.4 s, same cubic interpolation, starting horizontal offset and prescribed translation as the free-piece replay',rows=rows,scope='Centered smoothing trials are candidate commands only. The 8 mm/revolution lead is a sizing example. The parked design uses a diameter 10 mm screw envelope; actual lead, reducer, motor, nut and residual collision/retention require verification.')
    (p/'slide-demand.json').write_text(json.dumps(report,indent=2))
    for r in rows: print(*(round(r[k],5) for k in ['smoothingWindowS','dtS','peakSpeedMps','peakAccelMps2','maximumRetreatMm','maximumAdvanceMm','peakTimeS','maxScrewRPM8mmLead']))

