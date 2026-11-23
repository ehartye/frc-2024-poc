"""Check the 135 s TELEOP clock and separation when driving away after the last shot."""
from pathlib import Path
import json,math
import numpy as np
p=Path(__file__).resolve().parent
read=lambda f:json.loads(f.read_bytes())
route=read(p/'endgame-transfer.json');cut=route['powerCutFromSixthS'];bounds=read(p/'plan.json')['envelope']
# Endgame heading is fixed on the outward leg. Its first segment spans the full field observation window.
a,b=np.array(route['path'][:2])/1000;L=np.linalg.norm(b-a);D=route['legsS'][0]
MAX_SPEED=1.2;ACC=1.4;v=min(MAX_SPEED,math.sqrt(ACC*L));ta=v/ACC
def drive(t):
    t=np.clip(t,0,D);return np.where(t<ta,.5*ACC*t*t,np.where(t>=D-ta,L-.5*ACC*(D-t)**2,.5*ta*v+(t-ta)*v))
report=[]
for n in (48,96):
    d=read(p/f'run-dev-{n}.json');part=d['parts'][-1];s=read(p/f'split-tail_.25_0-dev-center-low-{n}-6.88.json')
    depart=s['feedTime']+1.;assert depart>s['opening']['fullyThroughByS']>s['release']['t']
    tr=s['trace'];t=np.array([r['t'] for r in tr]);x=np.array([r['q'][0] for r in tr])
    x0=a[0]+drive(t-depart)*(b[0]-a[0])/L
    gap=x0+bounds[0][0]-x-.1778
    sel=t[1:]>=depart
    conservative=np.minimum(gap[:-1],gap[1:])-(12+1.2)*np.diff(t)*.5
    # Use the full high moving mass and a conservative upper bound for all four motors at peak speed and acceleration.
    mass=read(p.parent/'mounted-slide-02/gross-power.json')['runningKg']
    amps=(mass*(ACC+.025*9.80665)/4)*.0508/(5.5*.82*(2.6/105))
    voltage=(12.-12./105*1.8)/(5676*math.tau/60)*v*5.5/.0508+amps*12/105
    watts=4*amps*voltage/.95;net=11.8-.027*35;bus=(net+math.sqrt(net**2-4*.027*watts))/2
    report.append(dict(n=n,scoreClockS=d['elapsedS'],scoredThenCutS=d['elapsedS']+cut,marginIfWaitingForCountS=135-d['elapsedS']-cut,robotDepartureAfterFeedS=1.,robotDepartureS=part['shotStart']+depart,cutClockOnRobotTimerS=part['shotStart']+depart+cut,matchMarginOnTimerS=135-part['shotStart']-depart-cut,outboundPieceSphereGapM=float(min(conservative[sel])),weakBusBoundDuringFirstLeg=bus,driveMotorReserveV=bus-voltage,completeOriginalFieldTraceThroughS=t[-1],untilNextLegS=D,comparison='practice scoring and 24 s terminal branch are separate narrative objectives; combined 135 s case assumes scoring starts at TELEOP zero, same original traffic and prompt controller transitions. No time allocation for hesitation or field disturbances.'))
(p/'endgame-clock.json').write_text(json.dumps(report,indent=2));print(report)

