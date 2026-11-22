"""Drive the two hook cables from the shared, declutched pivot shaft with a bounded weak-pack motor."""
from pathlib import Path
import sys,math
p=Path(__file__).resolve().parent;srcdir=p.parent/'endgame-route-01';label,cg,mode,ramp=sys.argv[1:5];ramp=float(ramp)
src=(srcdir/'guided-equipped.py').read_text()
def sub(a,b):
    global src
    assert src.count(a)==1,a
    src=src.replace(a,b)
sub("p=Path(__file__).resolve().parent;srcdir=p.parent/'chain-contact-02'","p=Path(__file__).resolve().parent;srcdir=p.parent/'chain-contact-02'")
sub("doc=", "doc=") if False else None
sub("dt=m.opt.timestep;cut=5.;","""dt=m.opt.timestep;cut=8.8;coil=0.;velocity=0.;minV=12.;maxI=0.;maxBatt=0.;maxPower=0.;maxTension=0.;at=None;timeSeat=0.;peakCableForce=0.;peakPitchReaction=0.;peakGuideMoment=0.;rDrum=.0295;reduction=250.;kt=.0198;ke=.0202;res=.115;eff=.72;rotorInertia=.00004;reflected=rotorInertia*(reduction/rDrum)**2+.003/rDrum**2;wiringR=.027;aux=25.;""")
sub("u=np.clip((t-.35)/2.5,0,1);target=(stroke+.002)*u*u*(3-2*u) if bilateral else 0.","""if bilateral and at is None and t<cut:at=t
    u=np.clip((t-.45)/ramp,0,1);target=(stroke+.009)*u*u*(3-2*u) if at is not None else 0.
    tv=(stroke+.009)*6*u*(1-u)/ramp if at is not None else 0.
    # Current control accounts for back EMF, battery droop, and short-duration negative torque during speed regulation.
    em=velocity*reduction/rDrum*ke
    want=np.clip(3000*(target-coil)+550*(tv-velocity),-25.,35.) if t<cut and at is not None else 0.
    bus=11.8-wiringR*aux
    if t<cut and at is not None:
        for repeat in range(6):
            amps=float(np.clip(want,(-bus-em)/res,(bus-em)/res));amps=float(np.clip(amps,-25,35))
            watts=max(0.,(em+amps*res)*amps);bus=11.8-wiringR*(aux+watts/max(8.,bus))
    else:amps=0.;watts=0.
    maxBatt=max(maxBatt,watts/max(8.,bus)+aux);minV=min(minV,bus);maxI=max(maxI,abs(amps));maxPower=max(maxPower,watts)
    # Both parallel ropes together; finite stiffness, 0.5 mm slack and unilateral tension.
    cable=float(np.clip(200000*(coil-x(draw)-.0005)+90*(velocity-speed(draw)),0,3500)) if coil>x(draw)+.0005 else 0.
    fDrive=amps*kt*reduction*eff/rDrum;force=fDrive-cable-75*velocity
    velocity+=dt*force/reflected;coil+=dt*velocity
    if coil<0:coil=0.;velocity=max(0.,velocity)
    maxTension=max(maxTension,cable)
    timeSeat=timeSeat+dt if abs(x(draw)-stroke)<.0018 and abs(speed(draw))<.030 and at is not None else 0.""")
sub("if bilateral and x(draw)>stroke-.002 and mode!='no-catch' and not latched:","if timeSeat>=.045 and t<cut and mode!='no-catch' and not latched:")
sub("pull=float(np.clip(230000*(target-x(draw))-1300*speed(draw),0,1350)) if t<cut and bilateral else (float(np.clip(-180000*x(draw)-900*speed(draw),-150,150)) if t<cut else 0.)","""# Home detent is withdrawn only after capture. It cannot support the robot once the carriage leaves.
    pull=cable if at is not None else float(np.clip(-180000*x(draw)-900*speed(draw),-150,150))
    eid=m.equality('brake').id
    for j in np.flatnonzero((d.efc_type==int(mj.mjtConstraint.mjCNSTR_EQUALITY))&(d.efc_id==eid)):
        peakPitchReaction=max(peakPitchReaction,abs(d.efc_force[j]))""")
sub("pull=pull,ground=ground","pull=pull,coil=coil,bus=bus,phaseA=amps,battA=aux+watts/max(8.,bus),ground=ground")
sub("out=dict(strokeM=stroke","out=dict(weakBusMin=minV,peakBatteryA=maxBatt,peakMotorW=maxPower,peakPhaseA=maxI,maxCombinedCableN=maxTension,peakPitchLockNm=peakPitchReaction,cutS=cut,firstCapture=at,strokeM=stroke")
sub("'scope'","'scope'") if False else None
sub("Assumed 1350 N drive cap and 750 g carriage increment.","Shared 250:1 pivot motor, 29.5 mm cable radius, assumed rotor inertia, two tensile ropes, weak-pack droop, 750 g moving allocation and idealized pitch lock. Both cables are represented by equal reflected motion; load share and guide friction need independent checks.")
sys.argv=[__file__,label,cg,mode,'.230']
exec(compile(src,'guided cable hoist on existing pivot motor','exec'))

