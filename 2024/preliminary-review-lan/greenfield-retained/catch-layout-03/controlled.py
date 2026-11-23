"""Seat under closed-loop position control to avoid dropping a suspended load onto a rigid stop."""
from pathlib import Path
p=Path(__file__).resolve().parent
body=(p/'rigid_catch.py').read_text()
idx=body.index("a=(p/'sequence.py')");exec(body[:idx])
original=stiffen
def stiffen(src):
    src=original(src)
    src=src.replace('overloaded=False;maxCompression','setpoint=None;integral=0.;unload=None;motorAtSeat=None;overloaded=False;maxCompression')
    a='if armed:torque*=1.-.98*np.clip((t-armAt)/.45,0,1)'
    assert src.count(a)==1
    src=src.replace(a,'''if armed and t<cut:
        if setpoint is None:
            setpoint=angle;integral=torque
        above=float(setpoint-np.deg2rad(158))
        rateDown=np.deg2rad(.22+min(3.8,max(0.,np.rad2deg(above)-.20)*3.8))
        setpoint=max(np.deg2rad(157.4),setpoint-dt*rateDown)
        err=setpoint-angle
        integral=np.clip(integral+10000.*err*dt,-215.,215.)
        torque=np.clip(integral+9500.*err-240.*d.qvel[vi],-215.,215.)
        if unload is not None:torque=motorAtSeat*(1.-np.clip((t-unload)/.60,0,1))''')
    src=src.replace('if reaction>0 and contactAt is None:contactAt=t','''if reaction>0 and contactAt is None:contactAt=t
    if reaction>20. and abs(d.qvel[vi])<np.deg2rad(.9) and unload is None:
        unload=t;motorAtSeat=torque''')
    return src
exec(body[idx:])

