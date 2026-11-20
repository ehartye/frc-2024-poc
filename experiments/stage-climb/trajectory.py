"""Prescribed quasi-static solo pickup and passive hold. No hardware commands."""
import math
import numpy as np


def run(screen, chassis_bounds):
    # Roof and pin travel are reopened joint observations. Final z uses the loaded
    # V-chain from the same static screen. The intake remains in its floor pose.
    hook=np.array([.5,-30.,480.])
    solo=screen['stageChecks'][0]
    loaded,free=solo['loadedSeatMm'],solo['unloadedLowMm']
    cg=np.array(screen['newNominalCenterMm'])
    u=hook-cg
    u/=np.linalg.norm(u)
    axis=np.cross(u,[0.,0.,1.])
    theta=math.atan2(np.linalg.norm(axis),u[2])
    axis/=np.linalg.norm(axis)
    x,y,z=axis
    K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    # t, horizontal approach offset, roof above chassis, pin withdrawal,
    # supported fraction of weight, suspended pose fraction
    knots=np.array([
        [0,460,480,0,0,0],[.6,460,480,52,0,0],[2.1,460,860,52,0,0],
        [3.4,0,860,52,0,0],[4.2,0,free,52,0,0],[5.2,0,loaded,52,1,0],
        [7,0,480,52,1,1],[7.6,0,480,0,1,1],[10,0,480,0,1,1],[15,0,480,0,1,1]])
    rows={}
    for enabled in (True,False):
        trace=[]
        for t in np.round(np.arange(0.,15.001,.05),2):
            offset,roof,pin,load,hang=(float(np.interp(t,knots[:,0],knots[:,i])) for i in range(1,6))
            if not enabled:
                if t>=7: pin=52.
                if t>10:
                    # Free backdrive is a failure hypothesis. Bound descent speed
                    # at 100 mm/s and transfer weight to the floor after contact.
                    rise=min(loaded-480,(t-10)*100)
                    roof=480+rise
                    hang=1-rise/(loaded-480)
                    if hang < 1e-9:
                        hang=0
                        loss=min(free-loaded,max(0,(t-10)*100-(loaded-480)))
                        roof=loaded+loss
                        load=1-loss/(free-loaded)
            ang=theta*hang
            rot=np.eye(3)+math.sin(ang)*K+(1-math.cos(ang))*K@K
            origin=np.array([offset,0.,0.])
            if hang>0:
                origin=np.array([hook[0],hook[1],loaded])-rot@np.array([hook[0],hook[1],roof])
            clearance=float((chassis_bounds@rot[2]).min()+origin[2])
            carried=screen['newNominalKg']*9.80665*load
            trace.append(dict(tS=float(t),teleopS=125+float(t),powered=bool(t<10),approachOffsetMm=offset,
                slideMm=roof-480,pinWithdrawnMm=pin,chainLoadN=carried,lockLoadN=carried if pin==0 and t>=7.6 else 0.,
                originMm=origin.tolist(),quaternionXYZW=[*(axis*math.sin(ang/2)),math.cos(ang/2)],
                floorClearanceMm=clearance,chainLowMm=free-(free-loaded)*load))
        name='engaged' if enabled else 'missed'
        rows[name]=trace
    for group in rows.values():
        if min(p['floorClearanceMm'] for p in group)<-.05:
            raise ValueError('Prescribed floor-pose path penetrates carpet')
    if rows['engaged'][-1]['floorClearanceMm']<1 or abs(rows['missed'][-1]['floorClearanceMm'])>.05:
        raise ValueError('Unexpected terminal floor state')
    dt=.05
    speed=max(abs((b['slideMm']-a['slideMm'])/dt) for a,b in zip(rows['engaged'],rows['engaged'][1:]) if a['chainLoadN']>0)
    power=max(abs(b['slideMm']-a['slideMm'])/dt/1000*max(a['chainLoadN'],b['chainLoadN']) for a,b in zip(rows['engaged'],rows['engaged'][1:]))
    return dict(schema='crescendo-solo-quasistatic/1',nativeSha256=screen['nativeSha256'],stepS=dt,
        startTeleopS=125,cutoffS=10,assessmentS=12.5,holdThroughS=15,
        assumptions=['Other robots come to rest 2.5 seconds after the buzzer. Also inspect the full five-second bound.',
        'Seat capture, loaded-chain profile and linear motion are prescribed. No impact, actuator feedback, swing or referee model.',
        'A fully engaged steel pin transfers guide load after screw unload. A missed pin leaves the drive backdrivable at an assumed 100 mm/s after cutoff.'],
        peakLoadedSlideSpeedMmS=speed,peakMechanicalPowerW=power,minimumScrewInputAt30PctW=power/.30,
        outcome={label:dict(atAssessment=trace[250],atFiveSeconds=trace[-1],
            floorAndLoadWitnessMet=trace[250]['floorClearanceMm']>1 and trace[250]['lockLoadN']>0,
            physicalOnstageCredit='unqualified') for label,trace in rows.items()},
        # Keep decision instants and quarter-second samples for the public replay.
        samples={k:[v for i,v in enumerate(trace) if i%5==0 or v['tS'] in knots[:,0]] for k,trace in rows.items()})
