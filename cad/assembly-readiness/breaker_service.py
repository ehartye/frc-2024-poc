"""Screen the breaker service approach and a local flexible cable route against the parked assembly."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

ROOT=Path(__file__).resolve().parents[2]


def evaluate(observations, boot_od=30.):
    model=json.loads(observations.read_bytes())
    breaker=next(row for row in model['rows'] if row['body']=='R7 149 main-breaker')
    shift=round(breaker['boundsMm'][1]+199.13)
    if shift not in (0,12):raise ValueError('Unexpected breaker registration')
    rows=[row for row in model['rows'] if row is not breaker and row['component']!='R9 local breaker harness']
    # A single box fills the space between the two strap flanges. Replace
    # those boxes with the original three conservative stock envelopes.
    refined=[]
    for row in rows:
        if row['body'].startswith(('R7 205 battery-strap-leg','R7 212 battery-strap-leg')):
            x,y,z,X,Y,Z=row['boundsMm']
            cy=-286.5 if '286.5' in row['body'] else -193.5
            for bb in [[x,y,z,X,Y,z+3],[x,y,Z-3,X,Y,Z],[x,cy-1.5,z+3,X,cy+1.5,Z-3]]:
                refined.append({**row,'boundsMm':bb})
        else:refined.append(row)
    rows=refined
    boxes=np.asarray([row['boundsMm'] for row in rows])
    def obstacles(points,rad):
        p=np.asarray(points)
        gap=np.maximum(boxes[None,:,:3]-p[:,None,:],p[:,None,:]-boxes[None,:,3:])
        distances=np.linalg.norm(np.maximum(gap,0.),axis=2)
        other=np.unravel_index(np.argmin(distances),distances.shape)
        return float(distances[other]-rad),rows[other[1]]['body']
    contacts=[]
    for x,y,preferred,turn_sign in [(-308.575,-161.03,105,-1),(-251.425,-188.97,15,1)]:
        y+=shift
        candidates=[]
        for azimuth in [preferred]:
            a=math.radians(azimuth);direction=np.array([math.cos(a),math.sin(a),0])
            for sign in [turn_sign]:
                turn=sign*np.array([-direction[1],direction[0],0]);origin=np.array([x,y,262.])
                # The diameter keepout is a receiving gauge around the
                # boot exterior, which the catalog specifies only by internal fit.
                boot=origin+[0,0,9]+np.linspace(0,27,28)[:,None]*direction
                points=origin+np.linspace(30,53,24)[:,None]*direction
                t=np.linspace(0,math.pi/2,91)
                bend=origin+53*direction+50*np.sin(t[:,None])*direction+50*(1-np.cos(t[:,None]))*turn
                end=bend[-1]+np.linspace(0,18,19)[:,None]*turn
                wire=np.vstack((points,bend,end))
                low_boot,near_boot=obstacles(boot,boot_od/2+.5)
                low_wire,near_wire=obstacles(wire,7.75/2+.5)
                # Do not route wiring onto or out of the frame. Both ends
                # spend their last straight segment in the fixed assembly.
                outline=np.vstack((boot,wire))
                inside=min(outline[:,0].min()+353.6625,454.462-outline[:,0].max(),
                           outline[:,1].min()+355.6,355.6-outline[:,1].max())-boot_od/2-.5
                # Outward-facing lug: avoid routing its barrel into the body.
                facing=direction@np.array([x+280,y+175-shift,0.])>12
                if not facing:continue
                score=min(low_boot,low_wire,inside)
                candidates.append((score,-abs((azimuth-preferred+180)%360-180),azimuth,sign,low_boot,near_boot,
                                   low_wire,near_wire,wire[-1].tolist(),wire.tolist()))
        selected=max(candidates,key=lambda c:c[:2])
        contacts.append(dict(studXYmm=[x,y],azimuthDeg=selected[2],bendSign=selected[3],screenClearanceMm=selected[0],
            bootGaugeToOtherBodyMm=selected[4],nearestBootObstacle=selected[5],
            cableToOtherBodyMm=selected[6],nearestCableObstacle=selected[7],clampCenterMm=[x+45*math.cos(a),y+45*math.sin(a),262.],routeOutletMm=selected[8],
            wireSamplesMm=selected[9]))
    routes=[np.asarray(k.pop('wireSamplesMm')) for k in contacts]
    spacing=np.min(np.linalg.norm(routes[0][:,None,:]-routes[1][None,:,:],axis=2))-7.75-1.
    # Source switch/flag is near original terminal Z=+16 mm and reaches 37.323 mm in native Y.
    # The exterior approach begins at the top of its model; positions of moving internal contacts are unspecified.
    x,y=-280.,-191.13+shift
    rows+= [row for row in model["rows"] if row["component"]=="R9 local breaker harness"]
    boxes=np.asarray([row["boundsMm"] for row in rows])
    hand=np.array([[x,y,z] for z in np.arange(291.15,351.2,.5)])
    clear,closest=obstacles(hand,10.25)
    # Project-sized gloved-hand mock-up. Pull the hand straight up through
    # a column extending above the parked machine, with one fingertip below it.
    lo=np.array([x-42.5,y-17.5,341.15])
    hi=np.array([x+42.5,y+17.5,max(boxes[:,5])+100.])
    gaps=np.maximum(np.maximum(boxes[:,:3]-hi,lo-boxes[:,3:]),0.)
    hand_distances=np.linalg.norm(gaps,axis=1)
    closest_hand=int(np.argmin(hand_distances))
    received=None
    if shift==12:
        from boot_receiving import measure
        received=measure(model['rows'])
    return dict(allNeighborBootReceiving=received,archiveSha256=model['archiveSha256'],localRoutes=contacts,cablePairClearanceMm=float(spacing),
        receivingGauge=dict(bootOuterDiameterMm=boot_od,bootCenterHeightMm=271,bootAxisLengthMm=27,
                           cableDiameterMm=7.75,centerlineBendRadiusMm=50,samplePadMm=.5),
        resetApproach=dict(diameterMm=20,lengthMm=60,otherBodyClearanceMm=clear,nearest=closest,palmWidthMm=85,palmThicknessMm=35,
                           palmSweepBoundsMm=[*lo.tolist(),*hi.tolist()],palmClearanceMm=float(hand_distances[closest_hand]),
                           nearestPalm=rows[closest_hand]["body"]),
        selected=dict(lug='Panduit LCAX6-14-L',boot='Pacer TBCA2-R',wire='Southwire 104110604, 6 AWG red'),
        limits=['Checks conservative installed neighboring-body boxes. Device itself is excluded from the boot and wire boxes.',
                'Receiving gauge is an exterior acceptance limit; supplier boot data rates terminal OD, cable entry and post height.',
                'The clamp centers use a 45 mm straight station. See the native saddle and deck checks. Verify jacket grip and allowable bend radius on the received lot.',
                'Full disconnect/pull path, actual SB50 bracket and mating end, positive insulation and switch travel need physical assembly.'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--predecessor',type=Path);p.add_argument('observations',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    result=evaluate(a.observations)
    if a.predecessor:result['priorLocationControl']=evaluate(a.predecessor,30.)
    a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf8',newline='\n')
