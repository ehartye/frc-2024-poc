"""Apply two finite, independently driven belt zones to the shared free-NOTE pickup model."""
from pathlib import Path
import sys
here = Path(__file__).resolve().parent
sources = here.parent / 'six-piece-02'
sys.path.insert(0, str(here.parent / 'scorer-contact-01'))
from patch import belt_patch, coefficients
material = coefficients()
def split_intake(src):
    begin = src.index('        # Actual finite soft belt reaches')
    end = src.index('        bodyPos,yaw,accField', begin)
    contact = """        # All velocities in the cassette frame, including the translating pivot.
        angular = np.array([0., -w, 0.])
        origin = np.array([X+advance, 0., Z])
        pieceV = d.qvel[v0:v0+3] - [vx+d.qvel[model.jnt_dofadr[drive]],0.,0.] - np.cross(angular,q[:3]-origin)
        pieceW = world@d.qvel[v0+3:v0+6] - angular
        torques = np.zeros(3)
        for flank, side in enumerate((-1,1)):
            for zone,low,high,limit in ((0,.097,.352,8.),(1+flank,.407,.600,20.)):
                rail=np.column_stack((np.clip(local[:,0],low,high),np.full(n,side*.1898),np.clip(local[:,2],.032,.064)))
                force,pressure,traction,arc,dissipation=belt_patch(local-rail,pts,q[:3],pieceV,pieceW,R,beltVelocity[zone],dt,[*material[:4],limit])
                wr+=force;torques[zone]+=traction
                maxF=max(maxF,abs(traction))
                if abs(traction)>1 and takeover is None:takeover=t
        # Nose uses its own current-limited motor. Negative cassette S draws the floor piece inward.
        targets=np.full(3, -commanded)
        # Compact 4:1 feed motor; existing independent launcher motors at their pickup current limits.
        cur=np.clip((targets-beltVelocity)*np.array([18.,16.,16.]),[-8,-12,-12],[8,12,12])
        forcePerAmp=np.array([.01*4*.70/.012,.0198*.87/.018,.0198*.87/.018])
        beltVelocity+=dt*(cur*forcePerAmp-torques-np.array([.5,.4,.4])*beltVelocity)/np.array([.18,.20,.20])
        peakSideCurrent=np.maximum(peakSideCurrent,abs(cur))
"""
    src=src[:begin]+contact+src[end:]
    src=src.replace('    for step in range','    beltVelocity=np.zeros(3);peakSideCurrent=np.zeros(3)\n    for step in range',1)
    return src.replace('return dict(endPieceQpos=', 'return dict(endZoneSpeeds=beltVelocity.tolist(),peakZonePhaseA=peakSideCurrent.tolist(),endPieceQpos=')
loaded=(sources/'loaded.py').read_text()
loaded=loaded.replace("(folder/'plan.json')","(sources/'plan.json')")
# Once the templates have bound geometry, gravity and the moving chassis, switch only the belt contacts.
needle="exec(compile(script,'carried NOTE in accelerating chassis','exec'))"
hook="""script=script.replace("exec(compile(source,'joined-floor-to-carry','exec'))", "source=source.replace('exec(src);r=run','src=split_intake(src);exec(src);r=run');exec(compile(source,'joined-floor-to-carry','exec'))")
"""
assert loaded.count(needle)==1
exec(compile(loaded.replace(needle,hook+needle),'continuous split pickup and carry','exec'))

