from pathlib import Path
import sys
p=Path(__file__).resolve().parent;template=p.parent/'pickup-lip/shot_replay.py'
name,mode,n=sys.argv[1:];n=int(n)
setup=template.read_text().split("exec(compile(src")[0]
env={'__file__':str(template)};exec(setup,env)
src=env['src']
def sub(a,b):
    global src
    assert src.count(a)==1,a
    src=src.replace(a,b)
sub('    for step in range', '''    permitFor=0.;enableT=None;lowestPre=20.;rejected=0;overlaps=0;maxConcurrent=0.
    for step in range''')
sub('        currents=[]', '''        t=d.time
        requestAccel=bool(t<.4 or (.55<t<.70) or policy=="stuck")
        # Simulated power arbiter owns acceleration permission as well as scorer permission.
        acceleration=requestAccel and not(policy!="bypass" and enableT is not None and release is None)
        ancillary=118. if acceleration else 25.
        forecast=11.8-.027*(ancillary+100.)
        eligible=not acceleration and forecast>=8.2
        permitFor=permitFor+dt if eligible else 0.
        if enableT is None and (policy=="bypass" or permitFor>=.1-1e-10):enableT=t
        active=bool(enableT is not None)
        if requestAccel and not acceleration: rejected+=1
        if acceleration and active and release is None: overlaps+=1
        if not active:lowestPre=min(lowestPre,11.8-.027*ancillary)
        if t>1.2 and enableT is None:break
        currents=[]''')
sub('else 0.;Fmotor=', 'else 0.;Fmotor=') # checked below
sub('0,limit) if drive else 0.', '0,limit) if active else 0.')
sub('if drive else np.clip(F,-1.,1.)', 'if active else np.clip(F,-1.,1.)')
# End the requested high-current spin-up when the NOTE clears the cassette.
sub('if active else 0.;Fmotor=', 'if active and release is None else 0.;Fmotor=')
sub('bus=11.8-.027*(25+sum(currents))','maxConcurrent=max(maxConcurrent,ancillary+sum(currents));bus=11.8-.027*(ancillary+sum(currents))')
sub('result=dict(changedContacts=',"""result=dict(policy=policy,enableTime=enableT,maskedAccelerationSteps=rejected,poweredOverlapSteps=overlaps,
        peakPackA=maxConcurrent,allowWindowS=.100,voltageFloorV=8.,passShot=bool(enableT is not None and both is not None and minBus>=8 and overlaps==0),
        passInhibit=bool(enableT is None and release is None and minBus>=8 and both is None),changedContacts=""")
sub("here=Path(__file__).resolve().parent","here=destination")
env.update(policy=mode,destination=p,__name__='__main__')
sys.argv=[__file__,name,'125','1.43','1',str(.024/n),str(n),'1','.20','5.55','1','50']
exec(compile(src,'bounded shot','exec'),env)

