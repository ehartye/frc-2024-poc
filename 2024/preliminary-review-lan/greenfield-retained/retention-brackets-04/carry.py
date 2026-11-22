"""Copy of counted-reset-power-02/carry.py with the lift duration read from LIFT_S (4.8 s as released).
Every other line is unchanged, so LIFT_S=4.8 reproduces the released carry byte for byte."""
from pathlib import Path
import sys
p=Path(__file__).resolve().parent;root=p.parent
sys.path[:0]=[str(root/'follower-01'),str(root/'loaded-actuation-01')]
from smooth import times,table
import os,json
LIFT=os.environ.get("LIFT_S","4.8")
seed=json.loads(Path(os.environ['CYCLE_STATE']).read_bytes())
def install_state(s):
    replacements={
      'travel=0.;motorLog=[];':"travel=seed['noseTravelM'];motorLog=[];",
      'speed=0.; commanded=':"speed=seed['noseSpeedMps']; commanded=",
      'beltVelocity=np.zeros(3);':"beltVelocity=np.array(seed['zonesMps'],dtype=float);",
      "    for step in range":"    flapJoint=jid('flap');d.qpos[model.jnt_qposadr[flapJoint]]=seed['flapRad'];d.qvel[model.jnt_dofadr[flapJoint]]=seed['flapVelocityRadS'];peakStopError=0.;maxReaction=0."+chr(10)+"    for step in range",
      '        if m.degrees(theta)>102:':"        model.jnt_range[fj,1]=cam;model.jnt_solref[fj]=[.002,1.];peakStopError=max(peakStopError,d.qpos[fq]-cam)"+chr(10)+"        if m.degrees(theta)>102:",
      'endZoneSpeeds=':"cycleStart=seed,noseTravelM=travel,stopViolationRad=peakStopError,endFlapVelocity=d.qvel[model.jnt_dofadr[jid('flap')]],endZoneSpeeds=",
    }
    for a,b in replacements.items():
        assert s.count(a)==1,(a,s.count(a))
        s=s.replace(a,b)
    return s
text=(root/'seat-interlock-01/recovery.py').read_text()
def sub(old,new):
    global text
    assert text.count(old)==1,old
    text=text.replace(old,new)
sub("assert not plan.exists() or plan.read_bytes()==(ref/'plan.json').read_bytes()","assert plan.exists()")
sub("if not plan.exists():plan.write_bytes((ref/'plan.json').read_bytes())","assert all(r['loaded'][-1]==[1.39,5.871] for r in json.loads(plan.read_bytes())['plan'])")
sub('(local[:,0]-.055)','(local[:,0]-.060)');sub('abs(local[:,1])-.050','abs(local[:,1])-.225')
sub('return interlock(original_split(source))','''s=install_state(interlock(original_split(source)))
    assert s.count('125-deploy')==2 and s.count('count=38;')==1 and s.count('mass=".07" priority')==1
    s=s.replace('(t-startLift-delay)/3.4','(t-startLift-delay)/'+LIFT).replace('rate=6*u*(1-u)/3.4','rate=6*u*(1-u)/'+LIFT)
    assert '(t-startLift-delay)/'+LIFT in s and 'rate=6*u*(1-u)/'+LIFT in s
    marker="        tj=jid('truck');"
    assert s.count(marker)==1 and s.count('    for step in range')==1
    s=s.replace('    for step in range',"    tabX,tabV=table(deploy,122,g['pickPivot'][0])"+chr(10)+'    for step in range')
    calc="        profileTime=(t-startLift-delay)*3.4/"+LIFT+";X=float(np.interp(profileTime,times,tabX));delta=1000*(X-robotX);vx=float(np.interp(profileTime,times,tabV))*3.4/"+LIFT
    s=s.replace(marker,calc+chr(10)+marker)
    return s.replace('125-deploy','122-deploy').replace('count=38;','count=76;').replace('mass=".07" priority','mass=".035" priority')''')
sub("exec(compile(finish,'seat confirmed carry','exec'))","""marker="loc.write_text(json.dumps(r,separators=(',',':')));print(key"
assert loaded.count(marker)==1
assert loaded.count('startLift+3.4')==1
loaded=loaded.replace('startLift+3.4','startLift+'+LIFT)
loaded=loaded.replace(marker,"after=[v for v in r['trajectory'] if v[0]>=2.82+r['seatDelay']];r.update(originalHeld=r['held'],held=bool(after) and not r['failedGate'] and all(.12<v[8]<.35 and .018<v[9]<.09 and abs(v[2])<.05 for v in after),endPitchDegrees=122,liftDurationS="+LIFT+",slideSmoothingS=.16*"+LIFT+"/3.4);"+marker)
exec(compile(finish,'seat confirmed carry','exec'))""")
exec(compile(text,'lifted cassette at revised approach','exec'))

