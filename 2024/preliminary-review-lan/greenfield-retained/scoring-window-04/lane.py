"""Fire the carried NOTE from a stated station, cassette angle and chassis heading. Copy of
counted-reset-power-02/lane.py; the pose insert below re-poses launcher and seated NOTE together."""
from pathlib import Path
p=Path(__file__).resolve().parent
raw=(p/'shot.py').read_text()
exec(compile(raw[:raw.index("src=(root/'split-cycle-01")],'same deployed field and captured NOTE','exec'))
sys.argv[3]='outerlane'
build=(root/'split-cycle-01/split_fire.py').read_text().split("exec(compile(script,'fire from carried state','exec'))")[0]
exec(compile(build,'construct powered launcher','exec'))
assert script.count("'125','1.53'")==1
script=script.replace("'125','1.53'",repr(ANGLE)+','+repr(X))
build=script.split("exec(compile(base,'physical split drive trial','exec'))")[0]
exec(compile(build,'construct launcher and power arbitration','exec'))
mark="exec(compile(old,'two drive zones','exec'))"
assert base.count(mark)==1
base=base.replace(mark,"assert old.count("+repr("'5.55'")+")==1;old=old.replace("+repr("'5.55'")+","+repr(repr(Y))+");"+mark)
POSE=r"""
sub('q0=[math.cos(a/2),0,-math.sin(a/2),0]','q0=[math.cos(a/2),0,-math.sin(a/2),0];yw=math.radians(float(__import__("os").environ.get("W_YAW","0")));Rz=np.array([[math.cos(yw),-math.sin(yw),0],[math.sin(yw),math.cos(yw),0],[0,0,1.]]);R=Rz@R;pivot=np.array([robotX,robotY,0.])+Rz@np.array([.185,0,.501]);qz=np.array([math.cos(yw/2),0,0,math.sin(yw/2)]);qn=np.zeros(4);mj.mju_mulQuat(qn,qz,np.array(q0,dtype=float));q0=qn.tolist()')
sub(';mj.mj_forward(model,d)',';dA=math.radians(angle-122.);p0=np.array([robotX+.185,robotY,.501]);dR=np.array([[math.cos(dA),0,-math.sin(dA)],[0,1.,0],[math.sin(dA),0,math.cos(dA)]]);pp=p0+dR@(d.qpos[:3]-p0);qa=np.zeros(4);mj.mju_mulQuat(qa,np.array([math.cos(dA/2),0,-math.sin(dA/2),0]),d.qpos[3:7].copy());c0=np.array([robotX,robotY,0.]);d.qpos[:3]=c0+Rz@(pp-c0);qb=np.zeros(4);mj.mju_mulQuat(qb,qz,qa);d.qpos[3:7]=qb;mj.mj_forward(model,d)')
sub('friction=.45,belt_speed','friction=float(__import__("os").environ.get("W_FIELD_MU",".45")),belt_speed')
sub('int((7 if drive else .55)/dt)','int((float(__import__("os").environ.get("W_DURATION","7")) if drive else .55)/dt)')
"""
needle="env.update(incoming=incoming,priorHash=priorHash)"
assert base.count(needle)==1
base=base.replace(needle,POSE+needle)
exec(compile(base,'constant lateral offset trial','exec'))

