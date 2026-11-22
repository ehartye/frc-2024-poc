"""Field-relative route, chassis acceleration and free-body carry from the registered floor pickup."""
from pathlib import Path
import sys,json,numpy as np,math,csv,hashlib
folder=Path(__file__).resolve().parent
method=folder.parent/'follower-01';sys.path.insert(0,str(method))
key,attempt,nArg=sys.argv[1:4];n=int(nArg)
plan=json.loads((folder/'plan.json').read_bytes());leg=next(k for k in plan['plan'] if k['id']==key)
roster=Path('C:/Users/ehart/repos/frc-2024-poc/sim/pressure/evidence/speaker-repeatability-starts.csv')
assert hashlib.sha256(roster.read_bytes()).hexdigest()==plan['roster']
def rot(q):return np.array([[math.cos(q),-math.sin(q)],[math.sin(q),math.cos(q)]])
qEntry=leg['yaw'];mark=np.array(leg['mark']);origin=np.array(leg['out'][-1])
offsets=np.zeros(2)
if attempt!='dev':
    row=next(a for a in csv.DictReader(roster.open()) if a['attempt']==attempt)
    offsets=np.array([float(row[key+'_'+ax+'_mm']) for ax in 'xy'])/1000@rot(qEntry)
duration=np.array(leg['loadedT']);path=np.array(leg['loaded']);ends=np.cumsum(duration)
startLift=2.82+(duration[0] if key=='middle' else 0.)
endTime=max(2.82+sum(duration),startLift+3.4)+.78
def pathAt(t):
    tt=t-2.82
    if tt<0:return path[0],qEntry,np.zeros(2),0.,0.
    if tt>=ends[-1]:return path[-1],0.,np.zeros(2),0.,0.
    i=int(np.searchsorted(ends,tt,side='right'));T=duration[i];u=(tt-(ends[i-1] if i else 0))/T
    b=u*u*(3-2*u);bd=6*u*(1-u)/T;acc=(6-12*u)/T**2
    dyaw=-qEntry if key=='middle' and i==1 else 0.
    yaw=(qEntry if key=='middle' and i<2 else 0.)+dyaw*b
    return path[i]+(path[i+1]-path[i])*b,yaw,(path[i+1]-path[i])*acc,dyaw*bd,dyaw*acc
def inertial(model,d,body,advance,ac,om,al):
    # Wrench in the translating and yawing chassis frame at the moving body's COM.
    # MuJoCo free-joint angular velocities are body-local; objectVelocity(...,0) is world oriented.
    v=np.empty(6);mj.mj_objectVelocity(model,d,mj.mjtObj.mjOBJ_BODY,body,v,0)
    r=d.xipos[body]-[advance,0,0];W=np.array([0,0,om]);A=np.array([0,0,al])
    force=-model.body_mass[body]*(ac+np.cross(A,r)+np.cross(W,np.cross(W,r))+2*np.cross(W,v[3:]))
    ax=d.ximat[body].reshape(3,3);I=ax@np.diag(model.body_inertia[body])@ax.T
    w=v[:3];torque=-I@(A+np.cross(W,w))-np.cross(w+W,I@(w+W))+np.cross(w,I@w)
    return np.r_[force,torque]
script=(method/'carry_cam.py').read_text()
script=script[:script.index('values=np.array')]
script=script.replace("folder/'response.json'","method/'response.json'").replace("'+50","'+50")
script=script.replace("'0','0','0','.45'","'0','0',str(offsets[1]),'.45'")
script=script.replace("round(7.0/dt)","round(endTime/dt)").replace("(t-2.82)/3.4","(t-startLift)/3.4")
extra="""
sub('.8666309 {offset}', '{.8666309+offsets[0]} {offset}')
sub('d.xfrc_applied[bi]=wr', '''bodyPos,yaw,accField,om,al=pathAt(t)
        if np.any(accField) or om or al:
            accBody=np.r_[rot(yaw).T@accField,0.]
            wr+=inertial(model,d,bi,advance,accBody,om,al)
            fb=model.jnt_bodyid[jid('flap')]
            d.xfrc_applied[fb]=inertial(model,d,fb,advance,accBody,om,al)
        else:
            d.xfrc_applied[model.jnt_bodyid[jid('flap')]]=0.
        d.xfrc_applied[bi]=wr''')
"""
script=script.replace("source=source.replace('exec(src);r=run',patch+", "source=source.replace('stroke=.44','stroke=leg[\\'advance\\']');source=source.replace('exec(src);r=run',patch+extra+")
sys.argv=[__file__,attempt+'-'+key+'-'+str(n),str(n)]
exec(compile(script,'carried NOTE in accelerating chassis','exec'))
rows=np.array([[v[0],v[8],v[9],v[2]] for v in r['trajectory'] if v[0]>=2.82])
r.update(id=key,attempt=attempt,startLift=startLift,finish=endTime,route=leg,
      placedNoteOffset=offsets.tolist(),held=bool(np.all((rows[:,1]>.12)&(rows[:,1]<.35)&(rows[:,2]>.018)&(rows[:,2]<.09)&(abs(rows[:,3])<.05))),
      rangeLocal=[rows[:,1:].min(axis=0).tolist(),rows[:,1:].max(axis=0).tolist()])
loc.write_text(json.dumps(r,separators=(',',':')));print(key,r['held'],r['rangeLocal'])

