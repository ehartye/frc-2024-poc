"""Two independently driven longitudinal belt zones, each with a finite compliant contact patch."""
from pathlib import Path
import json,sys,numpy as np,hashlib,math
from patch import belt_patch,coefficients
p=Path(__file__).resolve().parent;root=p.parent/'six-piece-02';method=p.parent/'follower-01'
key,n,policyName=sys.argv[1:4];n=int(n);src=root/(f'{key}-{n}.json' if key.startswith(('R01-', 'R02-')) else f'dev-{key}-{n}.json');incoming=json.loads(src.read_bytes());priorHash=hashlib.sha256(src.read_bytes()).hexdigest()
assert incoming['held'] and incoming['seated']
# Instantiate the source-bound field model and the staged weak-pack arbitration.
base=(method/'fire.py').read_text();base=base[base.index('old=(p.parent/'):base.index('incoming=a;')]
extraSplit=r"""
start=src.index('            diff=local-nearest;ds=np.linalg.norm')
end=src.index('        maxCurrent=',start)
src=src[:start]+'''            # Independent upstream transport and downstream launcher, identical geometry on both sides.
            ffeed=0.;fshot=0.
            for low,high,spd,capF,tag in ((.097,.352,feedSpeed,8.,0),(.407,.600,speed,20.,1)):
                rail=np.column_stack((np.clip(local[:,0],low,high),np.full(n,side*.1898),np.clip(local[:,2],.032,.064)))
                wrench,N,force,arc,slipW=belt_patch(local-rail,pts,center,v,omega,R,spd,dt,[*params[:4],capF])
                wr+=wrench;normal+=N;peak=max(peak,abs(force))
                if tag==1:
                    fshot=force
                    if N>.2 and firstLaunchContact is None:firstLaunchContact=[t,belts.tolist(),feedSpeed]
                else:ffeed=force
            feedForce+=ffeed
            belts[jj]+=dt*(Fmotor-fshot-.4*speed)/inertia
        # Shared feed transmission, 4:1 reduction, 12 mm roller, 8 A total.
        command=-.40 if not feeding else 1.35
        feedI=np.clip((command-feedSpeed)*18,-8,8)
        feedSpeed+=dt*(feedI*.01*4*.7/.012-feedForce-.5*feedSpeed)/.18
        currents.append(abs(feedI)*min(1.,(.01*4*abs(feedSpeed)/.012+.35*abs(feedI))/max(7.,bus)))
'''+src[end:]
sub('    rows=[];coarse=[]','''    feedSpeed=0.;feeding=False;feedT=None;firstLaunchContact=None;proof=0.;seatedMax=0.;indexFirst=None;spinAtFeed=None
    rows=[];coarse=[]''')
sub('        currents=[]','''        readStop=any(model.geom(gg).name=='backstop' for ct in d.contact for gg in (ct.geom1,ct.geom2))
        seatedMax=max(seatedMax,float(readStop))
        if readStop and indexFirst is None:indexFirst=t
        permit=bool((min(belts)>=setpoint-.15 or splitPolicy=='unready') and enableT is not None and indexFirst is not None)
        proof=proof+dt if permit else 0.
        if not feeding and proof>=.04:feeding=True;feedT=t;spinAtFeed=belts.tolist()
        feedForce=0.;currents=[]''')
sub('if t>1.2 and enableT is None:break','if t>2.5 and not feeding:break')
sub('I=np.clip((bus-.0202*w)/.115,0,limit)', 'I=np.clip(min((bus-.0202*w)/.115,70.*(setpoint-speed)),0,limit)')
sub('result=dict(initialState=', 'result=dict(method="split drive",targetSpeed=setpoint,feedTime=feedT,rotorsAtFeed=spinAtFeed,firstLauncherContact=firstLaunchContact,stopAt=indexFirst,initialState=')
env.update(params=coefficients(),belt_patch=belt_patch,splitPolicy=policyName,setpoint=targetSpeed)
"""
targetSpeed=float(sys.argv[4]) if len(sys.argv)>4 else 7.35
base=base.replace("old=old.replace('env.update(policy=',extra+", "old=old.replace('env.update(policy=',extra+extraSplit+").replace("sys.argv=[__file__,'fire'+n", "sys.argv=[__file__,'split-'+policyName+'-'+key+'-'+str(n)+'-'+str(targetSpeed)")
base+="\nold=old.replace(\"'125','1.43'\",\"'125','1.53'\");exec(compile(old,'two drive zones','exec'))"
exec(compile(base,'physical split drive trial','exec'))

