"""Plan rest-to-rest swerve paths through the registered field with finite parked NOTES."""
from pathlib import Path
import numpy as np,json,hashlib,math,heapq,csv,sys
p=Path(__file__).resolve().parent;parent=p.parent/'follower-01'
season=Path('R:/sim/pressure')
sys.path.insert(0,str(season));from wing_field_frame import SUPPLY_MARKS
sys.path.insert(0,str(parent));from stroke import scheduled, mat
def read(f):return json.loads(f.read_bytes())
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
field=Path('R:/2024/six-speaker-20261118/field-source/whole-field-sliced-bounds.json')
fielddata=read(field);assert fielddata['source']=='42c91b3efb9521d059c13cf25e2d7dda6315bdd6b7eb1202847bfe84ffb43c1a'
from ast import parse,literal_eval,Assign
syntax=parse((season/'field_route_clearance.py').read_text())
ignore=set.union(*(literal_eval(k.value) for k in syntax.body if isinstance(k,Assign) and k.targets[0].id in ['SURFACES','GAME_PIECES']))
design=read(parent/'review-data.json');cs=next(c for c in design['cases'] if c['key']=='pickup-to-shot')
vs={role:np.vstack([design['bodies'][n]['v'] for n,g in cs['rig'].items() if g==role and n not in cs['removed']]) for role in ['arm','frame','slide','flap']}
def envelope():
    low=np.ones(3)*1000;high=-low
    for j in range(0,len(cs['time']),2):
        for role,v in vs.items():
            mx=np.array(cs['series'][role][j]).reshape(3,4);w=v@mx[:,:3].T+mx[:,3];low=np.minimum(low,w.min(axis=0));high=np.maximum(high,w.max(axis=0))
    return low/1000-.013,high/1000+.013
lo,hi=envelope()
# Physical stock near the heights of dormant NOTES plus a 3 mm geometric allowance.
def low_stock():
    points=[]
    for j in range(len(cs['time'])):
        for role,v in vs.items():
            mx=np.array(cs['series'][role][j]).reshape(3,4);w=v@mx[:,:3].T+mx[:,3]
            if w[:,2].min()<=54:points.extend(w[w[:,2]<=56,:2])
    a=np.asarray(points)/1000
    # Outer envelope also includes the bumper edges at Z52 and full deployed blade.
    return a.min(axis=0)-.003,a.max(axis=0)+.003
pl,ph=low_stock()
fieldob=[];bands=[(0,.09),(.09,.22),(.22,.45),(.45,.7),(.7,.9),(.9,1.05),(1.05,hi[2])]
bandpts=[[] for _ in bands];lowbands=[[] for _ in bands]
for role in vs:
    group=[np.array(design['bodies'][n]['v']) for n,g in cs['rig'].items() if g==role and n not in cs['removed']]
    jj=list(range(0,312,3))+[350] if role!='frame' else [0]
    for j in jj:
        mx=np.array(cs['series'][role][j]).reshape(3,4)
        for v in group:
            w=(v@mx[:,:3].T+mx[:,3])/1000;low=w.min(axis=0)-.035;high=w.max(axis=0)+.035
            for i,(a,b) in enumerate(bands):
                if low[2]<=b and high[2]>=a:
                    corners=[[x,y] for x in [low[0],high[0]] for y in [low[1],high[1]]]
                    bandpts[i].extend(corners)
                    if role=='frame' or cs['time'][j]<=3.92:lowbands[i].extend(corners)
from scipy.spatial import ConvexHull
lowbands=[np.array(v)[ConvexHull(v).vertices] if v else np.empty((0,2)) for v in lowbands]
bandpts=[np.array(v)[ConvexHull(v).vertices] if v else np.empty((0,2)) for v in bandpts]
for row in fielddata['parts']:
    if row['path'][1] in ignore:continue
    for a,b in row.get('tightBounds',[row['bounds']]):
        if a[2]<hi[2] and b[2]>0.0001 and b[0]>0 and a[0]<10.7 and b[1]>2.5 and a[1]<8.2:fieldob.append([a,b])
marks={k:np.array(v)*.0254 for k,v in SUPPLY_MARKS.items()}
station=np.array([1.53,5.55]);order=['middle','outer','inner','center-high','center-mid','center-low']
def yaw(q):return np.array([[math.cos(q),-math.sin(q)],[math.sin(q),math.cos(q)]])
def heading(k):return math.radians(-115 if k=='middle' else 0)
def dock(k):return marks[k]-yaw(heading(k))[:,0]*.8666309
def clock(L,acc=2.6,speed=2.7):
    if L<1e-9:return 0.
    # Cubic smoothstep reaches a maximum 1.5 L/T and 6 L/T^2.
    return max(1.5*L/speed,math.sqrt(6*L/acc))
class Grid:
    def __init__(self,idle,q,turn=False,low=False):
        corners=np.array([[x,y] for x in [lo[0],hi[0]] for y in [lo[1],hi[1]]])
        cs=np.vstack([corners@yaw(phi).T for phi in (np.arange(min(q,0),max(q,0)+.01,.01) if turn else [q])])
        lower=cs.min(axis=0)-.012;upper=cs.max(axis=0)+.012
        self.pieces=np.array([marks[k] for k in idle]).reshape(-1,2);self.rot=yaw(q);self.turn=turn
        boxes=np.array(fieldob);allA=[];allB=[]
        for i,(z0,z1) in enumerate(bands):
            part=(lowbands if low else bandpts)[i]
            if not len(part):continue
            shape=np.vstack([part@yaw(phi).T for phi in (np.arange(min(q,0),max(q,0)+.01,.01) if turn else [q])])
            sel=boxes[(boxes[:,0,2]<z1)&(boxes[:,1,2]>z0)]
            allA.extend(sel[:,0,:2]-shape.max(axis=0)-.005);allB.extend(sel[:,1,:2]-shape.min(axis=0)+.005)
        a=np.array(allA);b=np.array(allB)
        # Remove contained/duplicate rectangles in the planning area.
        keep=(b[:,0]>=.6)&(a[:,0]<=10.1)&(b[:,1]>=2.8)&(a[:,1]<=8.25)
        z=np.unique(np.round(np.concatenate([a[keep],b[keep]],axis=1),7),axis=0)
        keep=[]
        for i,v in enumerate(z):
            if not np.any(np.all(z[:,:2]<=v[:2],axis=1)&np.all(z[:,2:]>=v[2:],axis=1)&(np.arange(len(z))!=i)):keep.append(v)
        z=np.array(keep);self.lower=z[:,:2];self.upper=z[:,2:]
    def clear(self,a,b):
        vel=b-a;low=self.lower;upper=self.upper;mn=np.zeros(len(low));mx=np.ones(len(low))
        for k in [0,1]:
            if abs(vel[k])<1e-8:mn=np.where((a[k]<low[:,k])|(a[k]>upper[:,k]),2,mn)
            else:
                u=(low[:,k]-a[k])/vel[k];v=(upper[:,k]-a[k])/vel[k]
                mn=np.maximum(mn,np.minimum(u,v));mx=np.minimum(mx,np.maximum(u,v))
        if np.any(mn<=mx):return False
        # Transform the obstacle centers into the robot heading. Distances from
        # the segment to a rectangle include the finite game-piece radius.
        for pt in self.pieces:
            if self.turn:
                t=np.clip(np.dot(pt-a,vel)/max(1e-12,np.dot(vel,vel)),0,1)
                if np.linalg.norm(pt-a-t*vel)<.188+max(np.linalg.norm([x,y]) for x in (lo[0],hi[0]) for y in (lo[1],hi[1])):return False
                continue
            p0=(pt-a)@self.rot;v=-vel@self.rot
            ts=[0.,1.]
            for j in (0,1):
                if abs(v[j])>1e-10:ts.extend(np.clip((np.array([pl[j],ph[j]])-p0[j])/v[j],0,1))
            ts=sorted(ts);check=list(ts)
            for l,h in zip(ts,ts[1:]):
                mid=p0+v*(l+h)*.5
                mask=(mid<pl)|(mid>ph);edge=np.clip(mid,pl,ph);den=np.dot(v[mask],v[mask])
                if den:check.append(np.clip(np.dot((edge-p0)[mask],v[mask])/den,l,h))
            for t in check:
                at=p0+t*v
                if np.linalg.norm(at-np.clip(at,pl,ph))<.188:return False
        return True
    def path(self,a,b):
        if not self.clear(a,a) or not self.clear(b,b):raise ValueError(('endpoint',a,b,self.clear(a,a),self.clear(b,b)))
        if self.clear(a,b):return [a,b]
        step=.22;cost={(0,0):0};back={};queue=[(0,0,(0,0))]
        while queue:
            _,g,k=heapq.heappop(queue);pos=a+step*np.array(k)
            if g>cost[k]+1e-8:continue
            if self.clear(pos,b):break
            for dx,dy in ((1,0),(0,1),(-1,0),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                kk=(k[0]+dx,k[1]+dy);end=a+step*np.array(kk);c=g+step*math.hypot(dx,dy)
                if c<cost.get(kk,1e4) and all([.6<end[0]<10.1,2.8<end[1]<8.15]) and self.clear(pos,end):
                    cost[kk]=c;back[kk]=k;heapq.heappush(queue,(c+np.linalg.norm(end-b),c,kk))
        else:raise ValueError('route exhausted')
        path=[b]
        while k!=(0,0):path.append(a+step*np.array(k));k=back[k]
        path.append(a);path.reverse();out=[a];i=0
        while i<len(path)-1:i=max(j for j in range(i+1,len(path)) if self.clear(path[i],path[j]));out.append(path[i])
        return out
def plan():
    rows=[];idle=order.copy();first=marks['middle']-yaw(heading('middle'))[:,0]*.658;origin=first.copy()
    for k in order:
        dest=dock(k);q=heading(k);advance=.35 if k=='inner' else .44;active=dest+yaw(q)[:,0]*advance
        idle.remove(k);G=Grid(idle,q,low=True);out=G.path(origin,dest);assert G.clear(dest,active),(k,dest,active)
        # Initial piece is acquired at the first station. Subsequent transits must avoid it before intentional approach.
        if k!='middle':
            Gfull=Grid(idle+[k],q,low=True)
            if Gfull.clear(dest,dest):out=Gfull.path(origin,dest)
        H=Grid(idle,q,turn=abs(q)>.01);ret=H.path(active,station) if k!='middle' and H.clear(active,active) else []
        if not ret:
            # Backtrack at intake pitch while still holding the collected piece.
            for safe in sorted([np.array([x,y]) for x in np.arange(min(active[0],station[0])-.1,active[0]+.7,.15) for y in np.arange(active[1]-1.1,active[1]+1.11,.15)],key=lambda v:np.linalg.norm(active-v)+.12*np.linalg.norm(v-station)):
                if G.clear(active,safe) and H.clear(safe,safe):
                    ret=[active,*([safe] if abs(q)>.01 else []),*Grid(idle,0).path(safe,station)];break
            if not ret:raise ValueError(('no safe lift',active))
        outT=[clock(np.linalg.norm(b-a)) for a,b in zip(out,out[1:])]
        retT=[clock(np.linalg.norm(b-a)) for a,b in zip(ret,ret[1:])]
        if abs(q)>.01:retT[1]=max(retT[1],clock(abs(q),2.8,2.3))
        rows.append(dict(id=k,advance=advance,mark=marks[k].tolist(),yaw=q,out=np.array(out).tolist(),outT=outT,loaded=np.array(ret).tolist(),loadedT=retT));origin=station
    return rows
if __name__=='__main__':
    rows=plan();r=dict(field=sha(field),placedStep=design['step'],roster=sha(season/'evidence/speaker-repeatability-starts.csv'),envelope=[lo.tolist(),hi.tolist()],plan=rows)
    (p/'plan.json').write_text(json.dumps(r,indent=2));print([(v['id'],len(v['out']),len(v['loaded']),round(sum(v['outT'])+sum(v['loadedT']),2)) for v in rows])








