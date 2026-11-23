from pathlib import Path
import runpy,sys,itertools,math,json
import numpy as np
q=Path(__file__).parent;w=runpy.run_path(str(q/'rack-fit.py'))
from types import SimpleNamespace
s=SimpleNamespace(**w); C=np.array(list(itertools.product([-1,1],repeat=3)))
def localBounds(p):
 # axis extrema for source BReps (mm, full precision padded outward). Protect revolved/curved extrema between topological vertices.
 bounds={'REV-41-1503':[-23.5620,-12.0616,-1.3001,23.5620,6.3751,11.5001],'REV-41-1332':[-12.0001,-12.0001,-7.5001,12.0001,12.0001,7.5001],'REV-21-1651':[-.001,-17.501,-17.501,56.001,25.412,25.500],'REV-41-1607':[-21.786,-.001,-19.001,21.786,5.001,19.001],'REV-41-1608':[-4.113,-4.126,-3.501,4.113,4.044,3.501],'REV-41-1602':[-21.786,-19.164,-11.876,21.786,19.001,6.001],'REV-41-1615':[-21.786,-19.001,-6.001,21.786,19.001,12.251]}
 b=np.array(bounds[p.s]).reshape(2,3);p0=np.vstack([p.A@a+p.t for a in (b[0]+(C+1)/2*(b[1]-b[0]))]);return [p0.min(axis=0),p0.max(axis=0)]
def sepXZ(c1,e1,t1,c2,e2,t2):
 d=c1-c2; peak=abs(d[1])-e1[1]-e2[1]
 for a in [t1,t1+np.pi/2,t2,t2+np.pi/2]:
  u=np.array([np.cos(a),0,-np.sin(a)])
  h1=e1[0]*abs(np.cos(a-t1))+e1[2]*abs(np.sin(a-t1))
  h2=e2[0]*abs(np.cos(a-t2))+e2[2]*abs(np.sin(a-t2))
  peak=max(peak,abs(u@d)-h1-h2)
 return peak
def trace(n):
 c=s.repo/f'head-clear-{n}';a=np.load(q/f'replay-{n}/toe.npy');rows=json.loads((c/'floor/cycle.json').read_bytes())['samples'];tm=np.array([i['t'] for i in rows]);joints={k:np.interp(tm,a[:,0],1000*a[:,j],left=0,right=-53 if k=='rear' else -23)for k,j in [('left',2),('right',5),('rear',8)]}
 return rows,joints
def shift(k,qs):return {'left':np.array([0,qs[0],0]),'right':np.array([0,-qs[1],0]),'rear':np.array([-qs[2],0,0]),None:np.zeros(3)}[k]
def scan(n,rz):
 items=s.make(rz=rz);rows,qs=trace(n);base=[localBounds(p) for p in items]; results={}; inter={}
 for i,r in enumerate(rows):
  joints=[qs[k][i]for k in ['left','right','rear']];ang=r['toolAngleRad'];rot=s.rot(ang);tr=np.array([1000*r['toolX'],0,r['toolMm']]);local=[]
  for p,(lo,hi) in zip(items,base):
   c=(lo+hi)/2+shift(p.k,joints);e=(hi-lo)/2;wc=rot@c+tr
   local.append([c,e])
   for label,meas in [('cover',sepXZ(wc,e,ang,*s.cover)),('leftRail',sepXZ(wc,e,ang,*s.rails[0])),('rightRail',sepXZ(wc,e,ang,*s.rails[1])),('floor',wc[2]-abs(np.sin(ang))*e[0]-abs(np.cos(ang))*e[2]),('forward',724.8-wc[0]-abs(np.cos(ang))*e[0]-abs(np.sin(ang))*e[2]),('backbone',max(abs(c-np.array([-47,0,95]))-e-np.array([63,100,6])))]:
    key=(p.name,label)
    if key not in results or meas<results[key][0]:results[key]=(meas,r['t'])
  for j,p in enumerate(items):
   for k,p2 in enumerate(items[:j]):
    left=p.name.startswith(('side ','left ','right '));other=p2.name.startswith(('side ','left ','right '))
    if left==other:continue
    c,e=local[j];cc,ee=local[k]
    m=max(abs(c-cc)-e-ee)
    key=(p.name,p2.name)
    if key not in inter or m<inter[key][0]:inter[key]=(m,r['t'])
 return dict(sampledPartGaps=[[*k,*v]for k,v in results.items()],crossDriveBoxContacts=[[*k,*v]for k,v in inter.items() if v[0]<0],samples=len(rows),sourceSHA={f:s.H(s.repo/f'head-clear-{n}'/f)for f in ['floor/cycle.json','fixture.xml']})
if __name__=='__main__':
 for n in [96,192]:
  for rz in [130,190,185]:
   v=scan(n,rz);(q/f'obb-{n}-{rz}.json').write_text(json.dumps(v,indent=2))
   print(n,rz,[(ob,min((x[2],x[0]) for x in v['sampledPartGaps'] if x[1]==ob)) for ob in ['cover','leftRail','rightRail','floor','forward','backbone']],v['crossDriveBoxContacts'],flush=True)


