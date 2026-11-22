from pathlib import Path
import numpy as np, json,hashlib,math,itertools
q=Path(__file__).parent; repo=Path.home()/'repos/frc-2024-poc-worktrees/engineering-iterations/iterations/002/design/over-bumper-pickup'
def H(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def pts(s):
 p=np.load(q/(s+'-points.npy'))
 return p
class Part:
 def __init__(self,s,tag,A,t,k=None):self.name=tag;self.s=s;self.A=np.array(A,float);self.t=np.array(t,float);self.k=k;self.points=pts(s)@self.A.T+self.t;assert np.linalg.det(self.A)>.9999
D=47.12389; gear='REV-41-1332';rack='REV-41-1503'
def make(count=3,rx=-92,rz=130,ry=-25):
 out=[]
 def P(s,tag,A,t,k=None):out.append(Part(s,tag,A,t,k))
 # b = pinion axial direction away from its drive; u,v = pinion radial axes.
 def geartrain(tag,ct,u,v,b):
  center=np.array(ct); U=np.array(u);V=np.array(v);B=np.array(b); cols=lambda a,c,d:np.stack([a,c,d],axis=1)
  P(gear,tag+' pinion',cols(U,V,B),ct)
  # Assumed face planes from supplier bodies. Output is 5mm behind inner bearing center -12; motor face 35.25 further.
  face=center-17*B
  P('REV-41-1615',tag+' output',cols(U,V,B),face-12.25*B)
  P('REV-41-1602',tag+' stage',cols(U,V,B),face-(12.25+6)*B)
  P('REV-41-1607',tag+' input plate',cols(U,B,-V),face-(12.25+11+5)*B)
  mf=face-(12.25+11+5)*B
  P('REV-21-1651',tag+' motor',cols(-B,U,-V),mf+11.5*B)
  P('REV-41-1608',tag+' sun',cols(U,V,B),mf+8*B)
 gx=-4;gz=132
 geartrain('side',[gx,0,gz],[0,1,0],[0,0,1],[1,0,0])
 # outer edge at y=+/- 86.25mm at full deploy, 25.75mm inboard from post.
 for sign in [1,-1]:
  for i in range(2):
   ay=sign*(52.25-D*(i+.5))
   P(rack,('left' if sign==1 else 'right')+' seg '+str(i),[[0,0,-1],[sign,0,0],[0,-sign,0]],[gx+6,ay,gz+sign*(11.25+5.625)],'left' if sign==1 else 'right')
 geartrain('rear',[rx,ry,rz],[1,0,0],[0,0,-1],[0,1,0])
 # Rear rack teeth point down. Use non-mirrored local profile (u points -X, v down Z, w -Y).
 for i in range(count):
  P(rack,'rear seg '+str(i),[[-1,0,0],[0,0,-1],[0,-1,0]],[-41-D*(i+.5),ry+6,rz+11.25+5.625],'rear')
 return out
cover=(np.array([461.775,0,122]),np.array([42.275,350.9,64.6]),0)
rails=[(np.array([-141.49630645,sign*33.3,465.8862032]),np.array([212,12.7,6.35]),math.pi/3) for sign in [-1,1]]
def rot(angle):
 c=np.cos(angle);s=np.sin(angle);return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def calc(parts,n):
 c=repo/f'head-clear-{n}'; inp=json.loads((c/'floor/cycle.json').read_bytes())['samples'];a=np.load(q/f'replay-{n}/toe.npy')
 t=np.array([i['t'] for i in inp]);trans=np.array([[1000*i['toolX'],0,i['toolMm']]for i in inp]);angle=np.array([i['toolAngleRad']for i in inp]);s=np.sin(angle);co=np.cos(angle)
 joint={k:np.interp(t,a[:,0],1000*a[:,j],left=0,right=(-53 if k=='rear' else -23))for k,j in [('left',2),('right',5),('rear',8)]}
 allrows=[]
 for part in parts:
  p=part.points
  rm=np.zeros((len(t),3))
  if part.k=='rear':rm[:,0]=-joint[part.k]
  if part.k in ('left','right'):rm[:,1]=joint[part.k]*(1 if part.k=='left' else -1)
  local=p[:,None,:]+rm[None,:,:]
  world=np.stack([local[:,:,0]*co+local[:,:,2]*s, local[:,:,1],-local[:,:,0]*s+local[:,:,2]*co],axis=2)+trans[None,:,:]
  by={}
  for what,(ct,ext,theta)in zip(['cover','rail0','rail1'],[cover,*rails]):
   rel=(world-ct)@rot(theta)
   depth=(ext-abs(rel)).min(axis=2)
   j,h=np.unravel_index(depth.argmax(),depth.shape)
   by[what]=dict(maxVertexInsideDepthMm=float(depth[j,h]),t=t[h],point=world[j,h].tolist(),sample=int(h),sourceVertex=int(j))
  j,h=np.unravel_index(world[:,:,0].argmax(),world[:,:,0].shape)
  allrows.append(dict(part=part.name,source=part.s,sourceToHeadA=part.A.tolist(),baseTranslation=part.t.tolist(),joint=part.k,witness=by,peakX=world[j,h,0],peakXTime=t[h],minZ=world[:,:,2].min()))
 return dict(rows=allrows,cycle=H(c/'floor/cycle.json'),fixture=H(c/'fixture.xml'),toe=H(q/f'replay-{n}/toe.npy'),countSamples=len(t))
if __name__=='__main__':
 for params,label in [(dict(),'untrimmed')]:
  for n in [96,192]:
   r=calc(make(**params),n);(q/f'fit-{label}-{n}.json').write_text(json.dumps(r,indent=2))
   for x in r['rows']:
    print(n,x['part'],round(x['peakX'],1),*[round(x['witness'][i]['maxVertexInsideDepthMm'],1) for i in ['cover','rail0','rail1']])

