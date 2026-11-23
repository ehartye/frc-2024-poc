from pathlib import Path
import runpy,json,numpy as np
q=Path(__file__).parent;w=runpy.run_path(str(q/'support.py'));P=w['w']['make'](rx=-106,rz=200);pa=next(i for i in P if i.name=='left seg 0');rack=next(iter(w['load_solids'](q.parent/'REV-41-1503.step')[0].values()));rack=w['xf'](rack,pa.A,pa.t)
base=52.25-w['w']['D']/2
def land(w0,rad):
 p=np.array([2.,base+16.,153.45]);n=np.array([1.,0.,0.]);s=w['cyl'](p+n*w0,n,rad,3.02-w0);return w['cut'](s,w['cyl'](p-n,n,1.7,8))
o=[]
for rad in [2.35,2.45,2.55,2.65,2.75,2.85,3.0]:
 s=land(0,rad);o.append([rad,w['common'](s,rack),w['distance'](s,rack),w['common'](w['xf'](s,np.eye(3),[-.02,0,0]),rack)])
print(o);(q/'rack-land-radius.json').write_text(json.dumps(o,indent=1))
