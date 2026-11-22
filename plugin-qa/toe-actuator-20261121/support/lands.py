from pathlib import Path
import runpy,json,numpy as np
q=Path(__file__).parent;w=runpy.run_path(str(q/'support.py'));P=w['w']['make'](rx=-106,rz=200);pa=next(i for i in P if i.name=='left seg 0');rack=next(iter(w['load_solids'](q.parent/'REV-41-1503.step')[0].values()));rack=w['xf'](rack,pa.A,pa.t)
base=52.25-w['w']['D']/2
def land(w0):
 p=np.array([2.,base+16.,153.45]);n=np.array([1.,0.,0.]);s=w['cyl'](p+n*w0,n,3.25,3.02-w0);return w['cut'](s,w['cyl'](p-n,n,1.7,8))
o=[]
for z in [0,.25,.5,1,1.29,1.3,1.31]:
 s=land(z);o.append(dict(fromPadDatumMm=z,commonMm3=w['common'](s,rack),nearestMm=w['distance'](s,rack)))
# Register a cut-to-actual pad, and verify 0.02 mm approach yields nonzero bearing area (nominal small-depth probe alone cannot qualify load).
for z in [0,.25,1.3]:
 s=w['cut'](land(z),rack);res=[]
 for delta in [-.02,0,.02]:
  part=w['xf'](s,np.eye(3),[delta,0,0]);res.append([delta,w['common'](part,rack),w['distance'](part,rack)])
 o.append(dict(pocketFrom=z,seating=res))
print(o);(q/'rack-pad-lands.json').write_text(json.dumps(o,indent=1))
