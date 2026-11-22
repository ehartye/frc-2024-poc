import json,sys,math,numpy as np,runpy
from pathlib import Path
q=Path(__file__).parent;w=runpy.run_path(str(q/'rack-fit.py'));b=runpy.run_path(str(q/'bounds-fit.py'))
for n in [96,192]:
 rows,J=b['trace'](n);worst={}
 for name in ['side pinion','side motor','rear pinion','rear motor','rear seg 2','left seg 0']:
  p=next(p for p in w['make'](rz=185)if p.name==name);lo,hi=b['localBounds'](p);lo=np.array(lo);hi=np.array(hi)
  for i,r in enumerate(rows):
   angles=r['toolAngleRad'];R=w['rot'](angles);xy=1000*np.array([*r['robotXY'],0]);center=(np.array(r['centerMm'])-xy-np.array([1000*r['toolX'],0,r['toolMm']]))@R-b['shift'](p.k,[J[k][i]for k in ['left','right','rear']]);gap=np.linalg.norm(np.maximum(np.maximum(lo-center,center-hi),0))-177.8
   if name not in worst or gap<worst[name][0]:worst[name]=[round(gap,3),r['t']]
 print(n,worst)
