"""Check the displayed NOTE at every replay frame against the actual triangle meshes."""
from pathlib import Path
import sys,json,math,hashlib,numpy as np,vtk
from vtk.util.numpy_support import numpy_to_vtk,vtk_to_numpy,numpy_to_vtkIdTypeArray
p=Path(__file__).resolve().parent;root=p.parent
read=lambda f:json.loads(f.read_bytes())
doc=read(root.parent/'www/data.json');g=doc['concepts']['guided'];field=doc['concepts']['field'];svc=doc['slideService']
stock={k:v for k,v in doc['bodies'].items() if k not in field['remove']};stock.update(field['add']);stock.update(g['add']);stock.update(svc['add'])
def rot(a):
    co=np.cos(np.radians(a));sn=np.sin(np.radians(a));return np.array([[co,0,-sn],[0,1,0],[sn,0,co]])
phi=np.arange(128)*2*np.pi/128
ring=np.c_[215+152.4*np.cos(phi),152.4*np.sin(phi),np.full(len(phi),50.4)]@rot(125).T+[185,0,501]
def locator(b):
    pts=vtk.vtkPoints();pts.SetData(numpy_to_vtk(np.array(b['v'],dtype=float),deep=True));poly=vtk.vtkPolyData();poly.SetPoints(pts)
    faces=np.array(b['f'],dtype=np.int64);cells=vtk.vtkCellArray();cells.SetData(numpy_to_vtkIdTypeArray(np.arange(0,faces.size+1,3,dtype=np.int64),deep=True),numpy_to_vtkIdTypeArray(faces.flatten(),deep=True));poly.SetPolys(cells)
    fun=vtk.vtkImplicitPolyDataDistance();fun.SetInput(poly);return fun
def full(rows):
    return np.array(rows).reshape(-1,3,4)
for res in [48,96]:
    c=next(v for v in g['cases'] if v['key']==f'continuous-cycle-{res}');rig={**c['rig'],**{n:'frame' for n in svc['add']}};rm=set(c['removed']+g['fixtureOnlyNames']+['Chambered NOTE'])
    P=full(c['series']['piece']);Q=np.einsum('tij,kj->tki',P[:,:,:3],ring)+P[:,:,3][:,None,:]
    rows=[];stats=[]
    for group in set(rig.values())-{'piece'}:
        tr=full(c['series'][group]);local=np.einsum('tkj,tji->tki',Q-tr[:,:,3][:,None,:],tr[:,:,:3])
        low=local.min(axis=1)-25.4;high=local.max(axis=1)+25.4
        for n,b in stock.items():
            if n in rm or rig.get(n)!=group:continue
            v=np.array(b['v']);lo=v.min(axis=0);hi=v.max(axis=0)
            tix=np.flatnonzero(np.all((low<=hi)&(high>=lo),axis=1))
            if not len(tix):continue
            qq=local[tix];dd=np.linalg.norm(np.maximum(0,np.maximum(lo-qq,qq-hi)),axis=2)
            ix,j=np.where(dd<25.4);coords=np.ascontiguousarray(qq[ix,j]);tt=tix[ix]
            if not len(tt):continue
            fun=locator(b);value=vtk.vtkDoubleArray();fun.EvaluateFunction(numpy_to_vtk(coords,deep=True),value);dist=abs(vtk_to_numpy(value))
            depths=25.4-dist;best=int(np.argmax(depths));ti=int(tt[best])
            stats.append(dict(part=n,penetrationMm=float(depths[best]),t=c['time'][ti],phase=c['measure'][ti],frame=ti))
            if depths[best]>.01:
                deep=np.full(len(c['time']),-1.);np.maximum.at(deep,tt,depths)
                for ix in np.flatnonzero(deep>.01):
                    rows.append([n,c['time'][ix],round(float(deep[ix]),3),c['measure'][ix]])
    out=dict(case=c['key'],samples=len(c['time']),parts=len(rig.keys()-rm),circleSamples=len(phi),maxima=sorted(stats,key=lambda x:-x['penetrationMm']),penetrations=rows,scope='Displayed triangulated occupied surfaces at saved presentation times, using the registered torus centerline and 25.4 mm circular section. Positive depth detects nominal surface intrusion. Discretization can miss contacts. Distinguish allowable compliance from hard-stock contacts using CAD.')
    (p/f'view-contact-{res}.json').write_text(json.dumps(out,separators=(',',':')))
    print(res,[(r['part'],round(r['penetrationMm'],2),r['t']) for r in out['maxima'][:20]],flush=True)

