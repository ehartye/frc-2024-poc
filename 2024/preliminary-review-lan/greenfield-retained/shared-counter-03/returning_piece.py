"""Rebind an outgoing NOTE's return to the full parked chassis at the original scoring station."""
from pathlib import Path
import hashlib
exec(compile((Path(__file__).with_name('view_clipping.py')).read_text().split('for res in [48,96]:')[0],'shared viewer mesh loader','exec'))
from scipy.spatial.transform import Rotation
station=np.array([1.39,5.871,0.])
case=next(v for v in g['cases'] if v['key']=='continuous-cycle-96')
rig={**case['rig'],**{n:'frame' for n in svc['add']}}
rm=set(case['removed']+g['fixtureOnlyNames']+['Chambered NOTE'])
phis=np.arange(192)*2*np.pi/192
points=np.column_stack([152.4*np.cos(phis),152.4*np.sin(phis),np.zeros(len(phis))])
report=[]
for tag in ['R02-inner','R06-middle','R01-inner']:
    sf=root/'counted-reset-power-02'/f'split-tail_.25_0-{tag}-96-6.88.json';s=read(sf)
    rows=[r for r in s['trace'] if r['t']>s['opening']['fullyThroughByS']]
    t=np.array([r['t'] for r in rows]);q=np.array([r['q'] for r in rows])
    rr=Rotation.from_quat(q[:,[4,5,6,3]]).as_matrix()
    Q=np.einsum('tij,kj->tki',rr,points)+1000*(q[:,:3]-station)[:,None,:]
    hits=[];parts=0
    for name,b in stock.items():
        if name in rm or rig.get(name)!='frame':continue
        parts+=1
        v=np.array(b['v']);lo=v.min(0);hi=v.max(0)
        idx=np.flatnonzero(np.all((Q.min(1)-25.4<=hi)&(Q.max(1)+25.4>=lo),axis=1))
        if not len(idx):continue
        dist=np.linalg.norm(np.maximum(0,np.maximum(lo-Q[idx],Q[idx]-hi)),axis=2)
        ii,jj=np.where(dist<=25.4)
        if not len(ii):continue
        xyz=np.ascontiguousarray(Q[idx[ii],jj])
        out=vtk.vtkDoubleArray();locator(b).EvaluateFunction(numpy_to_vtk(xyz,deep=True),out)
        gap=np.abs(vtk_to_numpy(out))-25.4
        mask=gap<-.02
        if not mask.any():continue
        j=int(np.flatnonzero(mask)[0]);deep=int(gap.argmin())
        hits.append(dict(part=name,firstLocalS=float(t[idx[ii[j]]]),penetrationAtFirstSampleMm=float(-gap[j]),deepestMm=float(-gap[deep]),maxDepthAtLocalS=float(t[idx[ii[deep]]])))
    report.append(dict(input=hashlib.sha256(sf.read_bytes()).hexdigest(),tag=tag,parts=parts,samples=len(rows),first=sorted(hits,key=lambda v:v['firstLocalS'])[:4],hits=hits))
result=dict(stationM=station.tolist(),scope='Post-opening shared registration to fixed stock only. Intrusion at any sampled material site is a positive geometric control. The field-only path becomes physically unevaluated after the first interference; deeper intersections on that same trajectory are diagnostic only.',cases=report)
(p/'return-to-robot.json').write_text(json.dumps(result,indent=2))
print([(v['tag'],len(v['hits']),v['first'][:1]) for v in report])

