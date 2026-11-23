from pathlib import Path
import json,base64,numpy as np,fast_simplification as fs
p=Path('shot.py')
s=p.read_text().replace('center[2]>1.5 and ','').replace('(4.5 if drive','(7 if drive').replace('with belts pre-spun.','starting from rest.').replace('row=dict(t=d.time,q=q.tolist())','row=dict(t=d.time,q=q.tolist(),beltMps=belts.tolist(),busV=bus)').replace('/.14;','/inertia;').replace('/.14\n','/inertia\n').replace('    belts=np.zeros','    inertia=float(sys.argv[8]) if len(sys.argv)>8 else .14\n    belts=np.zeros').replace('mu=mu,k=k,angle=angle','mu=mu,k=k,beltEquivalentKg=inertia,angle=angle')
p.write_text(s)
data=json.load(open('fixtures.json'));out={}
for k,row in data.items():
    verts=np.frombuffer(base64.b64decode(row['v']),np.float32).reshape(-1,3).astype(float)
    faces=np.frombuffer(base64.b64decode(row['f']),np.uint32).reshape(-1,3).astype(np.int32)
    v,f=fs.simplify(verts,faces,target_count=min(len(faces)-1,{'speakerPanel':22000,'stageSteel':25000,'workingChain':6000,'otherChain':8000}.get(k,3000)))
    out[k]={'v':base64.b64encode(np.asarray(v*1000,np.float32).tobytes()).decode(),'f':base64.b64encode(np.asarray(f,np.uint32).tobytes()).decode(),'bounds':[v.min(axis=0).tolist(),v.max(axis=0).tolist()]}
    print(k,len(f),out[k]['bounds'],flush=True)
Path('display-field.json').write_text(json.dumps(out,separators=(',',':')))

