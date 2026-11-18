import argparse,hashlib,json,math,pathlib,sys,time
p=argparse.ArgumentParser();p.add_argument('--project',type=pathlib.Path,required=True);p.add_argument('--output',type=pathlib.Path,required=True);a=p.parse_args();assert not a.output.exists()
sys.path.insert(0,str(a.project/'sim/pressure'))
import fast_geometry as old,pressure_tiled_geometry as new,finite_area as area
config=json.loads((a.project/'cad/assembly/intake_components.json').read_bytes());rows=[]
for x,y,yaw,z in [(x,0.,0.,.0254) for x in [.658,.4,.324,.20119016209167434,.185,.138]]+[(.185,.0005,.002,.0254),(.185,-.0005,-.002,.0254),(.185,0.,0.,.03),(.185,.02,0.,.0254)]:
    b=[[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]]
    args=dict(note_center=[x,y,z],note_basis=b,cradle_origin=[.178,0,.0254],resolution=(1024,1024))
    results=[];timings=[]
    for module in [old,new]:
        start=time.perf_counter()
        try:r=module.prepare(config,**args);outcome='RETURNED'
        except ValueError as e:r={'type':type(e).__name__,'reason':str(e)};outcome='REJECTED'
        timings.append(time.perf_counter()-start);results.append((outcome,r))
    assert results[0]==results[1],(x,y,yaw,z)
    rows.append(dict(x=x,y=y,yaw=yaw,z=z,exactlyEqual=True,outcome=outcome,contacts=len(results[0][1].get('contacts',[])),originalSeconds=timings[0],candidateSeconds=timings[1]));print(json.dumps(rows[-1]),flush=True)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
result=dict(schema='pressure-tiled-culling-comparison/1',acceptance=False,scriptSha256=sha(pathlib.Path(__file__)),originalSourceSha256=sha(pathlib.Path(old.__file__)),candidateSourceSha256=sha(pathlib.Path(new.__file__)),configSha256=sha(a.project/'cad/assembly/intake_components.json'),rows=rows,limits=['Finite exact-output/exception controls; no hardware or capture validation','No grid, force law, scalar gradient or contact order changes','Optimization is separate from the running original-law continuation'])
a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8',newline='\n')
