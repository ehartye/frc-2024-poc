"""Source-bound exact-grid benchmark; wall times are observations, not realtime promises."""
import pathlib,sys,json,hashlib,time,platform
import numpy
R=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(R/'sim/pressure'))
import fast_geometry as fast,finite_transport as reference
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
FILES=['sim/pressure/fast_geometry.py','sim/pressure/benchmark_geometry.py','tests/test_pressure_fast_geometry.py']
def encoded(v):return json.dumps(v,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
def run(output):
 if output.exists():raise ValueError('fresh output required')
 source=json.loads((R/'sim/pressure/transport-evidence.json').read_text());pins=dict(source['inputs']);pins['sim/pressure/transport-evidence.json']=sha(R/'sim/pressure/transport-evidence.json');pins.update({f:sha(R/f) for f in FILES})
 if any(sha(R/f)!=h for f,h in pins.items()):raise ValueError('stale source')
 c=json.loads((R/'cad/assembly/intake_components.json').read_text());args=dict(note_center=[.158,0.,0.],resolution=(1024,1024));fast.material_grid.cache_clear()
 start=time.perf_counter();a=reference.prepare(c,**args);scalar=time.perf_counter()-start
 start=time.perf_counter();b=fast.prepare(c,**args);cold=time.perf_counter()-start
 start=time.perf_counter();d=fast.prepare(c,**args);warm=time.perf_counter()-start
 if encoded(a)!=encoded(b) or encoded(a)!=encoded(d):raise ValueError('contact byte mismatch')
 kw=dict(pitch_radius_m=c['pinch']['teeth']*c['pinch']['pitch']/1000/(2*reference.math.pi),rotor_omega=[20.,3.],note_velocity=[.1,.02,0.],note_omega=[0.,0.,.03]);ra=reference.rates(a,**kw);rb=reference.rates(b,**kw)
 if encoded(ra)!=encoded(rb):raise ValueError('force/work byte mismatch')
 if any(sha(R/f)!=h for f,h in pins.items()):raise ValueError('source changed')
 report=dict(schema='pressure-grid-cull-benchmark/1',inputs=pins,arguments=args,rateArguments=kw,python=platform.python_version(),numpy=numpy.__version__,scalarSeconds=scalar,coldFastSeconds=cold,warmFastSeconds=warm,contactBytesSha256=hashlib.sha256(encoded(a)).hexdigest(),rateBytesSha256=hashlib.sha256(encoded(ra)).hexdigest(),contactByteEquality=True,forceWorkByteEquality=True,activeSamples=len(a['contacts']),potentialJ=a['potentialJ'],features=a['features'],rates=ra,acceptance=False,limits=['Single measured1024-square front-wrap fixture; not a complete trajectory or realtime claim.','Exact scalar transforms/contact/force/work are retained after padded vectorized broadphase.','No grid coarsening or changed material phase.','Unit tests separately cover rigid frames, caps, separated/near-boundary, deep and tie rejection.'])
 output.parent.mkdir(parents=True,exist_ok=True);temp=output.with_suffix('.tmp');temp.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');temp.replace(output);print(json.dumps({k:report[k] for k in ['scalarSeconds','coldFastSeconds','warmFastSeconds','activeSamples','contactByteEquality','forceWorkByteEquality']}))
if __name__=='__main__':run(pathlib.Path(sys.argv[1]))
