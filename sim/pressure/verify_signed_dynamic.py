"""Prescribed-state implementation parity; never an acquisition checkpoint."""
import argparse,copy,hashlib,json,pathlib,sys
p=argparse.ArgumentParser()
for n in ['project','binding','output']:p.add_argument('--'+n,type=pathlib.Path,required=True)
a=p.parse_args();assert not a.output.exists();sys.path[:0]=[str(a.project/'sim/pressure'),str(a.project/'sim/integrated')]
import native_signed_pressure as native,signed_pressure_acquisition as signed,pressure_tiled_geometry as scalar,finite_area
load=lambda p:json.loads(p.read_bytes());binding=load(a.binding);cad=load(a.project/'cad/assembly/intake_components.json');nc=load(a.project/'sim/acquisition/free-planar-config.json')['model'];dc=load(a.project/'sim/drivetrain/config.json')
finite_area.belt.geometry=lambda c:dict(binding['effectiveGeometryMm'])
class Scalar(native.NativeSignedPressure):
 acquisition_rates=signed.SignedPressureAcquisition.acquisition_rates
 def prepared(self,d,n,cache):
  key=tuple(n['p'])+(n['yaw'],d['x'],d['y'],d['heading'])
  if key not in cache:
   cache[key]=scalar.prepare(self.cad,note_center=[*n['p'],.0254],note_basis=signed.rotation(n['yaw'],0),cradle_origin=[d['x']+.178,d['y'],.0254],cradle_basis=signed.rotation(d['heading'],0),resolution=self.resolution,k_area=self.k_area)
  return cache[key]
rows=[]
for duty in [.0034,-.0034,0.]:
 one=Scalar(dc,nc,cad,binding);two=native.NativeSignedPressure(dc,nc,cad,binding)
 for m in [one,two]:m.note['p']=[.185,0.];m.note['v']=[.01,0.];m.note['w']=[0.,0.];m.initial_energy=m.note_kinetic(m.note)+m.prepared(m.drive.state,m.note,{})['potentialJ']
 max_state=max_energy=0.
 for step in range(20):
  results=[m.step(.00025,[0.,0.],[duty,duty],[dc['massKg']*9.80665/2]*2,modes=['brake','brake']) for m in [one,two]]
  def flatten(v):
   if isinstance(v,dict):return sum([flatten(v[k]) for k in sorted(v)],[])
   if isinstance(v,list):return sum([flatten(x) for x in v],[])
   return [v]
  for k in ['note','chassis']:
   max_state=max(max_state,max(abs(x-y) for x,y in zip(flatten(results[0][k]),flatten(results[1][k]))))
  max_energy=max(max_energy,max(abs(results[0]['energyJ'][k]-results[1]['energyJ'][k]) for k in results[0]['energyJ']))
 assert max_state<1e-9 and max_energy<1e-10
 rows.append(dict(duty=duty,steps=20,stepS=.00025,maximumStateDifference=max_state,maximumEnergyDifferenceJ=max_energy))
 print(json.dumps(rows[-1]),flush=True)
r=dict(schema='signed-native-dynamic-parity/1',passed=True,acceptance=False,rows=rows,limits=['Prescribed contact state for implementation comparison only; not a capture, phase join, or calibration'])
a.output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n')
