import unittest,pathlib,sys,json,math,copy
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import finite_area as f

def rot(axis,angle):
 n=math.sqrt(f.dot(axis,axis));x,y,z=[v/n for v in axis];c=math.cos(angle);s=math.sin(angle);v=1-c
 return [[c+x*x*v,x*y*v-z*s,x*z*v+y*s],[y*x*v+z*s,c+y*y*v,y*z*v-x*s],[z*x*v-y*s,z*y*v+x*s,c+z*z*v]]
def mm(a,b):return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
class PressureTest(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.c=json.loads((R/'cad/assembly/intake_components.json').read_text())
 def test_surface_area(self):
  self.assertAlmostEqual(sum(w for p,w in f.material_points(.1524,.0254,32,32)),4*math.pi**2*.1524*.0254,places=12)
 def test_no_contact_and_no_renormalization(self):
  a=f.evaluate(self.c,note_center=[.01,0,0],resolution=(64,64));b=f.evaluate(self.c,note_center=[.01,0,0],resolution=(64,64),k_area=2e7)
  self.assertAlmostEqual(b['potentialJ'],2*a['potentialJ']);self.assertAlmostEqual(b['sides'][1]['normalMagnitudeN'],2*a['sides'][1]['normalMagnitudeN']);self.assertEqual(a['sides'][1]['activeAreaM2'],b['sides'][1]['activeAreaM2'])
  q=f.evaluate(self.c,note_center=[-1,0,0],resolution=(32,32));self.assertEqual(q['potentialJ'],0);self.assertEqual(q['activeSamples'],0)
 def test_objectivity_and_wrench_reciprocity(self):
  p=[.0105,0,.0003];b=rot([0,1,0],.004);a=f.evaluate(self.c,note_center=p,note_basis=b,resolution=(96,96));q=rot([1,2,3],.4);t=[.5,-.2,.7];z=f.evaluate(self.c,note_center=f.add(t,f.mv(q,p)),note_basis=mm(q,b),cradle_origin=t,cradle_basis=q,resolution=(96,96))
  self.assertAlmostEqual(a['potentialJ'],z['potentialJ'],places=12)
  for key in ['noteForceWorldN','noteMomentWorldNm','cradleReactionWorldN','cradleMomentWorldNm']:
   for x,y in zip(f.mv(q,a[key]),z[key]):self.assertAlmostEqual(x,y,places=9)
  self.assertLess(max(map(abs,f.add(a['noteForceWorldN'],a['cradleReactionWorldN']))),1e-12)
  total=f.add(f.add(a['noteMomentWorldNm'],a['cradleMomentWorldNm']),f.cross(p,a['noteForceWorldN']));self.assertLess(max(map(abs,total)),1e-12)
 def test_potential_work_body_cradle_and_u(self):
  p=[.0105,0,.0003];nb=rot([0,1,0],.004);v=[.003,-.002,.001];w=[.01,.02,-.03];cv=[-.001,.001,.002];cw=[-.02,.005,.01];ud=.0007;h=1e-6
  a=f.evaluate(self.c,note_center=p,note_basis=nb,u_positive_m=.00002,note_velocity=v,note_omega=w,cradle_velocity=cv,cradle_omega=cw,u_velocity=ud,resolution=(96,96))
  def e(dt):
   return f.evaluate(self.c,note_center=f.add(p,f.scale(v,dt)),note_basis=mm(rot(w,math.sqrt(f.dot(w,w))*dt),nb),cradle_origin=f.scale(cv,dt),cradle_basis=rot(cw,math.sqrt(f.dot(cw,cw))*dt),u_positive_m=.00002+dt*ud,resolution=(96,96))['potentialJ']
  self.assertAlmostEqual((e(h)-e(-h))/(2*h),a['predictedPotentialRateW'],delta=2e-8)
 def test_source_sdf_gradient(self):
  g=f.belt.geometry(self.c);y=(g['axisYAbs']-g['outerRadius'])/1000
  for p in [[.03,y+.0002,0],[.03,y+.0004,g['halfHeight']/1000-.0001]]:
   q=f.shallow_gradient(p,1,g);self.assertGreater(q['penetrationM'],0)
   for j in range(3):
    h=1e-8;a=p.copy();b=p.copy();a[j]+=h;b[j]-=h
    fd=(f.belt.query(f.scale(a,1000),1,g)['signedDistanceToMaterialMm']-f.belt.query(f.scale(b,1000),1,g)['signedDistanceToMaterialMm'])/1000/(2*h)
    self.assertAlmostEqual(fd,q['normal'][j],places=7)
 def test_unsupported_wrap_and_deep(self):
  g=f.belt.geometry(self.c);x=(g['rearX']-g['outerRadius']*.5)/1000;y=(g['axisYAbs']-g['outerRadius']*math.sqrt(.75)+.0001)/1000
  with self.assertRaises(f.DomainError):f.shallow_gradient([x,y,0],1,g)
  y=(g['axisYAbs']-g['innerRadius']-.1)/1000
  with self.assertRaises(f.DomainError):f.shallow_gradient([.03,y,0],1,g)
 def test_coarse_missed_contact_control(self):
  a=f.evaluate(self.c,note_center=[.010567440020061214,0,0],resolution=(32,32));b=f.evaluate(self.c,note_center=[.010567440020061214,0,0],resolution=(128,128));self.assertEqual(a['activeSamples'],0);self.assertGreater(b['activeSamples'],0)
 def test_bad_inputs(self):
  for k in [True,float('nan'),float('inf'),0,-1]:
   with self.assertRaises(ValueError):f.evaluate(self.c,note_center=[0,0,0],k_area=k)
if __name__=='__main__':unittest.main()
