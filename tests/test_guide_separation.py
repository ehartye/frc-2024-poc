import unittest,pathlib,sys,math
R=pathlib.Path(__file__).resolve().parents[1];sys.path[:0]=[str(R/'sim/integrated'),str(R/'cad/assembly')]
import guide_separation as s,finite_retention_guide as g
class Separation(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.c=g.load_contract(R)
 def test_index_and_small_negative_pitch(self):
  for pitch in [0.,-1e-6,-.001]:
   q=s.query([.1885674400200612,0,.0254],[-math.sin(pitch),0,math.cos(pitch)],self.c)
   self.assertTrue(q['separated']);self.assertGreater(q['axisGapM'][2],.0075);self.assertIn('UNKNOWN',q['nativeMaterialStatus'])
 def test_known_disagreement_not_cleared(self):
  for k in self.c['knownDisagreements']:
   a=math.radians(k['pitchDeg']);q=s.query([v/1000 for v in k['centerRobotMm']],[-math.sin(a),0,math.cos(a)],self.c)
   self.assertFalse(q['separated']);self.assertFalse(q['geometryDomainValid'])
 def test_dense_surface_containment(self):
  # Ring basis for pitched torus, sample tube circles as an independent support check.
  for a in [-2.,-.3,0.,.6,2.]:
   axis=[-math.sin(a),0,math.cos(a)];q=s.query([0,0,0],axis,self.c);lo,hi=q['torusBoundsM']
   for i in range(80):
    u=2*math.pi*i/80;rad=[math.cos(a)*math.cos(u),math.sin(u),math.sin(a)*math.cos(u)]
    for j in range(24):
     v=2*math.pi*j/24;point=[(.1524+.0254*math.cos(v))*rad[k]+.0254*math.sin(v)*axis[k] for k in range(3)]
     self.assertTrue(all(lo[k]-1e-12<=point[k]<=hi[k]+1e-12 for k in range(3)))
 def test_bad_inputs_and_touch(self):
  for center,axis in [([True,0,0],[0,0,1]),([0,0,0],[0,0,2]),([float('nan'),0,0],[0,0,1])]:
   with self.assertRaises(ValueError):s.query(center,axis,self.c)
  # Bound touching is not strict separation.
  c=self.c;face=c['guide']['guideFaceXmm']/1000
  q=s.query([face-.1778,0,.1],[0,0,1],c);self.assertFalse(q['separated'])
if __name__=='__main__':unittest.main()
