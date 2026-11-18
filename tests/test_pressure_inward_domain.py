import pathlib,sys,unittest,json
R=pathlib.Path(__file__).resolve().parents[1];sys.path[:0]=[str(R/'sim/pressure'),str(R/'sim/acquisition')]
import inward_domain_gate as new,obstacle_gate as old
class Domain(unittest.TestCase):
 def test_only_domain_and_prefilter_change(self):
  p=(R/'sim/acquisition/obstacle_gate.py').read_text();q=(R/'sim/pressure/inward_domain_gate.py').read_text()
  q=q.replace('Additive 138 mm experiment; rebuild full inventory, preserve positive-candidate UNKNOWN.','Conservative flat-torus/AABB domain gate; positive candidates are UNKNOWN.').replace(' lower=138. # millimetres: explicit additional 40 mm experimental domain\n','').replace('b[3]<lower-major-tube','b[3]<178-major-tube').replace('not lower<=center[0]<=658','not 178<=center[0]<=658')
  self.assertEqual(p,q)
 def test_previously_filtered_obstacle_is_restored(self):
  report={'schema':'conservative-floor-obstacles/1','assemblyPartCount':711,'excludedContactParts':['intake candidate side pinch belt -1','intake candidate side pinch belt 1'],'obstacles':[{'name':'newly admitted obstacle','basis':'box','boundsMm':[-15,-1,20,-14,1,30]}]}
  # R=152.4,r=25.4: old lowermost possible torus extent0.2 excludes box, extended domain includes it.
  a=old.make_gate(report,152.4,25.4,25.4);b=new.make_gate(report,152.4,25.4,25.4)
  self.assertEqual(a.candidate_count,0);self.assertEqual(b.candidate_count,1)
  self.assertEqual(b([138,0,25.4],152.4,25.4)['status'],'UNKNOWN_HARD_OR_RESERVE_CANDIDATE: newly admitted obstacle')
 def test_original_domain_and_new_lower_bound(self):
  r=json.loads((R/'sim/acquisition/free-obstacles.json').read_text());a=old.make_gate(r,152.4,25.4,25.4);b=new.make_gate(r,152.4,25.4,25.4)
  for x in [178,180,200,300,500,658]:self.assertEqual(a([x,0,25.4],152.4,25.4),b([x,0,25.4],152.4,25.4))
  self.assertEqual(b([137.99,0,25.4],152.4,25.4)['status'],'OUTSIDE_BOUNDED_PLANAR_EXPERIMENT')
 def test_full_inventory_and_boundaries(self):
  r=json.loads((R/'sim/acquisition/free-obstacles.json').read_text());g=new.make_gate(r,152.4,25.4,25.4)
  for x in range(138,659,5):
   for y in [-20,-10,0,10,20]:
    expected='SUPPORTED'
    for o in r['obstacles']:
     refined=o['basis'] in ['configured source profile band conservative box','full native bearing outer-bounds correction; existing unseated center, both flange directions']
     fn=new.torus_vertical_cylinder_distance if refined else new.torus_box_distance
     if fn([x,y,25.4],152.4,o['boundsMm'])<=25.4+1e-7:
      expected='UNKNOWN_HARD_OR_RESERVE_CANDIDATE: '+o['name'];break
    self.assertEqual(g([x,y,25.4],152.4,25.4)['status'],expected)
  for q in [[137.999,0,25.4],[170,20.001,25.4]]:self.assertEqual(g(q,152.4,25.4)['status'],'OUTSIDE_BOUNDED_PLANAR_EXPERIMENT')
  for q,major,tube in [([170,0,25.41],152.4,25.4),([170,0,25.4],152.5,25.4),([170,0,25.4],152.4,25.5)]:
   with self.assertRaises(ValueError):g(q,major,tube)
if __name__=='__main__':unittest.main()
