import pathlib,sys,unittest,json
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import run_fresh_local_refinement as m
class Local(unittest.TestCase):
 def test_exact_state_energy_and_next_command(self):
  p,c,pins,bus,a=m.load();r=json.loads((R/m.SOURCE).read_text());row=r['trace'][1399]
  self.assertEqual(p.drive.state,row['chassis']);self.assertEqual(p.note,row['note']);self.assertEqual(p.energy,{k:row['energyJ'][k] for k in p.energy});self.assertEqual(p.t,row['timeS']);self.assertAlmostEqual(a['nextControllerTimeS'],1.4)
  o=m.prior.observe(p.t,p.drive.state,p.note,bus);self.assertEqual(c.update(p.t,o),r['events'][140]['command'])
 def test_first_original_step_matches(self):
  p,c,pins,bus,a=m.load();r=json.loads((R/m.SOURCE).read_text());cmd=c.update(p.t,m.prior.observe(p.t,p.drive.state,p.note,bus));row=p.step(.001,cmd['driveDemands'],cmd['pinchDemands'],[p.drive.c['massKg']*9.80665/2]*2,modes=cmd['modes'])
  old=r['trace'][1400]
  for k,v in row.items():self.assertEqual(v,old[k],k)
if __name__=='__main__':unittest.main()
