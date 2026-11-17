import unittest,pathlib,sys,json
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import compare_entry as c
class ComparisonTest(unittest.TestCase):
 def test_fresh_endpoint_replay(self):
  a=c.compare();b=json.loads((R/'sim/pressure/entry-comparison.json').read_text());self.assertEqual(a,b)
  x,y=a['rows'];self.assertEqual([v['activeSamples'] for v in x['endRates']['contacts']],[1184,1184]);self.assertEqual([v['activeSamples'] for v in y['endRates']['contacts']],[1184,1184])
  self.assertLess(abs(x['endRates']['contacts'][0]['normalMagnitudeN']-y['endRates']['contacts'][0]['normalMagnitudeN']),.000022)
  self.assertLess(y['maxAbsEnergyResidualJ'],x['maxAbsEnergyResidualJ'])
if __name__=='__main__':unittest.main()
