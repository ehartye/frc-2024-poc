import unittest,importlib.util,pathlib,tempfile,csv,json
ROOT=pathlib.Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('replay',ROOT/'scripts/build-a2-replay.py');replay=importlib.util.module_from_spec(spec);spec.loader.exec_module(replay)
class ReplayInput(unittest.TestCase):
 def row(self,t):return dict(timeS=t,state='TEST_FIXTURE',robotX=1,robotY=0,robotHeadingRad=3.14,intakeAngleDeg=-40,intakeLengthM=.68,piecePhase='UNKNOWN',pieceXWorld='',pieceYWorld='',pieceZWorld='',shooterRpm=0,shotCount=0,fault='NONE',event='')
 def run_rows(self,rows):
  with tempfile.TemporaryDirectory() as d:
   trace=pathlib.Path(d)/'trace.csv';out=pathlib.Path(d)/'view.html'
   with trace.open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
   replay.render(trace,out);return out.read_text(encoding="utf-8")
 def test_portable_unknown_state_and_binding(self):
  text=self.run_rows([self.row(0),self.row(.02)]);self.assertNotIn('__DATA__',text);self.assertIn('traceSha256',text);self.assertIn('contractSha256',text);self.assertIn('"pieceXWorld": null',text)
 def test_nonfinite_clock_rejected(self):
  with self.assertRaises(ValueError):self.run_rows([self.row('NaN')])
 def test_reverse_clock_rejected(self):
  with self.assertRaises(ValueError):self.run_rows([self.row(1),self.row(0)])
 def test_missing_required_numeric_rejected(self):
  row=self.row(0);row['shooterRpm']=''
  with self.assertRaises(ValueError):self.run_rows([row])
if __name__=='__main__':unittest.main()
