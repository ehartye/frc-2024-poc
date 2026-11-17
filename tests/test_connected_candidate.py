import unittest,pathlib,json,hashlib,sys
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'cad'),str(ROOT/'cad/assembly')]
import build_connected_candidate as q
class ConnectedCandidateTest(unittest.TestCase):
 def test_archive_binds_sources_and_all_states(self):
  p=ROOT/'cad/assembly/evidence';report=json.loads((p/'connected-report.json').read_text());bundle=json.loads((p/'connected-verification-bundle.json').read_text())
  self.assertEqual(report['inputs'],q.input_pins());self.assertTrue(bundle['valid'])
  self.assertEqual({x['state'] for x in report['states']},{'floor','rotated','handoff'})
  self.assertTrue(all(x['parts']==711 and x['reopenedRoots']==711 and not x['belowGround'] for x in report['states']))
  self.assertEqual(bundle['buildReceiptSha256'],hashlib.sha256((p/'connected-report.json').read_bytes()).hexdigest())
  self.assertEqual(bundle['contactReceiptSha256'],hashlib.sha256((p/'connected-note-contact.json').read_bytes()).hexdigest())
 def test_original_visual_and_step_are_exact(self):
  m=json.loads((ROOT/'cad/assembly/connected-visual.json').read_text())
  self.assertEqual(hashlib.sha256((ROOT/m['image']).read_bytes()).hexdigest(),m['imageSha256'])
  self.assertEqual(hashlib.sha256((ROOT/m['originalStep']).read_bytes()).hexdigest(),m['originalStepSha256'])
 def test_feed_and_attachment_scope(self):
  p=ROOT/'cad/assembly/evidence';feed=json.loads((p/'connected-intake-feed.json').read_text())
  self.assertEqual(feed['inputs'],q.input_pins());self.assertEqual(feed['nativePulleySource']['solidCount'],3)
  self.assertTrue(feed['nativePulleySource']['perSolidAbsoluteVolumes']);self.assertEqual(len(feed['samples']),49)
  self.assertFalse(any(r['nativePulleyHits'] or any(not h['intendedContactHypothesis'] for h in r['hits']) for r in feed['samples']))
if __name__=='__main__':unittest.main()
