import unittest
from native_drive_interactive_v3 import InputLatch,RunTiming
from native_drive_interactive import command
class LatchTests(unittest.TestCase):
 def test_short_tap_latches_without_held_state(self):
  l=InputLatch();l.event(87,True,10);keys,quit,m=l.poll([],True,10.01);self.assertIn('W',keys);self.assertEqual(command(keys)[0],(.3,.3));self.assertEqual(m['acceptedEventCount'],1);self.assertNotIn('W',l.poll([],True,10.101)[0])
 def test_focus_loss_clears_not_resumes(self):
  l=InputLatch();l.event(87,True,1);self.assertEqual(l.poll(['W'],False,1.01)[0],set());self.assertEqual(l.poll([],True,1.02)[0],set())
 def test_held_extends_and_unknown_rejected(self):
  l=InputLatch();l.event(87,True,0);self.assertIn('W',l.poll(['W'],True,1)[0]);l.event(999,True,1);l.event(87,False,1);self.assertEqual(l.poll([],True,1.01)[2]['acceptedEventCount'],1)
 def test_quit_event_and_brake_priority(self):
  l=InputLatch();l.event(81,True,0);self.assertTrue(l.poll([],True,.01)[1]);self.assertEqual(command(['W','B'])[1],('brake','brake'))
 def test_arrow_callback_unmapped_but_held_available(self):
  l=InputLatch();l.event(-1,True,0);self.assertEqual(l.poll([],True,.01)[0],set());self.assertIn('UP',l.poll(['UP'],True,.01)[0]);self.assertEqual(l.poll([],True,.01)[2]['acceptedEventCount'],0)
 def test_measured_timing(self):
  c=RunTiming(sim_start=5.,now=100.);v=c.sample(6.,now=102.);self.assertEqual(v['wallElapsedS'],2.);self.assertEqual(v['simulatedElapsedS'],1.);self.assertEqual(v['realTimeFactor'],.5);self.assertIsNone(c.sample(5.,now=100.)['realTimeFactor'])
if __name__=='__main__':unittest.main()
