"""Averaged resistive pinch braking uses no battery power at either rotation sign."""
import sys
from pathlib import Path
import unittest

root=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(root/"sim"/k) for k in ("pressure","integrated","drivetrain","actuation","acquisition")]
from retained_route import short_brake
from native_pressure_lift import LiftDomainError


class PinchBrake(unittest.TestCase):
    def setUp(self):
        self.motor=dict(kvRadpsPerV=50.,motorResistanceOhm=.1,currentLimitA=40.)

    def test_pwm_weights_heat_and_delivered_torque(self):
        for omega in (-20.,20.):
            full=short_brake(omega,1.,self.motor)
            part=short_brake(omega,.25,self.motor)
            self.assertAlmostEqual(part["torqueNm"],full["torqueNm"]*.25)
            self.assertAlmostEqual(part["windingHeatW"],full["windingHeatW"]*.25)
            self.assertLess(part["torqueNm"]*omega,0.)
            self.assertAlmostEqual(part["windingHeatW"],-part["shaftPowerW"])
            self.assertEqual(part["terminalPowerW"],0.)

    def test_stopped_rotor_cannot_hold_with_a_passive_brake(self):
        self.assertEqual(short_brake(0.,1.,self.motor)["torqueNm"],0.)

    def test_reject_peak_current_even_with_small_average_fraction(self):
        with self.assertRaises(LiftDomainError):
            short_brake(1000.,.01,self.motor)


if __name__=="__main__":
    unittest.main()
