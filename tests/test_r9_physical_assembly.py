"""Check that the public R9 receipts identify the downloadable parked assembly."""
import hashlib
import json
import os
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]
READY=ROOT/'cad/assembly-readiness'


def read(path):
    return json.loads(path.read_bytes())


class MechanicalHandoff(unittest.TestCase):
    def test_archive_and_bill(self):
        receipt=read(ROOT/'docs/detailed-design/archive/draft-001/media/r9-assembly.json')
        digest=hashlib.sha256((ROOT/receipt['assets']['archive']['path']).read_bytes()).hexdigest()
        result=read(READY/'mass/result.json')
        actual=read(READY/'mass/actual.json')
        self.assertEqual(digest,receipt['sourceSha256'])
        self.assertEqual(digest,result['archiveSha256'])
        self.assertEqual(digest,actual['archiveSha256'])
        self.assertEqual(receipt['solids'],len(actual['bodies']))
        self.assertAlmostEqual(result['rawNativeKg'],sum(row['massKg'] for row in actual['bodies']),places=6)
        self.assertFalse(result['complete'])
        self.assertIsNone(result['inspectionMarginKg'])
        if os.environ.get('R9_CHECKED_MASS'):
            checked=read(Path(os.environ['R9_CHECKED_MASS']))['nativeMass']
            for k in ('archiveSha256','nativeBodies','rawNativeKg','supplierSubtotalKg','knownInspectionRangeKg','inspectionMarginKg'):
                self.assertEqual(result[k],checked[k],k)

    def test_fabricated_deck_and_seating(self):
        fit=read(READY/'harness-fit.json')
        self.assertEqual(len(fit['deckBores']),8)
        self.assertEqual(len(fit['posts']),4)
        self.assertTrue(any(row['upperDriverOtherBodies'] for row in fit['predecessorPostLocations']))
        for cable in fit['cables']:
            self.assertEqual(cable['lumps'],1)
            self.assertAlmostEqual(cable['centerLengthMm'],123.123816,places=4)
            self.assertAlmostEqual(cable['installedVolumeMm3'],5808.11698,places=2)
            self.assertGreater(min(cable['arcCenterTestSphereVolumesMm3']),4.17)
        self.assertTrue(all(r['filledMm3']<.01 for r in fit['deckBores']))
        self.assertTrue(all(min(r['postFootLandMm2'],r['shelfLandMm2'])>55 for r in fit['posts']))
        self.assertTrue(all(not r['hits'] for r in fit['serviceGauges']))
        breaker=read(READY/'breaker-fit.json')
        self.assertEqual(len(breaker['nominalLugSeats']),2)
        for seat in breaker['nominalLugSeats']:
            self.assertGreater(seat['padToLugMm2'],85)
            self.assertAlmostEqual(seat['distanceBeyondNutMm'],4.0132,places=3)

    def test_parked_frame_and_service(self):
        hull=read(READY/'frame-fit.json')['measurement']
        self.assertFalse(hull['otherBodyBoxFailures'])
        self.assertLess(hull['enclosingRectanglePerimeterMm']+4,3048)
        screen=read(READY/'breaker-service.json')
        self.assertTrue(all(route['bootGaugeToOtherBodyMm']>0 for route in screen['localRoutes']))
        self.assertGreaterEqual(min(g['distanceToNeighborBoxMm'] for lead in screen['allNeighborBootReceiving'] for g in lead['measurements']),1.49)
        self.assertLess(min(route['bootGaugeToOtherBodyMm'] for route in screen['priorLocationControl']['localRoutes']),0)


if __name__=='__main__':
    unittest.main()
