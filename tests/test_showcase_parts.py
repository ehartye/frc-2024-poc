import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts/showcase"))
from public_parts import BUMPER, public_occurrence


class PublicSwerveBodyTests(unittest.TestCase):
    def test_generic_body_needs_its_authored_assembly_and_part(self):
        article = dict(depth=2, parent=BUMPER, component="Left bumper frame rail",name="Body1")
        self.assertTrue(public_occurrence(article))
        for change in (dict(depth=3), dict(parent="Supplier assembly"), dict(component="Supplier plate"),
                       dict(component="Front bumper frame rail")):
            with self.subTest(change=change):
                self.assertFalse(public_occurrence(article | change))

    def test_supplier_does_not_inherit_display_family(self):
        name="MAXSwerve Front left wheel"
        self.assertFalse(public_occurrence(dict(depth=1,parent=name,component=name,
                                               name="original wheel",sourceRootIndex=1)))
