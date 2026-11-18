import pathlib,sys,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'sim/component-launcher'))
import extract_geometry as e
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS_Compound
from OCP.gp import gp_Pnt
class MaterialTest(unittest.TestCase):
 def test_compound_aggregates_each_material_solid(self):
  builder=BRep_Builder();compound=TopoDS_Compound();builder.MakeCompound(compound)
  for x in [0,3]:builder.Add(compound,BRepPrimAPI_MakeBox(gp_Pnt(x,0,0),1,1,1).Shape())
  probe=BRepPrimAPI_MakeBox(5,1,1).Shape()
  self.assertAlmostEqual(e.common_volume(compound,probe),2.,places=9)
 def test_empty_compound_rejected(self):
  builder=BRep_Builder();compound=TopoDS_Compound();builder.MakeCompound(compound)
  with self.assertRaisesRegex(ValueError,'Solid material'):e.common_volume(compound,BRepPrimAPI_MakeBox(1,1,1).Shape())
 def test_separated_solids_clear(self):
  self.assertEqual(e.common_volume(BRepPrimAPI_MakeBox(1,1,1).Shape(),BRepPrimAPI_MakeBox(gp_Pnt(2,0,0),1,1,1).Shape()),0.)
if __name__=='__main__':unittest.main()
