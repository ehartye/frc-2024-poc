import hashlib
import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT / "experiments/passive-entry"
DOCS = ROOT / "docs"

def read(path):
    return json.loads(path.read_bytes())

class PassiveEntry(unittest.TestCase):
    def test_reopened_parts_and_published_model(self):
        receipt = read(STUDY / "part-receipt.json")
        view = read(DOCS / "passive-cassette.json")
        sha = hashlib.sha256((STUDY / "paired-entry.step").read_bytes()).hexdigest()
        self.assertEqual((sha,sha),(receipt["stepSha256"],view["stepSha256"]))
        self.assertEqual(receipt["roots"],16)
        self.assertEqual([r["name"] for r in receipt["articles"] if r["side"]==1],
                         [g["name"] for g in view["groups"]])
        self.assertAlmostEqual(receipt["totalG"],215.58)
        for group in view["groups"]:
            self.assertTrue(0 <= min(group["f"]) <= max(group["f"]) < len(group["v"]))

    def test_original_clocks_and_actual_ownership(self):
        field = read(STUDY / "field.json")
        self.assertEqual(field,read(DOCS / "passive-entry.json")["runs"])
        first, second = "center-2/1", "center-1/1"
        expected = {"clear":("stored",126.3),"late":("B2",126.3),"misaligned":("stored",127.55),
                    "late-offset":("B2",127.4),"slow-receiver":("pad",126.75),"directDispatch":("B2",126.75)}
        for case,(owner,t) in expected.items():
            run=field[case]
            self.assertEqual(run["accepted"],[first,second])
            self.assertEqual(run["finalCustody"],{first:"stored",second:owner})
            self.assertEqual(run["receiverReady"],owner=="stored")
            self.assertTrue(run["carrierReset"])
            self.assertAlmostEqual(run["pieces"][second]["timeS"],t)
            for p in run["pieces"].values():
                self.assertTrue(p["qualified"])
                self.assertGreater(p["minCarryFloorMm"],.25)
                self.assertLess(p["maxTopMm"],57)
                self.assertLess(p["contactEndS"],p["timeS"]-.25)
            self.assertEqual(sum(e["action"]=="reset" and e["piece"]==second for e in run["receiverEvents"]),int(owner=="stored"))
        self.assertEqual(field["directDispatch"]["receiverEvents"][-1]["timeS"],148.2)

    def test_materials_and_service_controls(self):
        record=read(STUDY / "service.json")
        for side in ("-1","1"):
            s=record[side]
            self.assertFalse(s["installed"] or s["driver"] or s["pilotAgainstOtherBodies"])
            self.assertFalse(any(s["frontExtraction"].values()))
            self.assertTrue(s["verticalControl"] and s["shortRiseHostControl"])
            self.assertGreater(s["wheel360"][0],4.6)
            self.assertGreater(s["serviceWheel360"][0],2.7)
            self.assertIn("tire",s["outboardWheelControl"]["article"])
        self.assertGreater(record["carriedNote"]["backingMm"],9)
        controls=read(STUDY / "sensitivity.json")["clear"]
        for k in ("nominal","halfStep","fine","finer"):
            self.assertGreater(controls[k]["minFloorMm"],.25)
        self.assertLess(controls["wideSoft"]["minFloorMm"],0)
        self.assertLess(controls["carpetHigh"]["minFloorMm"],0)
        self.assertGreater(controls["narrowStiff"]["maxTopMm"],57)
        self.assertGreater(controls["slick"]["padErrorMm"],4000)

    def test_exact_showcase_assets_and_separate_pairs(self):
        audits=read(STUDY / "showcase-audit.json")
        self.assertEqual(audits["nearSurfaceMm"],.2)
        for key,asset,count in [("r7","swerve-r7",27),("r9","r9-assembly",24)]:
            audit=audits[key]
            self.assertEqual(audit["assetSha256"],hashlib.sha256((DOCS / "media" / (asset+".glb")).read_bytes()).hexdigest())
            self.assertEqual(len(audit["islands"]),count)
            pairs=[{name for i in g for name in audit["shells"][i]["names"]} for g in audit["islands"] if len(g)==2]
            for number in (513,528):
                self.assertTrue(any(all(any(n.startswith("R7 "+str(v)+" ") for n in names) for v in (number,number+1)) for names in pairs))
            for corner in ("Front left","Front right","Rear left","Rear right"):
                self.assertIn({corner+" wheel hub",corner+" tire"},pairs)
        self.assertIn('src="media/swerve-r7.glb"',(DOCS/"index.html").read_text(encoding="utf-8"))
        self.assertIn("?view=r9-assembly#model",(ROOT/"README.md").read_text(encoding="utf-8"))

if __name__=="__main__":
    unittest.main()
