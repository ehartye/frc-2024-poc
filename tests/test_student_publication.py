"""Check the actual Pages source and links against the withdrawn exhibit set."""
import json
import re
import subprocess
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"

class Publication(unittest.TestCase):
    def test_student_routes_and_excluded_assets(self):
        config=(DOCS/"_config.yml").read_text()
        excluded=set(re.findall(r"^  - (.+)$",config,re.M))
        self.assertTrue({"media","robot","robot-model.js","vendor","passive-cassette.json"} <= excluded)
        self.assertFalse((DOCS/".nojekyll").exists())
        self.assertFalse((DOCS/"robot/cad/index.html").exists())
        home=(DOCS/"index.html").read_text(encoding="utf-8")
        for route in ("field-lab.html","ground-relay.html","ground-contact.html","passive-entry.html"):
            self.assertIn(route,home)
            self.assertTrue((DOCS/route).is_file())
        public=[p for p in DOCS.iterdir() if p.is_file() and p.name not in excluded and p.suffix in (".html",".js")]
        for p in public:
            src=p.read_text(encoding="utf-8")
            for marker in ("<model-viewer","media/r9-assembly","media/swerve-r7","/robot/cad/","passive-cassette.json"):
                self.assertNotIn(marker,src,p.name)
        self.assertEqual(json.loads((DOCS/"passive-carrier.json").read_bytes())["groups"][0]["name"],"carrier")

    def test_readmes_do_not_offer_withdrawn_models(self):
        files=subprocess.check_output(["git","ls-files","-z"],cwd=ROOT).decode().split("\0")
        for name in files:
            if Path(name).name.lower() != "readme.md":
                continue
            text=(ROOT/name).read_text(encoding="utf-8-sig")
            for marker in ("<!-- cad-still -->","?view=r","#model)","/robot/cad/","r9-physical-assembly.f3d)","r7-maxswerve.f3d)"):
                self.assertNotIn(marker,text,name)

if __name__=="__main__":
    unittest.main()
