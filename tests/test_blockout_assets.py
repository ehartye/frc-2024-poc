import pathlib,json,hashlib,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1]/'kickoff/blockout'
class BlockoutAssets(unittest.TestCase):
 def test_static_source_hashes(self):
  manifest=json.loads((ROOT/'SOURCE.json').read_text())
  for path,expected in manifest['filesSha256'].items():self.assertEqual(hashlib.sha256((ROOT/path).read_bytes()).hexdigest(),expected,path)
 def test_vendor_provenance(self):
  for path,item in json.loads((ROOT/'THIRD_PARTY.json').read_text()).items():self.assertEqual(hashlib.sha256((ROOT/'vendor'/path).read_bytes()).hexdigest(),item['sha256'],path)
 def test_portable_inventory(self):
  actual={p.relative_to(ROOT).as_posix() for p in ROOT.rglob('*') if p.is_file()}
  self.assertEqual(actual,{'app.js','rehearsal.js','blockout.json','index.html','SOURCE.json','README.md','THIRD_PARTY.json','vendor/LICENSE.txt','vendor/OrbitControls.js','vendor/three.module.js','ARTIFACT.md','verify-browser.cjs','verify-geometry.test.mjs','dist/design-state.js','dist/app.js','dist/design-default.json','dist/artifact-panel.js','dist/vendor/three.module.js','dist/index.html','dist/rehearsal.js','dist/design-artifact.mjs','dist/blockout.json','dist/vendor/LICENSE.txt','dist/vendor/OrbitControls.js'})
  self.assertIn('./vendor/three.module.js',(ROOT/'index.html').read_text());self.assertIn('not chosen architecture',(ROOT/'blockout.json').read_text())
if __name__=='__main__':unittest.main()
