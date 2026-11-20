"""Audit committed public text/metadata without printing matched private values."""
import argparse,hashlib,pathlib,re,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1]
PATTERNS={
 'personal-absolute-path':r'(?<![A-Za-z])[A-Z]:[\\/](?:Users|Documents and Settings)[\\/]|/(?:Users|home)/[^\s/]+',
 'private-network-address':r'\b(?:192\.168\.\d{1,3}\.\d{1,3}|10\.\d{1,3}\.\d{1,3}\.\d{1,3}|172\.(?:1[6-9]|2\d|3[01])\.\d{1,3}\.\d{1,3})\b',
 'private-network-url':r'https?://[^/\s]+\.(?:local|lan|home)(?:[:/\s]|$)|\\\\[A-Za-z0-9_-]+\\(?:Users|home|share)',
 'unexplained-machine-alias':r'\b(?:Run on|execution on|tested on|verified on)\s+[A-Z][A-Z0-9_-]{1,}(?=[\s.,;]|$)',
 'internal-coordination':r'/(?:root|user_notes)/|private[-_]workspace|home[ ]network',
 'credential':r'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----|\bgh[pousr]_[A-Za-z0-9]{20,}|\bgithub_pat_[A-Za-z0-9_]{20,}'
}
# Only the upstream Apache-licensed Gradle wrapper binary is permitted.
WRAPPER='experiments/a2-campaign/shared/examples/java-2026-sim/gradle/wrapper/gradle-wrapper.jar'
WRAPPER_SHA='2db75c40782f5e8ba1fc278a5574bab070adccb2d21ca5a6e5ed840888448046'
# Exact reviewed first-party render/visual meshes only; no general binary exemption.
ORIGINAL_ASSETS={'cad/assembly/connected-floor.png':'d33b182fd6a02b6ed33509fef781b856fea87b01c76c4b9d9a17db64bd8e415f','sim/mujoco/visual/fixed.stl':'60d02bc03df2ade596ee8f12603c07c5a16f0e01e08ef692245b260e4f6a5b25','sim/mujoco/visual/cradle.stl':'8492f434d6ee09e233d939acce1cb26015d2b40778a7236e3104e3e9ef9bcb66','sim/mujoco/visual/carriage.stl':'930c5ca0880fca59bf95ae43c7abd547bdcf7806f182af0bbaa8f1e176d2a3c7'}

SHOWCASE_ASSETS = {'docs/robot/cad/assembly.png': 'e2bd5548feedb28c19c5c0daaec9f9509cea62bf8e0119ed80b184189cae4898', 'docs/robot/cad/assembly.stl.gz': 'e9f6928eae7f4464a55316e2d30ba139506ac96f94763974ee750c70ea6c7b70'}

SHOWCASE_ASSETS.update({'docs/media/robot-native.png': 'cb63a717dfb9e159222cb68a8b9d12d0b11b2766c8bd70c86d8e2579cd13f898', 'docs/media/robot-native.glb': 'dd08c0a26cfdde4c1965f05bd063fd8cc39ec82cf29b52972916bbf2e4a05dea'})

def findings(data):
 text=data.decode('utf-8',errors='replace')
 return [name for name,p in PATTERNS.items() if re.search(p,text)]
SHOWCASE_ASSETS["docs/media/r7-nominal-d1.glb"] = "5bfe2bbbfe008a4915b23f4d185790aacef62e921498ccd2f6bcd588f4fe8e02"

SHOWCASE_ASSETS["cad/student/r7-maxswerve.f3d"] = "6e2b65f3493dfd0a8c7a27bbf26445dec29213e93806ff9b0c55df5548b00266"

SHOWCASE_ASSETS["cad/student/r8-stage-climber.f3d"] = "26a5335e66c05dec8918b767dd27a4cb62421c1d88450102e6c66d1a4914c612"
SHOWCASE_ASSETS["docs/media/r8-stage.glb"] = "77a0076da3873d89b3741887262ee825d030712e846702b1432859ce56abdeb3"
SHOWCASE_ASSETS["docs/media/r8-stage.png"] = "0e18ea715937810883b6e285848efaa76937e8ebd51440a0797baf6e5e0bc865"

SHOWCASE_ASSETS.update({
 "cad/student/r9-physical-assembly.f3d": "4663d9fa55b5f3eb8e23549f3205a2d29248322543beb3e7a7d8076a23f93f6d",
 "docs/media/r9-assembly.glb": "e0fa4d0cd90ffb64799bb63061b3b3d75fbd20b7637038a822f677b1322a74e4",
 "docs/media/r9-assembly.png": "0303f001895c0c8a0a7940896a5671174ebe18b3a28407dc82d4132692755d7f",
 "cad/assembly-readiness/drawings/r9-shop-drawings.pdf": "552981ea64eabc89ed47e6e5faa368b65344161f4e5ab539061a5dceef17b565"
})

SHOWCASE_ASSETS.update({
 "cad/student/r10-passive-service.f3d": "06d0f92aa7c3ccd4f23f489b8d8041805e07cf6ac70a3272aeeba3397e890a21",
 "cad/service-readiness/parked-fixture.png": "26ecd50c61056af72c67a63857749afe474f76ced790f66b6ccecc3769e78e99",
 "cad/service-readiness/mast-cartridge.png": "65b3c1062ae397ba87e83273479c87afdf0dca60244b9da93335e95d28330e8e"
})

def git(*args):return subprocess.check_output(['git','-C',str(ROOT),*args])
def audit(revision='HEAD'):
 failures=[]
 for raw in git('ls-tree','-rz','--name-only',revision).split(b'\0'):
  if not raw:continue
  path=raw.decode();data=git('show',revision+':'+path)
  if path in ORIGINAL_ASSETS:
   if hashlib.sha256(data).hexdigest()!=ORIGINAL_ASSETS[path]:failures.append((path,['unreviewed-original-asset']))
   continue
  if path in SHOWCASE_ASSETS:
   if hashlib.sha256(data).hexdigest()!=SHOWCASE_ASSETS[path]:failures.append((path,['unreviewed-cad-export']))
   continue
  if path==WRAPPER:
   if hashlib.sha256(data).hexdigest()!=WRAPPER_SHA:failures.append((path,['unreviewed-binary']))
   continue
  if b'\0' in data:failures.append((path,['unexpected-binary']));continue
  categories=findings(data)
  if categories:failures.append((path,categories))
 # Commit identity/message metadata, not source history rewriting.
 metadata=git('log','--format=%an <%ae> %cn <%ce> %s',revision)
 if findings(metadata):failures.append(('commit metadata',findings(metadata)))
 return failures
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--revision',default='HEAD');v=a.parse_args();errors=audit(v.revision)
 for path,categories in errors:print(path+': '+', '.join(categories))
 if errors:sys.exit(1)
 print('Public-content guard passed: committed text, binary allowlist and commit metadata.')
