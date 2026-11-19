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
WRAPPER='shared/examples/java-2026-sim/gradle/wrapper/gradle-wrapper.jar'
WRAPPER_SHA='2db75c40782f5e8ba1fc278a5574bab070adccb2d21ca5a6e5ed840888448046'
# Exact reviewed first-party render/visual meshes only; no general binary exemption.
ORIGINAL_ASSETS={'cad/assembly/connected-floor.png':'d33b182fd6a02b6ed33509fef781b856fea87b01c76c4b9d9a17db64bd8e415f','sim/mujoco/visual/fixed.stl':'60d02bc03df2ade596ee8f12603c07c5a16f0e01e08ef692245b260e4f6a5b25','sim/mujoco/visual/cradle.stl':'8492f434d6ee09e233d939acce1cb26015d2b40778a7236e3104e3e9ef9bcb66','sim/mujoco/visual/carriage.stl':'930c5ca0880fca59bf95ae43c7abd547bdcf7806f182af0bbaa8f1e176d2a3c7'}

SHOWCASE_ASSETS = {'docs/robot/cad/assembly.png': 'e2bd5548feedb28c19c5c0daaec9f9509cea62bf8e0119ed80b184189cae4898', 'docs/robot/cad/assembly.stl.gz': 'e9f6928eae7f4464a55316e2d30ba139506ac96f94763974ee750c70ea6c7b70'}

def findings(data):
 text=data.decode('utf-8',errors='replace')
 return [name for name,p in PATTERNS.items() if re.search(p,text)]
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
