"""Replay historical source-bound tests in a disposable exact-blob sandbox."""
import pathlib,subprocess,hashlib,json,tempfile,tarfile,sys,re,platform
R=pathlib.Path(__file__).resolve().parents[2]
CASES={
 'entry':('sim/pressure/acquisition-entry-evidence.json',['test_pressure_acquisition.py','test_pressure_entry_receipt.py']),
 'half-step':('sim/pressure/entry-comparison.json',['test_pressure_entry_comparison.py']),
 'continuation':('sim/pressure/continuation-100ms-evidence.json',['test_pressure_checkpoint.py','test_pressure_continuation_receipt.py']),
 'late-policy':('sim/pressure/policy-evidence/nominal.json',['test_pressure_policy_transition.py','test_pressure_policy_boundary.py','test_pressure_policy_receipts.py'])}
sha=lambda b:hashlib.sha256(b).hexdigest()
def git(repo,*args):return subprocess.run(['git','-C',str(repo),*args],check=True,capture_output=True).stdout
def safe_path(name):
 p=pathlib.PurePosixPath(name)
 if p.is_absolute() or '..' in p.parts or not p.parts or '\\' in name or ':' in name:raise ValueError('repository-relative path required')
 return p

def recover(repo,name,wanted):
 safe_path(name)
 if not re.fullmatch('[0-9a-f]{64}',wanted):raise ValueError('SHA256 required')
 commits=['HEAD']+git(repo,'log','--all','--format=%H','--',name).decode().splitlines()
 for commit in dict.fromkeys(commits):
  result=subprocess.run(['git','-C',str(repo),'show',commit+':'+name],capture_output=True)
  if result.returncode==0 and sha(result.stdout)==wanted:return result.stdout
 raise ValueError('matching historical blob unavailable: '+name)

def run(case,output):
 if case not in CASES or output.exists():raise ValueError('known case and fresh output required')
 receipt,patterns=CASES[case];source=(R/receipt).read_bytes();report=json.loads(source);pins=report['inputs'];head=git(R,'rev-parse','HEAD').decode().strip()
 with tempfile.TemporaryDirectory(prefix='pressure-history-') as folder:
  root=pathlib.Path(folder);archive=root/'tree.tar';git(R,'archive','--format=tar','--output='+str(archive),'HEAD');tree=root/'tree';tree.mkdir()
  with tarfile.open(archive) as tar:tar.extractall(tree,filter='data')
  restored=[]
  for name,wanted in pins.items():
   safe_path(name);target=tree/name
   if not target.exists() or sha(target.read_bytes())!=wanted:
    blob=recover(R,name,wanted);target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(blob);restored.append(name)
  if any(sha((tree/name).read_bytes())!=wanted for name,wanted in pins.items()):raise ValueError('historical input mismatch')
  tests=[]
  for pattern in patterns:
   testpath=tree/'tests'/pattern
   if not testpath.exists():raise ValueError('test unavailable: '+pattern)
   result=subprocess.run([sys.executable,'-m','unittest','discover','-s','tests','-p',pattern,'-v'],cwd=tree,capture_output=True,text=True)
   if result.returncode:raise ValueError('historical test failed: '+pattern+'\n'+result.stderr)
   count=re.search(r'Ran (\d+) tests?',result.stderr)
   if not count:raise ValueError('test-count evidence missing')
   tests.append(dict(pattern=pattern,testCount=int(count.group(1)),testSha256=sha(testpath.read_bytes()),exitCode=0))
  if any(sha((tree/name).read_bytes())!=wanted for name,wanted in pins.items()):raise ValueError('historical input changed during tests')
 if source!=(R/receipt).read_bytes():raise ValueError('receipt changed')
 result=dict(schema='pressure-historical-test-replay/1',case=case,receipt=receipt,receiptSha256=sha(source),sourceInputs=pins,restoredHistoricalFiles=sorted(restored),tests=tests,python=platform.python_version(),verifierSha256=sha(pathlib.Path(__file__).read_bytes()),trackedSnapshot=head,acceptance=False,limits=['Exact receipt-pinned dependency blobs recovered by SHA, never repinned.','Selected tests only, not a full trajectory replay or physical acceptance.','Requires repository history containing each exact blob; missing history fails closed.'])
 output.parent.mkdir(parents=True,exist_ok=True);tmp=output.with_suffix('.tmp');tmp.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');tmp.replace(output);print(case,'passed',sum(t['testCount'] for t in tests),'tests; restored',len(restored),'historical files')
if __name__=='__main__':run(sys.argv[1],pathlib.Path(sys.argv[2]))
