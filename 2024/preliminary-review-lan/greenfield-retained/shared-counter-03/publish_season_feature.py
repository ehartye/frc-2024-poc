import os,subprocess,json,tempfile
from pathlib import Path
script=Path(__file__).resolve()
record=script.with_suffix('.json');head='598c87a9176533966739ba7f28d20aceff37f37b'
remote='https://github.com/ehartye/frc-2024-poc.git'
branch='feature/preliminary-design-review'
folder=Path(tempfile.mkdtemp(prefix='frc-season-delivery-'))
env=dict(os.environ,GIT_TERMINAL_PROMPT='0',GCM_INTERACTIVE='Never',GH_PROMPT_DISABLED='1')
git=r'C:\Program Files\Git\cmd\git.exe'
gh=r'C:\Program Files\GitHub CLI\gh.exe'
env['PATH']=str(Path(gh).parent)+';'+env['PATH']
def run(args):
    r=subprocess.run(args,cwd=folder,env=env,capture_output=True,text=True,timeout=160,creationflags=subprocess.CREATE_NO_WINDOW)
    if r.returncode:raise RuntimeError((args[1:3],r.returncode,r.stderr[-650:]))
    return r.stdout.strip()
def cmd(*a):return run([git,'-c','credential.helper=','-c','credential.helper=!gh auth git-credential',*a])
try:
    assert run([gh,'api','user','--jq','.login']).lower()=='ehartye'
    cmd('init','--bare','.')
    cmd('fetch','--depth=1','--filter=tree:0',remote,'refs/heads/main:refs/heads/upstream')
    cmd('fetch',str(script.with_name('season-pages.bundle')),'refs/heads/'+branch+':refs/heads/'+branch)
    assert cmd('rev-parse','refs/heads/'+branch)==head
    cmd('push',remote,'refs/heads/'+branch+':refs/heads/'+branch)
    got=cmd('ls-remote',remote,'refs/heads/'+branch).split()[0];assert got==head
    record.write_text(json.dumps(dict(remote=remote,branch=branch,head=got)))
except BaseException as e:record.write_text(json.dumps(dict(error=repr(e))))

