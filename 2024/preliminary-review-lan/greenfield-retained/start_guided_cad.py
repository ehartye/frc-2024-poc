import ast, json, hashlib, sys, runpy, subprocess
from pathlib import Path
here=Path(__file__).resolve().parent
project=Path(r'R:/iterations/002/design/cad/guided-cassette/profile-a')
private=Path(r'R:/2024/private-readiness-20261120')
jobs=private/'native-jobs'
install=Path.home()/'.codex/plugins/cache/agent-frc-local/agent-frc/0.86.0'
native=private/'iteration-002-native'
read=lambda p:json.loads(p.read_bytes())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def save(p,x):p.write_text(json.dumps(x,indent=2)+'\n', encoding='utf8')
def main(tag):
    top=next(ast.literal_eval(t.value) for t in ast.parse((here/'pickup-lip/geometry.py').read_text(encoding='utf8')).body if isinstance(t,ast.Assign) and any(isinstance(v,ast.Name) and v.id=='top' for v in t.targets))
    selection=read(here/'swap-blade/blade-selection.json')
    assert sha(here/'swap-blade/A-blunt.step')==selection['sha256']
    save(project/'profile.json',dict(sectionXZmm=top,stationsYmm=[-165,-55,55,165],referenceSha256=selection['sha256']))
    stages=runpy.run_path(str(install/'scripts/native_job.py'))
    plan=here/(tag+'-job.json')
    marker=install/'.agent-frc-stage.json'
    save(plan,dict(schema='agent-frc/native-job@1',jobId=tag,pluginCommit=read(marker)['sourceHead'],pluginInventorySha256=sha(marker),recipe='inputs/author_blade.py',inputs=[dict(path=f,sha256=sha(project/f)) for f in ('author_blade.py','profile.json')]))
    digest=sha(plan)
    stages['stage'](install,project,plan,digest,jobs)
    bundle=jobs/tag
    principal=subprocess.check_output(['whoami'],text=True).strip()
    subprocess.run(['icacls',str(bundle),'/grant',principal+':(OI)(CI)RX','/T','/Q'],check=True,stdout=subprocess.DEVNULL)
    stages['verify'](bundle,digest)
    for phase in ('access','build'):
        cfg=read(native/(phase+'-linked-control01.json'))
        cfg.update(jobId=tag,requestId=tag+'-'+phase,bundle=str(bundle),output=str(native/tag),registrationSha256=digest,verifierSha256=sha(bundle/'plugin/scripts/native_job.py'))
        cfg['nativeJob'].update(bundle=str(bundle),registrationSha256=digest)
        cfgpath=native/(phase+'-'+tag+'.json')
        save(cfgpath,cfg)
        subprocess.run([sys.executable,'-B',str(native/'downhill_client.py'),str(cfgpath),str(native/'request-journal')],check=True)
        if phase=='access':
            result=read(native/'request-journal'/(tag+'--'+tag+'-access.result.json'))
            assert result.get('verified', result.get('success',True)) and not result.get('error')
if __name__=='__main__':main(sys.argv[1])
