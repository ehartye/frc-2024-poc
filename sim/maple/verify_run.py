import argparse, hashlib, json, os, pathlib, subprocess, xml.etree.ElementTree as ET
ROOT=pathlib.Path(__file__).resolve().parent
COMMIT='1b1e8ea7278de01d40a5ea14dffbaaf13381512a'
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def inventory(upstream):
    head=subprocess.check_output(['git','-C',str(upstream),'rev-parse','HEAD'],text=True).strip()
    if head!=COMMIT: raise ValueError('Unexpected Maple source commit')
    if subprocess.check_output(['git','-C',str(upstream),'status','--porcelain','--untracked-files=no'],text=True).strip(): raise ValueError('Modified upstream source')
    files=list((ROOT/'src').rglob('*.java'))+[ROOT/n for n in ('build.gradle','settings.gradle','gradle.properties','toolchain-source.json','verify_run.py','run.ps1')]+[ROOT.parent/'drivetrain/config.json']
    out={'project/'+p.relative_to(ROOT.parent).as_posix():sha(p) for p in files}
    for p in sorted((upstream/'project/src/main/java').rglob('*.java')): out['upstream/'+p.relative_to(upstream).as_posix()]=sha(p)
    out['upstream/LICENSE-IronMaple.md']=sha(upstream/'LICENSE-IronMaple.md')
    return out

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--maple-source',required=True);a=ap.parse_args();up=pathlib.Path(a.maple_source).resolve()
    before=inventory(up)
    jdk=ROOT/'.tools/jdk-17.0.21+9';gradle=ROOT/'.tools/gradle-8.14.3/bin/gradle.bat'
    if not (jdk/'bin/java.exe').is_file() or not gradle.is_file():raise RuntimeError('Approved local tools missing; no automatic installation')
    env=dict(os.environ,JAVA_HOME=str(jdk),GRADLE_USER_HOME=str(ROOT/'.gradle-home'))
    dest=ROOT/'artifacts/verified-run.json';dest.parent.mkdir(exist_ok=True)
    if dest.exists(): dest.replace(ROOT/'artifacts/verified-run.previous.json')
    subprocess.run([str(gradle),'-p',str(ROOT),'-PmapleSource='+str(up),'test','--rerun-tasks','--console=plain'],cwd=ROOT,env=env,check=True)
    after=inventory(up)
    if before!=after:raise RuntimeError('Input source changed during run; no accepted receipt')
    result=ROOT/'artifacts/arena-headless.json';xml=ROOT/'build/test-results/test/TEST-frc.maplearena.ArenaHarnessTest.xml';test=ET.parse(xml).getroot()
    if test.get('tests')!='1' or test.get('failures')!='0' or test.get('errors')!='0':raise RuntimeError('Unexpected test outcome')
    receipt={'schema':'maple-arena-source-bound-run/1','upstreamCommit':COMMIT,'upstreamUrl':'https://github.com/Shenzhen-Robotics-Alliance/maple-sim','sourceSha256':before,'sourceStable':True,'testCount':1,'scenarioCount':len(json.loads(result.read_text())['cases']),'resultSha256':sha(result),'junitSha256':sha(xml),'scope':'Headless actual 2024 Maple arena with reduced original differential force adapter; no accepted scoring, collection, full robot or calibrated six-wheel physics.'}
    dest.write_text(json.dumps(receipt,indent=2)+'\n');print('Accepted source-bound receipt:',dest.name)
if __name__=='__main__':main()
