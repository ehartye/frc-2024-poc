import argparse,json,os,pathlib,subprocess,xml.etree.ElementTree as ET
from verify_run import ROOT,COMMIT,inventory,sha

def inputs(up):
    result=inventory(up);result['project/maple/verify_projectile.py']=sha(pathlib.Path(__file__));return result

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--maple-source',required=True);a=ap.parse_args();up=pathlib.Path(a.maple_source).resolve();before=inputs(up)
    env=dict(os.environ,JAVA_HOME=str(ROOT/'.tools/jdk-17.0.21+9'),GRADLE_USER_HOME=str(ROOT/'.gradle-home'))
    dest=ROOT/'artifacts/projectile-verified.json'
    if dest.exists():dest.replace(ROOT/'artifacts/projectile-verified.previous.json')
    subprocess.run([str(ROOT/'.tools/gradle-8.14.3/bin/gradle.bat'),'-p',str(ROOT),'-PmapleSource='+str(up),'test','--tests','frc.maplearena.ProjectileCharacterizationTest','--rerun-tasks','--console=plain'],cwd=ROOT,env=env,check=True)
    if before!=inputs(up):raise RuntimeError('Source changed during selected test; no receipt')
    xml=ROOT/'build/test-results/test/TEST-frc.maplearena.ProjectileCharacterizationTest.xml';test=ET.parse(xml).getroot()
    if test.get('tests')!='1' or test.get('failures')!='0' or test.get('errors')!='0':raise RuntimeError('Test failed')
    report=ROOT/'artifacts/projectile-characterization.json'
    receipt={'schema':'maple-projectile-source-bound/1','upstreamCommit':COMMIT,'selectedSuite':'frc.maplearena.ProjectileCharacterizationTest','testCount':1,'sourceSha256':before,'sourceStable':True,'reportSha256':sha(report),'junitSha256':sha(xml),'scope':'Airborne obstruction limitation characterization, not accepted shot physics.'}
    dest.write_text(json.dumps(receipt,indent=2)+'\n')
if __name__=='__main__':main()
