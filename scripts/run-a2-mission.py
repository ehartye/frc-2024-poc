"""Fresh atomic native mission run; never relabel an existing XML/CSV as a new execution."""
import pathlib,json,hashlib,os,subprocess,tempfile,uuid,zipfile,csv,importlib.util,xml.etree.ElementTree as ET
ROOT=pathlib.Path(__file__).resolve().parents[1]
PROJECT=ROOT/'shared/examples/java-2026-sim'
TESTS={'fullTwoShotCyclesUseActualShooterAndGeometricVirtualCapture()','missingJamLowSupplyAndPoseFailuresDoNotInventCompletion()','traceSchemaTimeAndStatesAreObservable()','identicalInputsProduceIdenticalTrace()'}
SCENARIOS=['NORMAL','NORMAL_9V','MISSING_NOTE','JAM','STALE_POSE','STRICT_9V','STRICT_12V']
ROOT_CLASSES=['A2MissionSimulation','A2MissionGeometry','CrescendoMissionSupervisor','A2ShooterAdapter','PoweredGuidePlant','SharedSupply','A2MissionSimulationTest']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def pins():
 files=list((PROJECT/'src').rglob('*.java'))+[PROJECT/n for n in ['build.gradle','settings.gradle','gradle.lockfile','gradle.properties','gradlew.bat','gradle/wrapper/gradle-wrapper.jar','gradle/wrapper/gradle-wrapper.properties']]+[ROOT/n for n in ['scripts/run-a2-mission.py','scripts/run-a2-mission.ps1','scripts/generate-mission-geometry.py','mission/cycle-contract-A2.json','mission/a2_kinematics.py','cad/cycle-contract-A2.json','cad/a2_kinematics.py']]
 return {p.relative_to(ROOT).as_posix():sha(p) for p in sorted(files)}
def validate_xml(data,token):
 tree=ET.fromstring(data)
 if {n.get('name') for n in tree.findall('testcase')}!=TESTS or tree.get('tests')!=str(len(TESTS)) or any(tree.get(k)!='0' for k in ['failures','errors','skipped']):raise ValueError('incomplete mission tests')
 lines=[l.strip() for n in tree.findall('system-out') for l in (n.text or '').splitlines()]
 if lines.count('MISSION_RUN_ID='+token)!=1:raise ValueError('XML run binding mismatch')
 results=[l for l in lines if l.startswith('A2_RESULT ')]
 if len(results)!=7 or {l.split()[1].split('=')[1] for l in results}!=set(SCENARIOS):raise ValueError('scenario inventory')
 return results

def check_canonical_geometry(root=ROOT):
 for name in ['cycle-contract-A2.json','a2_kinematics.py']:
  if (root/'cad'/name).read_bytes()!=(root/'mission'/name).read_bytes():raise ValueError('Canonical CAD and mission snapshot differ: '+name)

def run(output,java_home,gradle_home):
 out=pathlib.Path(output).resolve();jdk=pathlib.Path(java_home);cache=pathlib.Path(gradle_home);java=jdk/'bin/java.exe'
 if out.exists():raise ValueError('fresh output required')
 if not java.is_file() or not cache.is_dir():raise ValueError('existing JDK/cache required; no installation')
 check_canonical_geometry()
 subprocess.run(['python',str(ROOT/'scripts/generate-mission-geometry.py'),'--check'],check=True)
 out.parent.mkdir(parents=True,exist_ok=True);pending=pathlib.Path(tempfile.mkdtemp(prefix=out.name+'-pending-',dir=out.parent));token=str(uuid.uuid4());before=pins()
 env=os.environ.copy();env.update(JAVA_HOME=str(jdk),GRADLE_USER_HOME=str(cache),FRC_MISSION_RUN_ID=token,FRC_MISSION_OUTPUT=str(pending));env['PATH']=str(java.parent)+os.pathsep+env['PATH']
 with (pending/'gradle.log').open('w') as log:r=subprocess.run([str(PROJECT/'gradlew.bat'),'test','--tests','org.agentfrc.lesson.A2MissionSimulationTest','--offline','--no-daemon','--rerun-tasks'],cwd=PROJECT,env=env,stdout=log,stderr=subprocess.STDOUT)
 if r.returncode:raise RuntimeError('native mission failure; pending diagnostics retained')
 if before!=pins():raise ValueError('inputs changed during execution')
 xml=(PROJECT/'build/test-results/test/TEST-org.agentfrc.lesson.A2MissionSimulationTest.xml').read_bytes();results=validate_xml(xml,token);(pending/'tests.xml').write_bytes(xml)
 runtime={line.split('=',1)[0]:line.split('=',1)[1] for line in (pending/'runtime.properties').read_text().splitlines() if '=' in line and not line.startswith('#')}
 if runtime.pop('runId',None)!=token:raise ValueError('runtime token mismatch')
 actual={}
 for scope in ['main','test']:
  for p in (PROJECT/f'build/classes/java/{scope}/org/agentfrc/lesson').glob('*.class'):
   if p.stem.split('$')[0] in ROOT_CLASSES:actual['org.agentfrc.lesson.'+p.stem]=sha(p)
 jars=list(cache.glob('caches/modules-2/files-2.1/edu.wpi.first.wpimath/wpimath-java/2026.2.1/*/*.jar'))
 if len(jars)!=1:raise ValueError('ambiguous WPIMath runtime')
 with zipfile.ZipFile(jars[0]) as z:
  for name in runtime:
   if name.startswith('edu.wpi.first.math.'):
    if not any(name==n or name.startswith(n+'$') for n in ['edu.wpi.first.math.system.plant.DCMotor','edu.wpi.first.math.geometry.Pose2d','edu.wpi.first.math.kinematics.DifferentialDriveKinematics']):raise ValueError('unexpected external class')
    actual[name]=hashlib.sha256(z.read(name.replace('.','/')+'.class')).hexdigest()
 if actual!=runtime:raise ValueError('runtime class inventory/hash mismatch')
 spec=importlib.util.spec_from_file_location('kinematics',ROOT/'mission/a2_kinematics.py');kin=importlib.util.module_from_spec(spec);spec.loader.exec_module(kin);intake=json.loads((ROOT/'mission/cycle-contract-A2.json').read_text())['geometry']['intake']
 for scenario in SCENARIOS:
  with (pending/(scenario+'.csv')).open() as stream:
   for row in csv.DictReader(stream):
    expected_length=kin.pose(intake,float(row['intakeAngleDeg']))['length']
    if abs(float(row['intakeLengthM'])-expected_length)>1e-10:raise ValueError('native telescope differs from pinned CAD kinematics')
  metadata=json.loads((pending/(scenario+'.json')).read_text())
  if metadata['scenario']!=scenario or metadata['contractSha256']!=sha(ROOT/'mission/cycle-contract-A2.json'):raise ValueError('contract/scenario mismatch')
 expected={s+ext for s in SCENARIOS for ext in ['.csv','.json']}|{'runtime.properties','gradle.log','tests.xml'}
 if {p.name for p in pending.iterdir()}!=expected:raise ValueError('artifact inventory')
 receipt={'schema':'frc-a2-mission-receipt/1','runId':token,'sourceCodeSha256':before,'runtimeClassSha256':actual,'javaExecutableSha256':sha(java),'wpimathJarSha256':sha(jars[0]),'tests':sorted(TESTS),'results':results,'artifactsSha256':{name:sha(pending/name) for name in sorted(expected)},'physicalCaptureValidated':False,'officialScores':None}
 (pending/'receipt.json').write_text(json.dumps(receipt,indent=2,allow_nan=False)+'\n');pending.rename(out);return receipt
if __name__=='__main__':
 import argparse
 a=argparse.ArgumentParser();a.add_argument('--output',required=True);a.add_argument('--java-home',required=True);a.add_argument('--gradle-home',required=True);v=a.parse_args();run(v.output,v.java_home,v.gradle_home);print('Fresh mission traces and receipt written.')
