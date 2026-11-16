package org.agentfrc.lesson;
import org.junit.jupiter.api.*;
import static org.junit.jupiter.api.Assertions.*;
import java.util.*;
import java.nio.file.*;
class A2MissionSimulationTest {
 static final Map<A2MissionSimulation.Scenario,A2MissionSimulation.Result> results=new EnumMap<>(A2MissionSimulation.Scenario.class);
 static A2MissionSimulation.Result run(A2MissionSimulation.Scenario scenario){return results.computeIfAbsent(scenario,s->new A2MissionSimulation(s).run());}
 @Test void fullTwoShotCyclesUseActualShooterAndGeometricVirtualCapture(){for(var scenario:List.of(A2MissionSimulation.Scenario.NORMAL,A2MissionSimulation.Scenario.NORMAL_9V,A2MissionSimulation.Scenario.STRICT_12V)){
  var r=run(scenario);System.out.printf("A2_CYCLE scenario=%s state=%s missionFault=%s shooterFault=%s time=%.6f captures=%d releases=%d residual=%.12g%n",scenario,r.state(),r.missionFault(),r.shooterFault(),r.time(),r.captures(),r.releases(),r.maxEnergyResidual());
  assertEquals(CrescendoMissionSupervisor.State.DONE,r.state());assertEquals(1,r.captures());assertEquals(2,r.releases());assertEquals(2,r.ledger().departedBodies());assertTrue(r.ledger().outgoingBodyEnergy()>0);assertEquals(0,r.ledger().cumulativeFeedEnergy());assertTrue(r.maxEnergyResidual()<2e-5);assertTrue(r.exits().get(1).time()>r.exits().get(0).time());
  for(var e:r.exits()){assertTrue(e.speed()>0);assertEquals(e.speed()*e.speed(),e.vx()*e.vx()+e.vy()*e.vy()+e.vz()*e.vz(),1e-9);assertEquals(.7,e.z(),1e-12);}
 }}
 @Test void missingJamLowSupplyAndPoseFailuresDoNotInventCompletion(){
  var missing=run(A2MissionSimulation.Scenario.MISSING_NOTE);assertEquals(CrescendoMissionSupervisor.Fault.TIMEOUT,missing.missionFault());assertEquals(0,missing.captures());assertEquals(1,missing.releases());
  var jam=run(A2MissionSimulation.Scenario.JAM);assertEquals(CrescendoMissionSupervisor.Fault.JAM_EXHAUSTED,jam.missionFault());assertEquals(1,jam.releases());
  var stale=run(A2MissionSimulation.Scenario.STALE_POSE);assertEquals(CrescendoMissionSupervisor.Fault.OBSERVATION,stale.missionFault());assertEquals(0,stale.releases());
  var low=run(A2MissionSimulation.Scenario.STRICT_9V);assertEquals(A2ShooterAdapter.Fault.LOW_RELEASE_RPM,low.shooterFault());assertEquals(0,low.releases());assertEquals(CrescendoMissionSupervisor.State.FAULT,low.state());
 }
 @Test void identicalInputsProduceIdenticalTrace(){assertEquals(run(A2MissionSimulation.Scenario.NORMAL).trace(),new A2MissionSimulation(A2MissionSimulation.Scenario.NORMAL).run().trace());}
 @Test void traceSchemaTimeAndStatesAreObservable(){
  var r=run(A2MissionSimulation.Scenario.NORMAL);assertEquals(A2MissionSimulation.HEADER,r.trace().get(0));double previous=-1;Set<String> states=new HashSet<>();int events=0;
  for(String row:r.trace().subList(1,r.trace().size())){String[] c=row.split(",",-1);assertEquals(29,c.length);double time=Double.parseDouble(c[0]);assertTrue(time>=previous);previous=time;states.add(c[1]);if(c[20].equals("EXIT")){events++;assertFalse(c[21].isEmpty());assertTrue(c[25].isEmpty());}}
  assertEquals(2,events);assertTrue(states.containsAll(List.of("ACQUIRE","INDEX","STOW","RETURN_SHOT","DONE")));
 }
 static void hashClass(Class<?> c,Properties p)throws Exception{
  String n=c.getName(),leaf=n.substring(n.lastIndexOf('.')+1)+".class";
  try(var actual=c.getResourceAsStream(leaf)){p.setProperty(n,HexFormat.of().formatHex(java.security.MessageDigest.getInstance("SHA-256").digest(actual.readAllBytes())));}
  for(Class<?> nested:c.getDeclaredClasses())hashClass(nested,p);
 }
 @AfterAll static void emitBoundRun()throws Exception{
  String token=System.getenv("FRC_MISSION_RUN_ID"),directory=System.getenv("FRC_MISSION_OUTPUT");if(token==null&&directory==null)return;assertNotNull(token);assertNotNull(directory);Path dir=Path.of(directory);Files.createDirectories(dir);assertEquals(7,results.size());
  for(var scenario:A2MissionSimulation.Scenario.values()){var r=results.get(scenario);A2MissionSimulation.write(r,dir);System.out.printf("A2_RESULT scenario=%s state=%s missionFault=%s shooterFault=%s releases=%d time=%.9f%n",scenario,r.state(),r.missionFault(),r.shooterFault(),r.releases(),r.time());}
  var p=new Properties();p.setProperty("runId",token);for(Class<?> c:new Class<?>[]{A2MissionSimulation.class,A2MissionGeometry.class,CrescendoMissionSupervisor.class,Class.forName("org.agentfrc.lesson.CrescendoMissionSupervisor$1"),A2ShooterAdapter.class,PoweredGuidePlant.class,SharedSupply.class,A2MissionSimulationTest.class,edu.wpi.first.math.system.plant.DCMotor.class,edu.wpi.first.math.geometry.Pose2d.class,edu.wpi.first.math.kinematics.DifferentialDriveKinematics.class})hashClass(c,p);
  try(var out=Files.newOutputStream(dir.resolve("runtime.properties"),StandardOpenOption.CREATE_NEW)){p.store(out,"Fresh A2 mission execution");}System.out.println("MISSION_RUN_ID="+token);
 }
}
