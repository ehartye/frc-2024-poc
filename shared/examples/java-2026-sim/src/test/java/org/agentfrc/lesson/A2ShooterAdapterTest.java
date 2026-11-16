package org.agentfrc.lesson;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
class A2ShooterAdapterTest {
 static PoweredGuidePlant.Config plant(double volts,double duty){return new PoweredGuidePlant.Config(.235,.0508,.0003,.24,Math.toRadians(60),25,.8,duty,volts);}
 static A2ShooterAdapter adapter(double volts,double duty,double ready,double floor){return new A2ShooterAdapter(new A2ShooterAdapter.Config(plant(volts,duty),ready,floor,.05,1,.5));}
 static A2ShooterAdapter.Result ready(A2ShooterAdapter a,double dt,double duty){var r=a.sample();for(int i=0;i<200000&&r.state()!=A2ShooterAdapter.State.READY&&r.state()!=A2ShooterAdapter.State.FAULT;i++)r=a.update(dt,true,duty,false);return r;}
 static A2ShooterAdapter.Result release(A2ShooterAdapter a,double dt,double duty){var r=a.update(dt,true,duty,true);for(int i=0;i<200000&&r.state()==A2ShooterAdapter.State.FEEDING;i++)r=a.update(dt,true,duty,false);return r;}
 @Test void samePlantSpinupReleaseRetainsEnergyAndActualRpm(){
  for(double dt:new double[]{.0001,.00005,.000025})for(double volts:new double[]{9,12}){
   var a=adapter(volts,.8,1800,1000);var r=ready(a,dt,.8);assertEquals(A2ShooterAdapter.State.READY,r.state());assertTrue(r.actualRpm()>=1800);assertEquals(0,r.sample().x());assertTrue(r.sample().motorWork()>0);
   r=release(a,dt,.8);assertEquals(A2ShooterAdapter.State.EXITED,r.state());assertEquals(r.releaseBoundaryBefore(),r.releaseBoundaryAfter());assertTrue(r.sample().time()>r.releaseBoundaryBefore().time());assertTrue(r.sample().motorWork()>r.releaseBoundaryBefore().motorWork());assertEquals(0,r.sample().feedEnergy());assertTrue(Math.abs(r.sample().energyResidual())<2e-5);assertTrue(r.actualRpm()>=1000);
   System.out.printf("A2_CONTINUOUS volts=%.1f dt=%.8f spinTime=%.9f spinRpm=%.6f exitSpeed=%.9f exitRpm=%.6f motorWork=%.9f frictionLoss=%.9f residual=%.12g%n",volts,dt,r.releaseBoundaryBefore().time(),r.releaseBoundaryBefore().omega()*60/(2*Math.PI),r.sample().speed(),r.actualRpm(),r.sample().motorWork(),r.sample().frictionLoss(),r.sample().energyResidual());
  }
 }
 @Test void retainsFromRestCounterexample(){var p=new PoweredGuidePlant(plant(12,.8));p.feed(0);var r=p.sample();while(!r.exited())r=p.step(.00005);System.out.printf("A2_FROM_REST exitSpeed=%.9f exitRpm=%.6f%n",r.speed(),r.omega()*60/(2*Math.PI));assertTrue(Math.abs(r.energyResidual())<2e-5);}
 @Test void strictEqualThresholdPreservesSupplyDependentOutcome(){
  for(double volts:new double[]{9,12}){var a=adapter(volts,.8,2500,2500);assertEquals(A2ShooterAdapter.State.READY,ready(a,.00005,.8).state());var r=release(a,.00005,.8);System.out.printf("A2_STRICT volts=%.1f state=%s rpm=%.6f speed=%.6f%n",volts,r.state(),r.actualRpm(),r.sample().speed());
   if(volts==9){assertEquals(A2ShooterAdapter.Fault.LOW_RELEASE_RPM,r.fault());assertTrue(r.actualRpm()<2500);assertFalse(r.sample().exited());}
   else assertEquals(A2ShooterAdapter.State.EXITED,r.state());
  }
 }
 @Test void missionReceivesActualRpmAndActualFirstExitWithoutReset(){
  for(double volts:new double[]{9,12}){
   var a=adapter(volts,.8,1800,1000);
   var mission=new CrescendoMissionSupervisor(new CrescendoMissionSupervisor.Config(.05,.1,12,2,1,1,1,.5,.2,.05,1800,1000,.35,.15,.2));
   CrescendoMissionSupervisor.Output out=null;double now=0;long sequence=0;boolean finished=false;
   for(int tick=0;tick<300&&!finished;tick++){
    boolean exited=a.sample().sample().exited();
    var observation=new CrescendoMissionSupervisor.Observation(sequence++,now,exited?CrescendoMissionSupervisor.Piece.ABSENT:CrescendoMissionSupervisor.Piece.PRESENT,true,true,CrescendoMissionSupervisor.Goal.SHOT_POSE,true,true,false,false,a.sample().actualRpm(),exited);
    out=mission.update(now,observation,tick==0,false);
    assertNotEquals(CrescendoMissionSupervisor.State.FAULT,out.state());
    if(out.completedReleases()==1){finished=true;break;}
    boolean enabled=out.shooterTargetRpm()>0;
    for(int k=0;k<200;k++){var result=a.update(.00005,enabled,enabled?.8:0,out.releaseDuty()>0);assertNotEquals(A2ShooterAdapter.State.FAULT,result.state());if(result.state()==A2ShooterAdapter.State.EXITED)break;}
    now+=.01;
   }
   assertTrue(finished);assertEquals(CrescendoMissionSupervisor.State.SEEK_NOTE,out.state());assertEquals(a.sample().releaseBoundaryBefore(),a.sample().releaseBoundaryAfter());assertEquals(0,a.sample().sample().feedEnergy());assertTrue(Math.abs(a.sample().sample().energyResidual())<2e-5);
   System.out.printf("A2_MISSION volts=%.1f globalObservedExit=%.6f plantTime=%.9f speed=%.9f rpm=%.6f releases=%d%n",volts,now,a.sample().sample().time(),a.sample().sample().speed(),a.sample().actualRpm(),out.completedReleases());
  }
 }
 @Test void impossibleReadinessAndZeroMotorPowerTimeout(){assertEquals(A2ShooterAdapter.Fault.STARTUP_TIMEOUT,ready(adapter(12,.8,9500,1000),.0001,.8).fault());assertEquals(A2ShooterAdapter.Fault.STARTUP_TIMEOUT,ready(adapter(12,0,1800,1000),.0001,0).fault());}
 @Test void changedZeroDisabledAndPrematureCommandsNeverResetRotor(){
  for(double demand:new double[]{0,.6}){var a=adapter(12,.8,1800,1000);ready(a,.00005,.8);var before=a.sample().sample();var r=a.update(.00005,true,demand,false);assertEquals(A2ShooterAdapter.Fault.COMMAND_UNSUPPORTED,r.fault());assertEquals(before,r.sample());assertTrue(r.actualRpm()>0);assertTrue(r.physicsSuspended());}
  var a=adapter(12,.8,1800,1000);ready(a,.00005,.8);var r=a.update(.00005,false,0,false);assertEquals(A2ShooterAdapter.Fault.COMMAND_UNSUPPORTED,r.fault());assertTrue(r.actualRpm()>0);
  assertEquals(A2ShooterAdapter.Fault.PREMATURE_RELEASE,adapter(12,.8,1800,1000).update(.00005,true,.8,true).fault());
 }
}
