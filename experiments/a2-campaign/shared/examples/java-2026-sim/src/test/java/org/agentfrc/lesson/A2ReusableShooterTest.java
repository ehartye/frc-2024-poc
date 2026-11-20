package org.agentfrc.lesson;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
class A2ReusableShooterTest {
 static A2ShooterAdapter adapter(){return new A2ShooterAdapter(new A2ShooterAdapter.Config(A2ShooterAdapterTest.plant(12,.8),1800,1000,.05,1,.5),true);}
 @Test void twoBodiesCoastWithContinuousLedger(){
  var a=adapter();A2ShooterAdapterTest.ready(a,.00005,.8);var first=A2ShooterAdapterTest.release(a,.00005,.8);var boundary=first.sample();var ledger=a.departBody();var after=a.sample().sample();assertEquals(1,ledger.departedBodies());assertTrue(ledger.outgoingBodyEnergy()>0);assertEquals(boundary.omega(),after.omega());assertEquals(boundary.time(),after.time());assertEquals(boundary.motorWork(),after.motorWork());assertEquals(boundary.frictionLoss(),after.frictionLoss());assertEquals(boundary.energyResidual(),after.energyResidual(),1e-12);
  for(int k=0;k<10000;k++)a.update(.00005,false,0,false);var coast=a.sample().sample();assertTrue(coast.time()>boundary.time()+.49);assertEquals(boundary.omega(),coast.omega());assertEquals(boundary.motorWork(),coast.motorWork());assertTrue(coast.omega()>0);
  A2ShooterAdapterTest.ready(a,.00005,.8);var second=A2ShooterAdapterTest.release(a,.00005,.8);assertTrue(second.sample().time()>coast.time());assertTrue(Math.abs(second.sample().energyResidual())<2e-5);assertEquals(2,a.departBody().departedBodies());assertTrue(a.ledger().outgoingBodyEnergy()>ledger.outgoingBodyEnergy());assertEquals(0,a.ledger().cumulativeFeedEnergy());
 }
 @Test void optInOpenCircuitDoesNotStopRotorOrRetainedEnergy(){var a=adapter();A2ShooterAdapterTest.ready(a,.00005,.8);var r=a.sample();for(int k=0;k<2000;k++)r=a.update(.00005,false,0,false);assertEquals(A2ShooterAdapter.State.OFF,r.state());assertTrue(r.actualRpm()>1800);assertFalse(r.physicsSuspended());assertEquals(0,r.sample().x());}
 @Test void powerOffDuringReleaseCanKeepPieceMoving(){var a=adapter();A2ShooterAdapterTest.ready(a,.00005,.8);var r=a.update(.00005,true,.8,true);double x=r.sample().x(),work=r.sample().motorWork();for(int k=0;k<200;k++)r=a.update(.00005,false,0,false);assertTrue(r.sample().x()>x);assertTrue(r.actualRpm()>0);assertEquals(work,r.sample().motorWork(),1e-12);assertTrue(Math.abs(r.sample().energyResidual())<2e-5);}
 @Test void departureOnlyAtExitAndDefaultSingleShotUnchanged(){assertThrows(IllegalStateException.class,()->adapter().departBody());var p=new PoweredGuidePlant(A2ShooterAdapterTest.plant(12,.8));assertThrows(IllegalStateException.class,()->p.setMotorEnabled(false));assertThrows(IllegalStateException.class,()->p.departBody());}
}
