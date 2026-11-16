package org.agentfrc.lesson;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
class A2LiftPlantTest {
 @Test void finiteTorqueLiftAndEnergy(){for(double volts:new double[]{9,12}){var p=new A2LiftPlant(new A2LiftPlant.Config(.68,2,.235,100,.85,.4,volts),Math.toRadians(-40));double target=Math.toRadians(110);var s=p.sample();for(int k=0;k<20000;k++){p.command(Math.max(-.8,Math.min(.8,2*(target-s.angle())-.5*s.angularSpeed())));s=p.step(.0005);}assertEquals(target,s.angle(),.08);assertTrue(Math.abs(s.energyResidual())<.01);System.out.println("A2_LIFT volts="+volts+" angle="+s.angle()+" omega="+s.angularSpeed()+" residual="+s.energyResidual());}}
 @Test void powerOffIsNotRetention(){var p=new A2LiftPlant(new A2LiftPlant.Config(.68,2,.235,100,.85,.4,12),Math.toRadians(-40));p.command(0);for(int k=0;k<2000;k++)p.step(.0005);assertNotEquals(Math.toRadians(-40),p.sample().angle(),.001);}
}
