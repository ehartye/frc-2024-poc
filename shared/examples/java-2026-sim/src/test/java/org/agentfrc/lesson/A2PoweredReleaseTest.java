package org.agentfrc.lesson;
import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
/** A2 sensitivity hypotheses, not component specifications or physical qualification. */
class A2PoweredReleaseTest {
 @Test void sweepPrescribedA2Guide(){for(double volts:new double[]{9,12})for(double normal:new double[]{15,25,35})for(double duty:new double[]{.6,.8}){
  var p=new PoweredGuidePlant(new PoweredGuidePlant.Config(.235,.0508,.0003,.24,Math.toRadians(60),normal,.8,duty,volts));p.feed(0);var s=p.sample();for(int k=0;k<40000&&!s.exited();k++)s=p.step(.00005);assertTrue(s.exited());assertEquals(0,s.feedEnergy());assertTrue(Math.abs(s.energyResidual())<.01);
  System.out.println("A2_PLANT volts="+volts+" normalEach="+normal+" duty="+duty+" speed="+s.speed()+" time="+s.time()+" residual="+s.energyResidual()+" omega="+s.omega());
 }}
}
