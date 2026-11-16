package org.agentfrc.lesson;
import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
/** A2 sensitivity hypotheses, not component specifications or physical qualification. */
class A2PoweredReleaseTest {
 @Test void configuredGuideRetainsProtocolAndFaultControls(){
  var r=IntegratedPregripPrototype.runConfigured(2024,.235,12,.00005,0,.0003,.8,Math.toRadians(60),.24,25,.8);
  assertEquals(PregripSupervisor.State.DONE,r.state());assertEquals(7.257734126538242,r.plant().speed(),1e-9);assertEquals(.8,r.commandedDuty());
  for(int f:new int[]{1,2,3,4})assertEquals(PregripSupervisor.State.FAULT,IntegratedPregripPrototype.runConfigured(2024,.235,12,.00005,f,.0003,.8,Math.toRadians(60),.24,25,.8).state());
  var stopped=IntegratedPregripPrototype.runConfigured(2024,.235,12,.00005,0,.0003,0,Math.toRadians(60),.24,25,.8);assertFalse(stopped.plant().exited());assertEquals(0,stopped.commandedDuty());
 }
 @Test void sweepPrescribedA2Guide(){for(double volts:new double[]{9,12})for(double normal:new double[]{15,25,35})for(double duty:new double[]{.6,.8}){
  var p=new PoweredGuidePlant(new PoweredGuidePlant.Config(.235,.0508,.0003,.24,Math.toRadians(60),normal,.8,duty,volts));p.feed(0);var s=p.sample();for(int k=0;k<40000&&!s.exited();k++)s=p.step(.00005);assertTrue(s.exited());assertEquals(0,s.feedEnergy());assertTrue(Math.abs(s.energyResidual())<.01);
  System.out.println("A2_PLANT volts="+volts+" normalEach="+normal+" duty="+duty+" speed="+s.speed()+" time="+s.time()+" residual="+s.energyResidual()+" omega="+s.omega());
 }}
}
