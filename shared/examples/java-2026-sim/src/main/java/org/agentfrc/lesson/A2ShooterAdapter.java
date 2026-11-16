package org.agentfrc.lesson;
/** One continuous fixed-duty plant, with hypothesis-only readiness and loaded-RPM gates. */
public final class A2ShooterAdapter {
 public enum State { OFF, SPINUP, READY, FEEDING, EXITED, FAULT }
 public enum Fault { NONE, COMMAND_UNSUPPORTED, PREMATURE_RELEASE, STARTUP_TIMEOUT, RELEASE_TIMEOUT, LOW_RELEASE_RPM }
 public record Config(PoweredGuidePlant.Config plant,double readyRpm,double releaseFloorRpm,double readyDwell,double startupTimeout,double releaseTimeout) {
  public Config {
   if(plant==null)throw new IllegalArgumentException("plant config");
   for(double v:new double[]{readyRpm,releaseFloorRpm,readyDwell,startupTimeout,releaseTimeout})if(!Double.isFinite(v)||v<=0)throw new IllegalArgumentException("positive finite threshold");
   if(releaseFloorRpm>readyRpm||readyDwell>=startupTimeout)throw new IllegalArgumentException("threshold ordering");
  }
 }
 public record Result(State state,Fault fault,PoweredGuidePlant.Sample sample,double actualRpm,boolean physicsSuspended,
   PoweredGuidePlant.Sample releaseBoundaryBefore,PoweredGuidePlant.Sample releaseBoundaryAfter){}
 private final Config c;private final PoweredGuidePlant plant;private State state=State.OFF;private Fault fault=Fault.NONE;
 private double readySince=-1,feedTime=-1;private PoweredGuidePlant.Sample before,after;
 public A2ShooterAdapter(Config config){c=java.util.Objects.requireNonNull(config);plant=new PoweredGuidePlant(c.plant());}
 private static double rpm(PoweredGuidePlant.Sample s){return s.omega()*60/(2*Math.PI);}
 public Result sample(){var s=plant.sample();return new Result(state,fault,s,rpm(s),state==State.FAULT||state==State.EXITED,before,after);}
 private Result fail(Fault f){fault=f;state=State.FAULT;return sample();}
 /** Duty must equal configured plant duty while enabled. Disabling a spinning plant is unsupported,
  * suspends this model and latches FAULT; it does not set RPM to zero or simulate braking/coasting. */
 public Result update(double dt,boolean enabled,double requestedDuty,boolean requestRelease){
  if(!Double.isFinite(dt)||dt<.00001||dt>.0002)throw new IllegalArgumentException("integration timestep");
  if(state==State.FAULT||state==State.EXITED)return sample();
  if(!Double.isFinite(requestedDuty)||requestedDuty<0||requestedDuty>.8)return fail(Fault.COMMAND_UNSUPPORTED);
  if(state==State.OFF&&!enabled&&!requestRelease&&requestedDuty==0)return sample();
  if(!enabled||Double.compare(requestedDuty,c.plant().duty())!=0)return fail(Fault.COMMAND_UNSUPPORTED);
  if(state==State.OFF)state=State.SPINUP;
  if(requestRelease){
   if(state==State.READY){before=plant.sample();plant.feed(0);after=plant.sample();feedTime=after.time();state=State.FEEDING;}
   else if(state!=State.FEEDING)return fail(Fault.PREMATURE_RELEASE);
  }
  var s=plant.step(dt);double actual=rpm(s);
  if(state==State.SPINUP||state==State.READY){
   if(actual>=c.readyRpm()){
    if(readySince<0)readySince=s.time();
    if(s.time()-readySince+1e-12>=c.readyDwell())state=State.READY;
   }else{readySince=-1;state=State.SPINUP;}
   if(state==State.SPINUP&&s.time()>=c.startupTimeout())return fail(Fault.STARTUP_TIMEOUT);
  }else{
   if(actual<c.releaseFloorRpm())return fail(Fault.LOW_RELEASE_RPM);
   if(s.exited())state=State.EXITED;
   else if(s.time()-feedTime>=c.releaseTimeout())return fail(Fault.RELEASE_TIMEOUT);
  }
  return sample();
 }
}
