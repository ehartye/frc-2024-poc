package org.agentfrc.lesson;

/** Injected-observation mission scaffold, not proof of physical acquisition or safe outputs. */
public final class CrescendoMissionSupervisor {
 public enum State { WAIT_START, DRIVE_SHOT, ALIGN_SHOT, SPINUP, RELEASE, SEEK_NOTE, ACQUIRE, JAM_REVERSE, INDEX, STOW, RETURN_SHOT, DONE, FAULT, DEGRADED_PLAN }
 public enum Piece { PRESENT, ABSENT, UNKNOWN }
 public enum Goal { NONE, SHOT_POSE, FLOOR_NOTE }
 public enum Fault { NONE, OBSERVATION, UNKNOWN_PIECE, PRELOAD, LOST_PIECE, LOW_RPM, TIMEOUT, ACQUISITION_EXHAUSTED, JAM_EXHAUSTED, RELEASE_CONTRADICTION }
 public record Config(double maxAge, double maxUpdateGap, double driveTimeout, double acquireTimeout,
   double indexTimeout, double stowTimeout, double spinupTimeout, double releaseTimeout,
   double reverseDuration, double readyDwell, double readyRpm, double intakeDuty, double indexDuty, double releaseDuty) {
  public Config {
   double[] positive={maxAge,maxUpdateGap,driveTimeout,acquireTimeout,indexTimeout,stowTimeout,spinupTimeout,releaseTimeout,reverseDuration,readyDwell,readyRpm};
   for(double v:positive) if(!Double.isFinite(v)||v<=0) throw new IllegalArgumentException("positive finite timing/threshold required");
   for(double v:new double[]{intakeDuty,indexDuty,releaseDuty}) if(!Double.isFinite(v)||v<=0||v>1) throw new IllegalArgumentException("duty in (0,1]");
   if(reverseDuration>=acquireTimeout||readyDwell>=spinupTimeout) throw new IllegalArgumentException("timing ordering");
  }
  public static Config hypothesis(){return new Config(.05,.10,12,2,1,1,2,.7,.20,.10,2500,.35,.15,.20);}
 }
 /** timestamp and sequence must advance. reachedGoal identifies the commanded goal, not an arbitrary pose. */
 public record Observation(long sequence,double timestamp,Piece piece,boolean indexed,boolean stowed,
   Goal reachedGoal,boolean aligned,boolean stopped,boolean floorTargetVisible,boolean jammed,
   double shooterRpm,boolean downstreamExitObserved) {}
 public record Output(State state,Fault fault,Goal driveGoal,boolean alignToShot,boolean deployIntake,
   boolean stowIntake,double intakeDuty,double indexDuty,double shooterTargetRpm,double releaseDuty,
   boolean degradedPlanningRequested,int completedReleases,int acquisitionRetries,int jamReversals) {}
 private final Config c; private State state=State.WAIT_START; private Fault reason=Fault.NONE;
 private long lastSequence=-1; private double lastNow=-1,lastStamp=-1,entered=0,readySince=-1;
 private int releases=0,retries=0,reversals=0;
 public CrescendoMissionSupervisor(Config config){if(config==null)throw new IllegalArgumentException("config");c=config;}
 private void transition(State next,double now){state=next;entered=now;readySince=-1;}
 private Output fail(Fault fault,double now){reason=fault;transition(State.FAULT,now);return output();}
 private boolean held(Observation o){return o.piece()==Piece.PRESENT&&o.indexed()&&o.stowed();}
 private Output output(){
  Goal goal=state==State.DRIVE_SHOT||state==State.RETURN_SHOT?Goal.SHOT_POSE:state==State.SEEK_NOTE?Goal.FLOOR_NOTE:Goal.NONE;
  return new Output(state,reason,goal,state==State.ALIGN_SHOT,state==State.ACQUIRE||state==State.INDEX||state==State.JAM_REVERSE,state==State.STOW,
   state==State.ACQUIRE?c.intakeDuty():state==State.JAM_REVERSE?-c.intakeDuty():0,
   state==State.INDEX?c.indexDuty():state==State.JAM_REVERSE?-c.indexDuty():0,
   state==State.SPINUP||state==State.RELEASE?c.readyRpm():0,state==State.RELEASE?c.releaseDuty():0,
   state==State.DEGRADED_PLAN,releases,retries,reversals);
 }
 public Output update(double now,Observation o,boolean start,boolean requestDegradedPlan){
  // Terminal inhibition persists; planning cannot reactivate mechanisms or fabricate a shorter shot.
  if(state==State.DEGRADED_PLAN||state==State.DONE)return output();
  if(state==State.FAULT){if(reason==Fault.LOW_RPM&&requestDegradedPlan)transition(State.DEGRADED_PLAN,now);return output();}
  if(!Double.isFinite(now)||now<0||o==null||o.piece()==null||o.reachedGoal()==null||o.sequence()<0||o.sequence()<=lastSequence||!Double.isFinite(o.timestamp())||o.timestamp()<0||o.timestamp()>now||now-o.timestamp()>c.maxAge()+1e-9||o.timestamp()<=lastStamp||lastNow>=0&&(now<=lastNow||now-lastNow>c.maxUpdateGap()+1e-9)||!Double.isFinite(o.shooterRpm())||o.shooterRpm()<0)return fail(Fault.OBSERVATION,now);
  lastSequence=o.sequence();lastNow=now;lastStamp=o.timestamp();
  if(o.piece()==Piece.UNKNOWN)return fail(Fault.UNKNOWN_PIECE,now);
  double age=now-entered;
  switch(state){
   case WAIT_START -> {if(start){if(!held(o))return fail(Fault.PRELOAD,now);transition(State.DRIVE_SHOT,now);}}
   case DRIVE_SHOT, RETURN_SHOT -> {
    if(!held(o))return fail(Fault.LOST_PIECE,now);
    if(o.reachedGoal()==Goal.SHOT_POSE&&o.stopped())transition(State.ALIGN_SHOT,now);
    else if(age>=c.driveTimeout())return fail(Fault.TIMEOUT,now);
   }
   case ALIGN_SHOT -> {
    if(!held(o))return fail(Fault.LOST_PIECE,now);
    if(o.aligned()&&o.stopped())transition(State.SPINUP,now);
    else if(age>=c.driveTimeout())return fail(Fault.TIMEOUT,now);
   }
   case SPINUP -> {
    if(!held(o))return fail(Fault.LOST_PIECE,now);
    if(!o.aligned()||!o.stopped()){transition(State.ALIGN_SHOT,now);break;}
    if(o.shooterRpm()>=c.readyRpm()){if(readySince<0)readySince=now;if(now-readySince+1e-9>=c.readyDwell())transition(State.RELEASE,now);}else readySince=-1;
    if(state==State.SPINUP&&age>=c.spinupTimeout())return fail(Fault.LOW_RPM,now);
   }
   case RELEASE -> {
    if(!o.aligned()||!o.stopped()||o.shooterRpm()<c.readyRpm())return fail(Fault.RELEASE_CONTRADICTION,now);
    if(o.piece()==Piece.ABSENT&&o.downstreamExitObserved()){releases++;transition(releases==2?State.DONE:State.SEEK_NOTE,now);}
    else if(age>=c.releaseTimeout())return fail(Fault.TIMEOUT,now);
   }
   case SEEK_NOTE -> {
    if(o.piece()!=Piece.ABSENT)return fail(Fault.RELEASE_CONTRADICTION,now);
    if(o.floorTargetVisible()&&o.reachedGoal()==Goal.FLOOR_NOTE&&o.stopped())transition(State.ACQUIRE,now);
    else if(age>=c.driveTimeout())return fail(Fault.TIMEOUT,now);
   }
   case ACQUIRE, INDEX -> {
    if(!o.stopped())return fail(Fault.OBSERVATION,now);
    if(o.jammed()){if(reversals>=1)return fail(Fault.JAM_EXHAUSTED,now);reversals++;transition(State.JAM_REVERSE,now);}
    else if(state==State.ACQUIRE&&o.piece()==Piece.PRESENT)transition(State.INDEX,now);
    else if(state==State.INDEX&&o.piece()==Piece.PRESENT&&o.indexed())transition(State.STOW,now);
    else if(age>=(state==State.ACQUIRE?c.acquireTimeout():c.indexTimeout())){
     if(retries>=1)return fail(Fault.ACQUISITION_EXHAUSTED,now);retries++;transition(o.piece()==Piece.ABSENT?State.SEEK_NOTE:State.INDEX,now);
    }
   }
   case JAM_REVERSE -> {
    if(!o.stopped())return fail(Fault.OBSERVATION,now);
    if(age+1e-9>=c.reverseDuration())transition(o.piece()==Piece.ABSENT?State.ACQUIRE:State.INDEX,now);
   }
   case STOW -> {
    if(o.piece()!=Piece.PRESENT||!o.indexed())return fail(Fault.LOST_PIECE,now);
    if(o.stowed()&&o.stopped())transition(State.RETURN_SHOT,now);
    else if(age>=c.stowTimeout())return fail(Fault.TIMEOUT,now);
   }
   default -> throw new IllegalStateException("unhandled state");
  }
  return output();
 }
}
