package org.agentfrc.lesson;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
import static org.agentfrc.lesson.CrescendoMissionSupervisor.*;

class CrescendoMissionSupervisorTest {
 static Config quick(){return new Config(.05,.1,.4,.2,.2,.2,.3,.2,.1,.05,2500,.35,.15,.2);}
 static class Rig {
  CrescendoMissionSupervisor s=new CrescendoMissionSupervisor(quick());double t=0,rpm=3000;long seq=0;
  Piece piece=Piece.PRESENT;Goal goal=Goal.NONE;boolean indexed=true,stowed=true,aligned=true,stopped=true,target=false,jam=false,exit=false;
  Observation observation(){return new Observation(seq++,t,piece,indexed,stowed,goal,aligned,stopped,target,jam,rpm,exit);}
  Output step(){return step(false,false);}
  Output step(boolean start,boolean plan){t+=.05;Output o=s.update(t,observation(),start,plan);assertSerialized(o);return o;}
  Output preload(){return step(true,false);}
  Output readyShot(){goal=Goal.SHOT_POSE;assertEquals(State.ALIGN_SHOT,step().state());assertEquals(State.SPINUP,step().state());step();return step();}
  Output release(){piece=Piece.ABSENT;exit=true;return step();}
  void acquire(){preload();assertEquals(State.RELEASE,readyShot().state());assertEquals(State.SEEK_NOTE,release().state());exit=false;indexed=false;stowed=false;goal=Goal.FLOOR_NOTE;target=true;assertEquals(State.ACQUIRE,step().state());}
 }
 static void assertSerialized(Output o){
  if(o.driveGoal()!=Goal.NONE||o.alignToShot()){assertEquals(0,o.intakeDuty());assertEquals(0,o.indexDuty());assertEquals(0,o.releaseDuty());assertEquals(0,o.shooterTargetRpm());}
  if(o.state()==State.FAULT||o.state()==State.DEGRADED_PLAN||o.state()==State.DONE){assertEquals(Goal.NONE,o.driveGoal());assertFalse(o.alignToShot());assertFalse(o.deployIntake());assertFalse(o.stowIntake());assertEquals(0,o.intakeDuty());assertEquals(0,o.indexDuty());assertEquals(0,o.releaseDuty());assertEquals(0,o.shooterTargetRpm());}
 }
 @Test void injectedTwoReleaseMissionIsSerializedNotPhysicalProof(){
  Rig r=new Rig();r.acquire();r.piece=Piece.PRESENT;assertEquals(State.INDEX,r.step().state());r.indexed=true;assertEquals(State.STOW,r.step().state());r.stowed=true;assertEquals(State.RETURN_SHOT,r.step().state());assertEquals(State.RELEASE,r.readyShot().state());Output done=r.release();assertEquals(State.DONE,done.state());assertEquals(2,done.completedReleases());assertEquals(0,done.acquisitionRetries());
 }
 @Test void acquisitionRetriesExactlyOnce(){
  Rig r=new Rig();r.acquire();Output o=null;for(int n=0;n<5;n++)o=r.step();assertEquals(State.SEEK_NOTE,o.state());assertEquals(1,o.acquisitionRetries());assertEquals(State.ACQUIRE,r.step().state());for(int n=0;n<6;n++)o=r.step();assertEquals(State.FAULT,o.state());assertEquals(Fault.ACQUISITION_EXHAUSTED,o.fault());assertEquals(1,o.acquisitionRetries());
 }
 @Test void jamReverseIsTimedAndSingleUse(){
  Rig r=new Rig();r.acquire();r.jam=true;Output o=r.step();assertEquals(State.JAM_REVERSE,o.state());assertTrue(o.intakeDuty()<0);r.jam=false;assertEquals(State.JAM_REVERSE,r.step().state());assertEquals(State.ACQUIRE,r.step().state());r.jam=true;o=r.step();assertEquals(Fault.JAM_EXHAUSTED,o.fault());assertEquals(1,o.jamReversals());
 }
 @Test void lowRpmOnlyAllowsOperatorRequestedInhibitedPlanning(){
  Rig r=new Rig();r.preload();r.goal=Goal.SHOT_POSE;r.step();r.step();r.rpm=0;Output o=null;for(int n=0;n<8;n++)o=r.step();assertEquals(Fault.LOW_RPM,o.fault());assertFalse(o.degradedPlanningRequested());assertEquals(State.FAULT,r.step().state());o=r.step(false,true);assertEquals(State.DEGRADED_PLAN,o.state());assertTrue(o.degradedPlanningRequested());r.rpm=3000;assertEquals(State.DEGRADED_PLAN,r.step(true,false).state());
 }
 @Test void unknownAndStaleObservationsInhibit(){
  Rig r=new Rig();r.piece=Piece.UNKNOWN;Output o=r.preload();assertEquals(Fault.UNKNOWN_PIECE,o.fault());assertSerialized(o);
  Rig q=new Rig();q.preload();q.t+=.2;o=q.s.update(q.t,q.observation(),false,false);assertEquals(Fault.OBSERVATION,o.fault());assertSerialized(o);
  Rig f=new Rig();f.t=.2;Observation old=new Observation(0,0,Piece.PRESENT,true,true,Goal.NONE,true,true,false,false,3000,false);o=f.s.update(.2,old,true,false);assertEquals(Fault.OBSERVATION,o.fault());
 }
 @Test void releaseNeedsAbsentPieceAndExitObservation(){
  Rig r=new Rig();r.preload();r.readyShot();r.piece=Piece.ABSENT;r.exit=false;Output o=null;for(int n=0;n<6;n++)o=r.step();assertEquals(Fault.TIMEOUT,o.fault());assertEquals(0,o.completedReleases());
 }
 @Test void releaseLosingRpmFaultsWithoutFallbackShot(){
  Rig r=new Rig();r.preload();r.readyShot();r.rpm=100;oCheck(r.step(),Fault.RELEASE_CONTRADICTION);
 }
 static void oCheck(Output o,Fault f){assertEquals(State.FAULT,o.state());assertEquals(f,o.fault());assertSerialized(o);}
 @Test void lostPieceAndMovingAcquisitionFault(){
  Rig r=new Rig();r.preload();r.piece=Piece.ABSENT;oCheck(r.step(),Fault.LOST_PIECE);
  Rig q=new Rig();q.acquire();q.stopped=false;oCheck(q.step(),Fault.OBSERVATION);
 }
 @Test void loadedReleaseFloorIsSeparateAndExplicit(){
  Rig r=new Rig();r.s=new CrescendoMissionSupervisor(new Config(.05,.1,.4,.2,.2,.2,.3,.2,.1,.05,2500,1000,.35,.15,.2));r.preload();r.readyShot();r.rpm=1500;assertEquals(State.RELEASE,r.step().state());r.rpm=999;oCheck(r.step(),Fault.RELEASE_CONTRADICTION);
  assertEquals(2500,quick().releaseMinRpm());
 }
 @Test void repeatedSequenceInvalidNumbersAndInvalidConfigRejected(){
  Rig r=new Rig();r.preload();r.seq--;oCheck(r.step(),Fault.OBSERVATION);
  Rig q=new Rig();q.rpm=Double.NaN;oCheck(q.preload(),Fault.OBSERVATION);
  assertThrows(IllegalArgumentException.class,()->new CrescendoMissionSupervisor(null));
  assertThrows(IllegalArgumentException.class,()->new Config(.05,.1,.4,.2,.2,.2,.3,.2,.1,.05,Double.NaN,.35,.15,.2));
 }
}
