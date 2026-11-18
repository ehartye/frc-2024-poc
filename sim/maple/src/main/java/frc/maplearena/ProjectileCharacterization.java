package frc.maplearena;
import static edu.wpi.first.units.Units.Seconds;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.crescendo2024.*;

/** Characterization only: does not add airborne collisions to Maple. */
public final class ProjectileCharacterization {
 static class Arena extends ArenaHarness.Arena {
  void blocker(double y){Body b=new Body();b.addFixture(Geometry.createRectangle(.4,.6));b.translate(3.5,y);b.setMass(MassType.INFINITE);physicsWorld.addBody(b);}
 }
 static Arena fresh(double blockerY){Arena a=new Arena();SimulatedArena.overrideInstance(a);a.clearGamePieces();if(Double.isFinite(blockerY))a.blocker(blockerY);return a;}
 static List<Double> xyz(Pose3d p){return List.of(p.getX(),p.getY(),p.getZ());}
 static Map<String,Object> airborne(String label,double blockerY){
  Arena a=fresh(blockerY);AtomicInteger callbacks=new AtomicInteger();
  var p=new GamePieceProjectile(CrescendoNoteOnField.CRESCENDO_NOTE_INFO,new Translation2d(2,2),new Translation2d(5,0),1,4,new Rotation3d())
   .withTargetPosition(()->new Translation3d(5,2,1.42)).withTargetTolerance(new Translation3d(.05,.05,.05)).withHitTargetCallBack(callbacks::incrementAndGet);
  a.addGamePieceProjectile(p);boolean prediction=p.willHitTarget();List<List<Double>> rows=new ArrayList<>();
  for(int i=0;i<35;i++){ArenaHarness.step(a,1);rows.add(xyz(p.getPose3d()));}
  ArenaHarness.check(prediction && callbacks.get()==1,"Analytic target callback occurs once");
  var out=new LinkedHashMap<String,Object>();out.put("case",label);out.put("targetPredicted",prediction);out.put("callbackCount",callbacks.get());out.put("trace20ms",rows);a.shutDown();return out;
 }
 static double ground(double blockerY){Arena a=fresh(blockerY);var n=new CrescendoNoteOnField(new Translation2d(3,2));n.setLinearVelocity(5,0);a.addGamePiece(n);double maxX=3;for(int i=0;i<60;i++){ArenaHarness.step(a,1);maxX=Math.max(maxX,n.getPoseOnField().getX());}a.shutDown();return maxX;}
 static Map<String,Object> swept(){
  // Independently specified analytic path, not Maple getPose helper. A finite sphere
  // bounds the 14-inch diameter, 2-inch thick NOTE at any orientation.
  double radius=Math.hypot(.1778,.0254),t=.3,x=2+5*t,z=1+4*t-5.5*t*t;
  boolean contained=Math.abs(x-3.5)+radius<.2 && radius<.3 && Math.abs(z-1.705)+radius<.3;
  boolean displacedSeparated=Math.abs(2-4)-radius>.3;
  ArenaHarness.check(contained && displacedSeparated,"Finite-piece positive and negative independent witnesses");
  return Map.of("pieceBoundingSphereRadiusM",radius,"witnessTimeS",t,"center",List.of(x,2.,z),"wholeFinitePieceContainedInBlockedBox",contained,"displacedBoxSeparatedForEntireFlightByY",displacedSeparated,"xExpandedSweepIntervalS",List.of((3.3-radius-2)/5,(3.7+radius-2)/5),"scope","Independent finite extent containment witness on continuous analytic path, not physical collision resolution or calibrated torus dynamics");
 }
 public static void run()throws Exception{
  ArenaHarness.check(HAL.initialize(500,0),"HAL");SimHooks.pauseTiming();SimulatedArena.overrideSimulationTimings(Seconds.of(.02),5);
  var open=airborne("open",Double.NaN);var blocked=airborne("blocked",2);var displaced=airborne("displaced",4);
  // Timer epochs differ; compare physical traces with floating point tolerance.
  var a=(List<List<Double>>)open.get("trace20ms");double error=0;
  for(var other:List.of(blocked,displaced)){var b=(List<List<Double>>)other.get("trace20ms");for(int i=0;i<a.size();i++)for(int j=0;j<3;j++)error=Math.max(error,Math.abs(a.get(i).get(j)-b.get(i).get(j)));}
  ArenaHarness.check(error<1e-9,"Blocker leaves airborne trajectory unchanged");
  double gOpen=ground(Double.NaN),gBlocked=ground(2),gDisplaced=ground(4); System.out.println("ground max X: "+gOpen+", "+gBlocked+", "+gDisplaced);
  ArenaHarness.check(gOpen>3.7 && gBlocked<3.3 && gDisplaced>3.7,"Same registered XY obstacle blocks ground body only");
  var report=new LinkedHashMap<String,Object>();report.put("schema","maple-projectile-characterization/1");report.put("mapleCommit","1b1e8ea7278de01d40a5ea14dffbaaf13381512a");report.put("gravityMps2",GamePieceProjectile.GRAVITY);report.put("airborne",List.of(open,blocked,displaced));report.put("maxTraceDifferenceM",error);report.put("independentSweptWitness",swept());report.put("groundMaxX",Map.of("open",gOpen,"blocked",gBlocked,"displaced",gDisplaced));report.put("limitations",List.of("Maple airborne path has no collision space and no drag; target callback is proximity logic, not physical shot acceptance.","Authored blocker is an engineering probe, not a scored field target. Its 2D fixture affects ground bodies only; 3D extent is independently evaluated.","No upstream physics changes; analytic gravity11 retained. Initial ground velocity is initialization, not prescribed motion each step."));
  Files.createDirectories(Path.of("artifacts"));new ObjectMapper().writerWithDefaultPrettyPrinter().writeValue(Path.of("artifacts/projectile-characterization.json").toFile(),report);
 }
}

