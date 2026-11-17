package frc.maplearena;
import static edu.wpi.first.units.Units.Seconds;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import java.nio.file.*;
import java.util.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.crescendo2024.*;

public final class ArenaHarness {
 static final Path CONFIG=Path.of("../drivetrain/config.json");
 static class Arena extends Arena2024Crescendo { int handlers(){return customSimulations.size();} int bodies(){return physicsWorld.getBodyCount();} }
 static void check(boolean ok,String message){if(!ok)throw new AssertionError(message);}
 static Arena fresh(){Arena a=new Arena();SimulatedArena.overrideInstance(a);a.resetFieldForAuto();return a;}
 static DifferentialKitbot robot(Arena a,double x,double y,double deg)throws Exception{var r=new DifferentialKitbot(CONFIG,new Pose2d(x,y,Rotation2d.fromDegrees(deg)));a.addDriveTrainSimulation(r);return r;}
 static void step(Arena a,int count){for(int i=0;i<count;i++){SimHooks.stepTiming(.02);a.simulationPeriodic();}}
 static Map<String,Object> driveCase(String name,double x,double y,double heading,double duty,int ticks)throws Exception{
  Arena a=fresh();a.clearGamePieces();var r=robot(a,x,y,heading);r.command(duty,duty,false);step(a,ticks);var pose=r.getSimulatedDriveTrainPose();var row=new LinkedHashMap<String,Object>();row.put("case",name);row.put("xM",pose.getX());row.put("yM",pose.getY());row.put("yawRad",pose.getRotation().getRadians());row.put("speedMps",r.getLinearVelocity().getMagnitude());row.put("forceWorkJ",r.forceWorkJ);row.put("bodies",a.bodies());a.shutDown();return row;
 }
 public static Map<String,Object> run()throws Exception{
  check(HAL.initialize(500,0),"HAL initialization");SimHooks.pauseTiming();SimulatedArena.overrideSimulationTimings(Seconds.of(.02),5);List<Map<String,Object>> cases=new ArrayList<>();
  var free=driveCase("free_drive",2,2,0,.5,100);check((double)free.get("xM")>3,"force drive accelerates");cases.add(free);
  var wall=driveCase("left_wall",2,2,180,.5,150);check((double)wall.get("xM")>.40 && (double)wall.get("xM")<.60,"actual Maple wall blocks chassis");cases.add(wall);
  var post=driveCase("blue_stage_post",2,4.1,0,.5,150);check((double)post.get("xM")<2.85,"actual Maple stage post blocks chassis");cases.add(post);
  Arena a=fresh();a.clearGamePieces();var left=robot(a,6.4,7,0);var right=robot(a,8.2,7,180);var note=new CrescendoNoteOnField(new Translation2d(7.3,7));a.addGamePiece(note);left.command(.2,.2,false);right.command(.2,.2,false);step(a,60);check(a.gamePiecesOnField().size()==1 && a.gamePiecesOnField().contains(note),"shared NOTE identity conserved");check(left.getSimulatedDriveTrainPose().getX()<right.getSimulatedDriveTrainPose().getX(),"robots do not pass through shared world");cases.add(Map.of("case","shared_note","noteCount",a.gamePiecesOnField().size(),"leftX",left.getSimulatedDriveTrainPose().getX(),"rightX",right.getSimulatedDriveTrainPose().getX(),"noteX",note.getPoseOnField().getX()));a.shutDown();
  Arena first=fresh();int bodies=first.bodies(),handlers=first.handlers();check(first.gamePiecesOnField().size()==11,"upstream eleven initial NOTES");first.addAmpCharge(true);first.addAmpCharge(true);first.activateAmp(true);first.shutDown();Arena reset=fresh();check(reset.gamePiecesOnField().size()==11 && reset.handlers()==handlers && reset.bodies()==bodies && !reset.isAmped(true) && reset.getScore(true)==0,"fresh-world reset clears state without duplicate handlers");reset.shutDown();cases.add(Map.of("case","fresh_world_reset","notes",11,"handlers",handlers,"bodies",bodies));
  Random seeded1=new Random(1780),seeded2=new Random(1780);double start1=1+seeded1.nextDouble(),start2=1+seeded2.nextDouble();var replay1=driveCase("seeded",start1,2,0,.2,20);var replay2=driveCase("seeded",start2,2,0,.2,20);check(replay1.equals(replay2),"seeded adapter trial exact replay");cases.add(replay1);
  var report=new LinkedHashMap<String,Object>();report.put("schema","maple-2024-arena-harness/1");report.put("mapleCommit","1b1e8ea7278de01d40a5ea14dffbaaf13381512a");report.put("cases",cases);report.put("limitations",List.of("Actual upstream Arena2024Crescendo 2D field, not official full CAD geometry.","Original differential force adapter: fixed12V, ideal rolling-derived motor speed, no wheel rotor/slip state; hypothetical mass/friction/yaw inertia, no mechanism calibration.","Scoring not accepted: pinned upstream amp/timer/reset defects are documented separately. Fresh-world reset used; upstream reset not qualified.","Seed covers original scenario generator only; upstream private RNG not seeded, no randomized gyro used.","No GUI/driver callback/3D mechanism simulation."));return report;
 }
 public static void main(String[]args)throws Exception{var r=run();Path out=Path.of("artifacts/arena-headless.json");Files.createDirectories(out.getParent());new ObjectMapper().writerWithDefaultPrettyPrinter().writeValue(out.toFile(),r);System.out.println(new ObjectMapper().writeValueAsString(r));}
}
