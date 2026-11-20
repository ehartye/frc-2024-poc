package org.agentfrc.lesson;
import java.util.*;
import java.nio.file.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import static org.agentfrc.lesson.A2MissionGeometry.*;
/** Full mission orchestration with ideal drive and prescribed retained-piece kinematics. */
public final class A2MissionSimulation {
 public enum Scenario { NORMAL, NORMAL_9V, MISSING_NOTE, JAM, STALE_POSE, STRICT_9V, STRICT_12V }
 public record Exit(double time,double x,double y,double z,double vx,double vy,double vz,double speed,double rpm,double energyResidual){}
 public record Result(Scenario scenario,CrescendoMissionSupervisor.State state,CrescendoMissionSupervisor.Fault missionFault,A2ShooterAdapter.Fault shooterFault,double time,int captures,int releases,double maxEnergyResidual,List<Exit> exits,List<String> trace,PoweredGuidePlant.Ledger ledger){}
 public static final String HEADER="timeS,state,robotX,robotY,robotHeadingRad,intakeAngleDeg,intakeLengthM,piecePhase,pieceXRobot,pieceYRobot,pieceZRobot,pieceXWorld,pieceYWorld,pieceZWorld,possession,indexed,stowed,shooterRpm,shotCount,fault,event,exitVx,exitVy,exitVz,bodyPitchRad,bodySpinRadPerS,poseAgeS,energyResidualJ,outgoingBodyEnergyJ";
 private static final double DT=.0001,PICKUP_X=3,PICKUP_Y=0,ARM_RATE=90,TRANSFER_SPEED=.4,SEATING_TIME=.15;
 private final Scenario scenario;private final boolean strict;private final A2ShooterAdapter shooter;private final CrescendoMissionSupervisor mission;
 private Pose2d pose=new Pose2d(2.4,.1,new Rotation2d(Math.PI+.1));private final DifferentialDriveKinematics drive=new DifferentialDriveKinematics(TRACK);
 private CrescendoMissionSupervisor.Output command;private double now,nextControl,lastStamp,angle=STOW_DEG,linear,angular,seat=-1,transfer,maxResidual,poseAge;
 private boolean held=true,floorAvailable=true,exitObserved,releaseLatched;private int captures;private long sequence;
 private double[] piece={SHOOTER_X,0,SHOOTER_Z};private String phase="PRELOADED";
 private final List<Exit> exits=new ArrayList<>();private final List<String> trace=new ArrayList<>();
 public A2MissionSimulation(Scenario scenario){
  this.scenario=Objects.requireNonNull(scenario);strict=scenario==Scenario.STRICT_9V||scenario==Scenario.STRICT_12V;
  double volts=scenario==Scenario.NORMAL_9V||scenario==Scenario.STRICT_9V?9:12,ready=strict?2500:1800,floor=strict?2500:1000;
  var plant=new PoweredGuidePlant.Config(.235,.0508,.0003,GUIDE_LENGTH,Math.toRadians(SHOT_PITCH_DEG),25,.8,.8,volts);
  shooter=new A2ShooterAdapter(new A2ShooterAdapter.Config(plant,ready,floor,.05,1,.5),true);
  mission=new CrescendoMissionSupervisor(new CrescendoMissionSupervisor.Config(.05,.1,20,4,6,3,1,.5,.2,.05,ready,floor,.35,.15,.2));
  floorAvailable=scenario!=Scenario.MISSING_NOTE;trace.add(HEADER);
 }
 private double length(){double c=Math.cos(Math.toRadians(angle)),s=Math.sin(Math.toRadians(angle)),extra=Math.max(Math.max(CAPTURE_RADIUS,RETENTION_TOP),Math.max(SHAFT_RADIUS,Math.max(Math.abs(c)*ARM_HALF,Math.max(Math.abs(c)*SLEEVE_HALF,Math.max(c*LINK_TOP,c*LINK_OFFSET)))));double value=ARM_MAX;if(c>1e-12)value=Math.min(value,(FORWARD_CENTER-PIVOT_X)/c);if(s>1e-12)value=Math.min(value,(MAXIMUM_HEIGHT-HEIGHT_CLEARANCE-PIVOT_Z-extra)/s);if(value<ARM_MIN-1e-9)throw new IllegalStateException("telescope travel");return value;}
 private double[] cradle(){double a=Math.toRadians(angle);return new double[]{PIVOT_X+length()*Math.cos(a),0,PIVOT_Z+length()*Math.sin(a)};}
 private double[] world(double[] p){double h=pose.getRotation().getRadians();return new double[]{pose.getX()+p[0]*Math.cos(h)-p[1]*Math.sin(h),pose.getY()+p[0]*Math.sin(h)+p[1]*Math.cos(h),p[2]};}
 private double[] local(double[] p){double h=pose.getRotation().getRadians(),x=p[0]-pose.getX(),y=p[1]-pose.getY();return new double[]{x*Math.cos(h)+y*Math.sin(h),-x*Math.sin(h)+y*Math.cos(h),p[2]};}
 private static double distance(double[] a,double[] b){return Math.sqrt(Math.pow(a[0]-b[0],2)+Math.pow(a[1]-b[1],2)+Math.pow(a[2]-b[2],2));}
 private double[] floorWorld(){return new double[]{PICKUP_X-FLOOR_PIECE_LOCAL[0],PICKUP_Y,FLOOR_PIECE_LOCAL[2]};}
 private boolean indexed(){return held&&distance(piece,new double[]{SHOOTER_X,0,SHOOTER_Z})<1e-9;}
 private boolean stowed(){return Math.abs(angle-STOW_DEG)<1e-8;}
 private boolean stopped(){return Math.abs(linear)<1e-9&&Math.abs(angular)<1e-9;}
 private double headingError(){return new Rotation2d(Math.PI).minus(pose.getRotation()).getRadians();}
 private boolean at(double x,double y){return Math.hypot(pose.getX()-x,pose.getY()-y)<.005&&Math.abs(headingError())<.005;}
 private boolean jam(){return scenario==Scenario.JAM&&command!=null&&(command.state()==CrescendoMissionSupervisor.State.ACQUIRE||command.state()==CrescendoMissionSupervisor.State.INDEX);}
 private void observe(){
  lastStamp=scenario==Scenario.STALE_POSE?Math.min(now,.5):now;poseAge=now-lastStamp;
  var goal=at(SHOT_X,SHOT_Y)?CrescendoMissionSupervisor.Goal.SHOT_POSE:at(PICKUP_X,PICKUP_Y)?CrescendoMissionSupervisor.Goal.FLOOR_NOTE:CrescendoMissionSupervisor.Goal.NONE;
  var o=new CrescendoMissionSupervisor.Observation(sequence++,lastStamp,held?CrescendoMissionSupervisor.Piece.PRESENT:CrescendoMissionSupervisor.Piece.ABSENT,indexed(),stowed(),goal,Math.abs(headingError())<.005,stopped(),floorAvailable&&Math.hypot(pose.getX()-PICKUP_X,pose.getY()-PICKUP_Y)<.6,jam(),shooter.sample().actualRpm(),exitObserved);
  command=mission.update(now,o,sequence==1,false);if(command.state()!=CrescendoMissionSupervisor.State.RELEASE)releaseLatched=false;
 }
 private void moveDrive(double dt){
  linear=angular=0;if(command.driveGoal()!=CrescendoMissionSupervisor.Goal.NONE){double gx=command.driveGoal()==CrescendoMissionSupervisor.Goal.SHOT_POSE?SHOT_X:PICKUP_X,gy=0,dx=gx-pose.getX(),dy=gy-pose.getY(),d=Math.hypot(dx,dy),e=new Rotation2d(Math.atan2(dy,dx)).minus(pose.getRotation()).getRadians();
   linear=d>.005?Math.min(.8,1.5*d)*Math.max(0,Math.cos(e)):0;angular=Math.max(-1.2,Math.min(1.2,2.5*(d>.005?e:headingError())));if(at(gx,gy)){linear=0;angular=0;}
  }else if(command.alignToShot())angular=Math.abs(headingError())<.005?0:Math.max(-1.2,Math.min(1.2,2.5*headingError()));
  var wheels=drive.toWheelSpeeds(new ChassisSpeeds(linear,0,angular));wheels.desaturate(1);var actual=drive.toChassisSpeeds(wheels);linear=actual.vxMetersPerSecond;angular=actual.omegaRadiansPerSecond;pose=pose.exp(new Twist2d(linear*dt,0,angular*dt));
 }
 private static double[] onTransfer(double d){for(int i=1;i<TRANSFER.length;i++){double len=distance(TRANSFER[i-1],TRANSFER[i]);if(d<len){double f=d/len;return new double[]{TRANSFER[i-1][0]+f*(TRANSFER[i][0]-TRANSFER[i-1][0]),0,TRANSFER[i-1][2]+f*(TRANSFER[i][2]-TRANSFER[i-1][2])};}d-=len;}return TRANSFER[TRANSFER.length-1].clone();}
 private void movePiece(double dt){
  if(!held&&floorAvailable&&phase.equals("FLOOR"))piece=local(floorWorld());
  if(command.state()==CrescendoMissionSupervisor.State.ACQUIRE&&!jam()){
   angle=Math.max(FLOOR_DEG,angle-ARM_RATE*dt);
   if(!held&&floorAvailable&&Math.abs(angle-FLOOR_DEG)<1e-8&&distance(world(cradle()),floorWorld())<.06){
    if(seat<0)seat=0;seat=Math.min(SEATING_TIME,seat+dt);double f=seat/SEATING_TIME;double[] start=local(floorWorld()),end=cradle();piece=new double[]{start[0]+f*(end[0]-start[0]),start[1]+f*(end[1]-start[1]),start[2]+f*(end[2]-start[2])};phase="SEATING_PRESCRIBED";
    if(distance(piece,end)<1e-9){held=true;floorAvailable=false;captures++;exitObserved=false;phase="RETAINED_IDEAL";}
   }
  }else if(command.state()==CrescendoMissionSupervisor.State.INDEX&&held&&!jam()){
   if(angle<STOW_DEG){angle=Math.min(STOW_DEG,angle+ARM_RATE*dt);piece=cradle();phase="LIFT_IDEAL";}
   else {transfer+=TRANSFER_SPEED*dt;piece=onTransfer(transfer);phase=indexed()?"INDEXED_IDEAL":"TRANSFER_PRESCRIBED";}
  }
 }
 private void row(String event,Exit e){
  double[] p=world(piece);var s=shooter.sample();String failure=command.fault().name();if(s.fault()!=A2ShooterAdapter.Fault.NONE)failure="SHOOTER_"+s.fault();
  String[] values={Double.toString(now),command.state().name(),Double.toString(pose.getX()),Double.toString(pose.getY()),Double.toString(pose.getRotation().getRadians()),Double.toString(angle),Double.toString(length()),phase,Double.toString(piece[0]),Double.toString(piece[1]),Double.toString(piece[2]),Double.toString(p[0]),Double.toString(p[1]),Double.toString(p[2]),Boolean.toString(held),Boolean.toString(indexed()),Boolean.toString(stowed()),Double.toString(s.actualRpm()),Integer.toString(exits.size()),failure,event,e==null?"":Double.toString(e.vx()),e==null?"":Double.toString(e.vy()),e==null?"":Double.toString(e.vz()),e==null?"":Double.toString(Math.toRadians(SHOT_PITCH_DEG)),"",Double.toString(poseAge),Double.toString(s.sample().energyResidual()),Double.toString(shooter.ledger().outgoingBodyEnergy())};trace.add(String.join(",",values));
 }
 public Result run(){
  observe();row("START",null);nextControl=.01;
  while(now<60&&command.state()!=CrescendoMissionSupervisor.State.DONE&&command.state()!=CrescendoMissionSupervisor.State.FAULT&&shooter.sample().state()!=A2ShooterAdapter.State.FAULT){
   boolean enable=command.shooterTargetRpm()>0,release=command.releaseDuty()>0&&!releaseLatched;
   double before=shooter.sample().sample().time();var result=shooter.update(DT,enable,enable?.8:0,release);double dt=result.sample().time()-before;
   if(release)releaseLatched=true;moveDrive(dt);movePiece(dt);now=result.sample().time();maxResidual=Math.max(maxResidual,Math.abs(result.sample().energyResidual()));
   if(result.state()==A2ShooterAdapter.State.FEEDING){piece=new double[]{SHOOTER_X+result.sample().x()*Math.cos(Math.toRadians(SHOT_PITCH_DEG)),0,SHOOTER_Z+result.sample().x()*Math.sin(Math.toRadians(SHOT_PITCH_DEG))};phase="GUIDE_PHYSICS";}
   if(result.state()==A2ShooterAdapter.State.EXITED){
    piece=new double[]{SHOOTER_X+GUIDE_LENGTH*Math.cos(Math.toRadians(SHOT_PITCH_DEG)),0,SHOOTER_Z+GUIDE_LENGTH*Math.sin(Math.toRadians(SHOT_PITCH_DEG))};double[] p=world(piece);double speed=result.sample().speed(),pitch=Math.toRadians(SHOT_PITCH_DEG),h=pose.getRotation().getRadians();var e=new Exit(now,p[0],p[1],p[2],speed*Math.cos(pitch)*Math.cos(h),speed*Math.cos(pitch)*Math.sin(h),speed*Math.sin(pitch),speed,result.actualRpm(),result.sample().energyResidual());exits.add(e);held=false;exitObserved=true;phase="EXIT_OBSERVED";row("EXIT",e);shooter.departBody();phase=floorAvailable?"FLOOR":scenario==Scenario.MISSING_NOTE?"MISSING":"EMPTY";
   }
   if(now+1e-12>=nextControl){observe();row("SAMPLE",null);nextControl+=.01;}
   if(dt==0&&result.state()!=A2ShooterAdapter.State.FAULT)throw new IllegalStateException("No clock advancement");
  }
  if(shooter.sample().state()==A2ShooterAdapter.State.FAULT&&command.state()!=CrescendoMissionSupervisor.State.FAULT&&now>lastStamp)observe();
  row("TERMINAL",null);return new Result(scenario,command.state(),command.fault(),shooter.sample().fault(),now,captures,exits.size(),maxResidual,List.copyOf(exits),List.copyOf(trace),shooter.ledger());
 }
 public static void write(Result r,Path dir)throws Exception{
  Files.createDirectories(dir);Files.write(dir.resolve(r.scenario()+".csv"),r.trace(),java.nio.charset.StandardCharsets.UTF_8,StandardOpenOption.CREATE_NEW);
  String json="{\"schema\":\"frc-a2-mission/1\",\"scenario\":\""+r.scenario()+"\",\"state\":\""+r.state()+"\",\"missionFault\":\""+r.missionFault()+"\",\"shooterFault\":\""+r.shooterFault()+"\",\"timeS\":"+r.time()+",\"captures\":"+r.captures()+",\"releases\":"+r.releases()+",\"maxEnergyResidualJ\":"+r.maxEnergyResidual()+",\"outgoingBodyEnergyJ\":"+r.ledger().outgoingBodyEnergy()+",\"contractSha256\":\""+CONTRACT_SHA256+"\",\"integrationStepS\":0.0001,\"controlPeriodS\":0.01,\"armSpeedDegPerS\":90,\"transferSpeedMps\":0.4,\"seatingDurationS\":0.15,\"energyLedgerScope\":\"Powered rollers and guide only; excludes drive and prescribed lift/transfer energy\",\"physicalCaptureValidated\":false,\"officialScores\":null,\"assumptions\":[\"Ideal differential-drive kinematics\",\"Prescribed proximity-triggered seating; no physical floor capture\",\"Coordinated telescope and level retained cradle\",\"Prescribed transfer polyline and guide-entry attitude\",\"Virtual sensors and provisional target registration\",\"Fixed-duty powered shooter with open-circuit no-drag coast\"]}\n";
  Files.writeString(dir.resolve(r.scenario()+".json"),json,StandardOpenOption.CREATE_NEW);
 }
 public static void main(String[] args)throws Exception{if(args.length!=2)throw new IllegalArgumentException("scenario outputDirectory");write(new A2MissionSimulation(Scenario.valueOf(args[0])).run(),Path.of(args[1]));}
}
