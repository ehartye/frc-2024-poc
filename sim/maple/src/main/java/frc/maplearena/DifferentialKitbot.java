package frc.maplearena;
import static edu.wpi.first.units.Units.*;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import java.io.IOException;
import java.nio.file.Path;
import org.dyn4j.geometry.Mass;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

/** Original reduced 2D differential force adapter. No pose/speed setters in stepping. */
public final class DifferentialKitbot extends AbstractDriveTrainSimulation {
 public final JsonNode source;
 public final double track,radius,ratio,k,resistance,currentLimit,motors,mass,mu;
 public double leftDuty,rightDuty,forceWorkJ,lastLeftForce,lastRightForce;
 public boolean coast;
 public DifferentialKitbot(Path configPath,Pose2d initial) throws IOException { this(new ObjectMapper().readTree(configPath.toFile()),initial); }
 private DifferentialKitbot(JsonNode c,Pose2d initial) {
  super(DriveTrainSimulationConfig.Default().withRobotMass(Kilograms.of(c.get("massKg").asDouble())).withBumperSize(Meters.of(.891787),Meters.of(.8763)),initial);
  source=c;track=c.get("trackM").asDouble();radius=c.get("wheelRadiusM").asDouble();ratio=c.get("gearRatio").asDouble();k=c.get("motorK").asDouble();resistance=c.get("motorResistanceOhm").asDouble();currentLimit=c.get("phaseCurrentLimitA").asDouble();motors=c.get("motorsPerSide").asDouble();mass=c.get("massKg").asDouble();mu=c.get("tireMu").asDouble();
  setMass(new Mass(new Vector2(),mass,c.get("yawInertiaKgM2").asDouble()));
 }
 public void command(double left,double right,boolean coast) {
  if(!Double.isFinite(left)||!Double.isFinite(right)||Math.abs(left)>1||Math.abs(right)>1)throw new IllegalArgumentException("finite duties in [-1,1]");
  leftDuty=left;rightDuty=right;this.coast=coast;
 }
 static double clamp(double x,double m){return Math.max(-m,Math.min(m,x));}
 @Override public void simulationSubTick(){
  double angle=getTransform().getRotationAngle(),cs=Math.cos(angle),sn=Math.sin(angle);Vector2 forward=new Vector2(cs,sn),lateral=new Vector2(-sn,cs);double dt=SimulatedArena.getSimulationDt().in(Seconds);
  for(int i=0;i<2;i++){
   double side=i==0?1:-1,duty=i==0?leftDuty:rightDuty;Vector2 p=getWorldPoint(new Vector2(0,side*track/2));Vector2 velocity=getLinearVelocity(p);
   double longitudinal=velocity.dot(forward),cross=velocity.dot(lateral),omega=longitudinal/radius*ratio;
   double current=coast?0:clamp((12*duty-k*omega)/resistance,currentLimit);
   double fx=motors*k*current*ratio/radius,fy=-mass*cross/2/.08,cap=mu*mass*9.81/2,mag=Math.hypot(fx,fy);
   if(mag>cap){fx*=cap/mag;fy*=cap/mag;}
   Vector2 force=new Vector2(cs*fx-sn*fy,sn*fx+cs*fy);applyForce(force,p);forceWorkJ+=force.dot(velocity)*dt;
   if(i==0)lastLeftForce=fx;else lastRightForce=fx;
  }
 }
}
