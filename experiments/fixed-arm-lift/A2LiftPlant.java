package org.agentfrc.lesson;
import edu.wpi.first.math.system.plant.DCMotor;
/** Hypothetical retained-piece lift. Ideal level cradle; no slip, linkage compliance or safety claim. */
public final class A2LiftPlant {
 public record Config(double armLength,double armMass,double pieceMass,double ratio,double efficiency,double damping,double supplyVolts){
  public Config {for(double x:new double[]{armLength,armMass,pieceMass,ratio,efficiency,supplyVolts})if(!Double.isFinite(x)||x<=0)throw new IllegalArgumentException("positive finite config");if(efficiency>1||!Double.isFinite(damping)||damping<0)throw new IllegalArgumentException("loss config");}
 }
 public record Sample(double time,double angle,double angularSpeed,double motorWork,double dissipation,double energyResidual,double duty){}
 private static final DCMotor MOTOR=DCMotor.getNEO(1);
 private final Config c; private final double inertia,gravityMoment,initialEnergy;private double[] q;private double time,duty;
 public A2LiftPlant(Config c,double initialAngle){this.c=java.util.Objects.requireNonNull(c);if(!Double.isFinite(initialAngle))throw new IllegalArgumentException("angle");inertia=(c.armMass()/3+c.pieceMass())*c.armLength()*c.armLength();gravityMoment=(c.armMass()/2+c.pieceMass())*9.80665*c.armLength();q=new double[]{initialAngle,0,0,0};initialEnergy=energy(q);}
 public void command(double value){if(!Double.isFinite(value)||Math.abs(value)>.8)throw new IllegalArgumentException("duty");duty=value;}
 private double energy(double[] x){return .5*inertia*x[1]*x[1]+gravityMoment*Math.sin(x[0]);}
 private double[] derivative(double[] x){double emf=x[1]*c.ratio()/MOTOR.KvRadPerSecPerVolt;
 // Explicit hypothetical shorted-motor brake when the requested averaged voltage would regenerate.
 // No battery regeneration is silently accepted; actual controller brake behavior needs validation.
 double bridgeDuty=duty*emf>0&&Math.abs(duty)*c.supplyVolts()<Math.abs(emf)?0:duty;
 var supply=SharedSupply.solve(c.supplyVolts(),.02,new double[]{bridgeDuty},new double[]{emf},new double[]{MOTOR.rOhms});double motorTorque=MOTOR.KtNMPerAmp*supply.armatureAmps()[0]*c.ratio();double friction=c.damping()*x[1];double lossTorque=(1-c.efficiency())*Math.abs(motorTorque)*Math.tanh(x[1]/.001);double applied=motorTorque-lossTorque;return new double[]{x[1],(applied-gravityMoment*Math.cos(x[0])-friction)/inertia,motorTorque*x[1],(lossTorque+friction)*x[1]};}
 private static double[] stage(double[]a,double[]d,double h){double[]r=a.clone();for(int i=0;i<r.length;i++)r[i]+=h*d[i];return r;}
 public Sample step(double dt){if(!Double.isFinite(dt)||dt<=0||dt>.001)throw new IllegalArgumentException("dt");var a=derivative(q);var b=derivative(stage(q,a,dt/2));var d=derivative(stage(q,b,dt/2));var e=derivative(stage(q,d,dt));for(int i=0;i<q.length;i++){q[i]+=dt*(a[i]+2*b[i]+2*d[i]+e[i])/6;if(!Double.isFinite(q[i]))throw new IllegalStateException("nonfinite");}time+=dt;return sample();}
 public Sample sample(){return new Sample(time,q[0],q[1],q[2],q[3],energy(q)+q[3]-q[2]-initialEnergy,duty);}
}
