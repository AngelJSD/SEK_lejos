import lejos.ev3.*;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;

public class GyroSensorMovePilot {

   private static EV3GyroSensor gs;
   private static EV3LargeRegulatedMotor mB,mB1;
   private static SampleProvider gyroSamples;
   static float[] sample= { 0.0f };

   public static void main(String[] args) {
	   GyroSensorMovePilot test = new GyroSensorMovePilot();
   }

public static void giro90() {

	   gyroSamples.fetchSample(sample, 0);
	   System.out.println(sample[0]);
	   Wheel wheel1 = WheeledChassis.modelWheel(mB1, 56.0).offset(-80.0);
	   //offset es la distancia del centro a la rueda
	   Wheel wheel2 = WheeledChassis.modelWheel(mB, 56.0).offset(80.0);
	   WheeledChassis chassis = new WheeledChassis(new Wheel[] { wheel1, wheel2 }, WheeledChassis.TYPE_DIFFERENTIAL);
	   MovePilot pilot = new MovePilot(chassis);
	   System.out.println("TestChassis");
	   pilot.rotate(-90);        // degree clockwise
	   gyroSamples.fetchSample(sample, 0);
	   System.out.println(sample[0]);
	   while(sample[0]<90){
		   pilot.rotate(-90 + sample[0]);
		   gyroSamples.fetchSample(sample, 0);
		   System.out.println(sample[0]);
	   }
   }

   public GyroSensorMovePilot(){

	   mB = new EV3LargeRegulatedMotor(MotorPort.B);
	   mB1 = new EV3LargeRegulatedMotor(MotorPort.A);
	   gs = new EV3GyroSensor(SensorPort.S1);
	   gyroSamples = gs.getAngleMode();
	   giro90();
   }
}
