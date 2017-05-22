package ev3Prueba1;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorMode;

import java.awt.Button;
//import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort; 
import lejos.robotics.SampleProvider;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.utility.Delay;
import lejos.robotics.navigation.DifferentialPilot;

public class test {
//	static EV3LargeRegulatedMotor rightW = new EV3LargeRegulatedMotor(MotorPort.B);
//	static EV3LargeRegulatedMotor letfW = new EV3LargeRegulatedMotor(MotorPort.A);
//	static Wheel leftWheel = WheeledChassis.modelWheel(Motor.A, 42.2).offset(72).gearRatio(2);
//	static Wheel rightWheel = WheeledChassis.modelWheel(Motor.B, 42.2).offset(-72).gearRatio(2);
	//static Chassis myChassis = new WheeledChassis( new Wheel[]{leftWheel, rightWheel}, WheeledChassis.TYPE_DIFFERENTIAL);
//	static MovePilot pilotTest = new MovePilot(myChassis);
	
//	public static void setAcceleratioBoth(int speed)
//	{
//		rightW.setAcceleration(speed);
//		letfW.setAcceleration(speed);
//	}
//	
//	public static void onlyGo()
//	{
//		//rightW.forward();
//		//letfW.forward();
//	}
//	
//	public static void advancedDistance( int distance )
//	{
//	//	pilotTest.travel(distance);
//	}
//	
	//private static EV3LargeRegulatedMotor mb;
	//private static EV3TouchSensor ts;
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		
//		EV3LargeRegulatedMotor mb = new EV3LargeRegulatedMotor(MotorPort.B) ;
//		EV3TouchSensor ts = new EV3TouchSensor(SensorPort.S2);
//		
//		
//		int sp = ts.sampleSize();
//		float [] sample = new float[sp];
//		ts.fetchSample(sample, 0);
		
		DifferentialPilot m_pilot=new DifferentialPilot(1.8 , 4.2 ,Motor.B,Motor.A);
		EV3GyroSensor gs = new EV3GyroSensor(SensorPort.S1);
		SampleProvider gyroSamples;
		float[] sample= { 0.0f };
		
		m_pilot.rotate(90);
		
		
		
		   gyroSamples = gs.getAngleMode();
		   gyroSamples.fetchSample(sample, 0);
		   System.out.println(sample[0]);
		   m_pilot.arcForward(0);

		   while(sample[0]<90){
			   gyroSamples.fetchSample(sample, 0);
			   System.out.println(sample[0]);
		   }
		   m_pilot.stop();
		
	}

}
