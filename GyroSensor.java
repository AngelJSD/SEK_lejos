package Gyro;

import lejos.ev3.*;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;

public class GyroSensor {

   private static EV3GyroSensor gs;
   private static EV3LargeRegulatedMotor mB,mB1;
   private static SampleProvider gyroSamples;
   static float[] sample= { 0.0f };

   public static void main(String[] args) {
	   GyroSensor test = new GyroSensor();
   }
   
   public static void giro90() {
	   gyroSamples = gs.getAngleMode();
	   gyroSamples.fetchSample(sample, 0);
	   System.out.println(sample[0]);
	   mB.backward();
	   mB1.forward();
	   while(sample[0]<90){
		   gyroSamples.fetchSample(sample, 0);
		   System.out.println(sample[0]);
	   }
	   mB.stop(true);
	   mB1.stop(true);
   }
   
   public GyroSensor(){
	   
	   mB = new EV3LargeRegulatedMotor(MotorPort.B);
	   mB1 = new EV3LargeRegulatedMotor(MotorPort.A);
	   gs = new EV3GyroSensor(SensorPort.S1);
	   giro90();
   }
}