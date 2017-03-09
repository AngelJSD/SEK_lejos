import lejos.ev3.*;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;

public class GyroSensor {

   private EV3GyroSensor gs;
   private SampleProvider gyroSamples;
   float[] sample= { 0.0f };

   public static void main(String[] args) {
	   GyroSensor test = new GyroSensor();
   }
   
   public GyroSensor(){
	   
	   gs = new EV3GyroSensor(SensorPort.S1);
	   gyroSamples = gs.getAngleMode();
	   gyroSamples.fetchSample(sample, 0);
	   System.out.println(sample[0]);
	   while(true){
		   gyroSamples.fetchSample(sample, 0);
		   System.out.println(sample[0]);
	   }
   }
}
