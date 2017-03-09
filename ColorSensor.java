import lejos.ev3.*;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.Color;

public class ColorSensor {

   private EV3ColorSensor cs;
   private SampleProvider colorSamples;
   float[] sample= { 0.0f };
   String colorName = "";

   public static void main(String[] args) {
	   ColorSensor test = new ColorSensor();
   }
   
   public ColorSensor(){
	   
	   cs = new EV3ColorSensor(SensorPort.S1);
	   colorSamples = cs.getColorIDMode();
	   colorSamples.fetchSample(sample, 0);
	   System.out.println(sample[0]);
	   while(true){
		   colorSamples.fetchSample(sample, 0);
		   switch((int) sample[0]){
				case Color.NONE: colorName = "NONE"; break;
				case Color.BLACK: colorName = "BLACK"; break;
				case Color.BLUE: colorName = "BLUE"; break;
				case Color.GREEN: colorName = "GREEN"; break;
				case Color.YELLOW: colorName = "YELLOW"; break;
				case Color.RED: colorName = "RED"; break;
				case Color.WHITE: colorName = "WHITE"; break;
				case Color.BROWN: colorName = "BROWN"; break;
		   }
		   System.out.println(colorName);
	   }
   }
}
