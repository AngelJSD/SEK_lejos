package testEV32016;
import lejos.hardware.BrickFinder;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.utility.Delay;

public class tesetBrick {
 
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		GraphicsLCD g = BrickFinder.getDefault().getGraphicsLCD();
		g.drawString("Holi Boli Colibroly", 0, 0,GraphicsLCD.VCENTER | GraphicsLCD.LEFT);
		Delay.msDelay(5000);
	}

}
