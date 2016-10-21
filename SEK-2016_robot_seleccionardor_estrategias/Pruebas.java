package testSEK;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.utility.Delay;
import lejos.hardware.sensor.NXTLightSensor;

public class Pruebas {
	static EV3ColorSensor colorSensorHuman = new EV3ColorSensor(SensorPort.S3);
	static EV3UltrasonicSensor ultraSonic = new EV3UltrasonicSensor(SensorPort.S1);
	static NXTLightSensor colorSensorHumanOwer = new NXTLightSensor(SensorPort.S4);
	static EV3MediumRegulatedMotor claw = new EV3MediumRegulatedMotor(MotorPort.B);
	
	static EV3LargeRegulatedMotor rightW = new EV3LargeRegulatedMotor(MotorPort.D);
	static EV3LargeRegulatedMotor letfW = new EV3LargeRegulatedMotor(MotorPort.A);
	static DifferentialPilot Mypilot = new DifferentialPilot(56, 128, rightW, letfW, true);
	static OdometryPoseProvider Odometro = new OdometryPoseProvider(Mypilot);	
	static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
	
	static int coloPisoVerde = 1;
	static int coloPisoAzul = 2;
	static int colorHumanBlanco = 6;
	static int colorHumanNegro = 7;
	static int coloSintaNegra = 7;
	
	public static void carGo(int distanciaCm)
	{
		Mypilot.travel(-10*distanciaCm);
	}
	
	public static void setStop()
	{
		rightW.stop();
		letfW.stop();
		Mypilot.stop();
	}
	public static void setAcceleratioBoth(int speed)
	{
		rightW.setAcceleration(speed);
		letfW.setAcceleration(speed);
		Mypilot.setAngularSpeed(speed);
	}
	
	public static boolean differentColor(int color)
	{
		if(colorSensor.getColorID()==color)
			return false;
		else
			return true;
	}
	
	public static boolean differentColorHuman(int color)
	{
		if(colorSensorHuman.getColorID()==color)
			return false;
		else
			return true;
	}
	
	public static void getColor()
	{
		LCD.drawString("Color- "+colorSensor.getColorID(), 0, 0);
		Delay.msDelay(500); 
		LCD.clear();
	}
	
	
	
	public static void get_colorID()
	{
		LCD.drawString("Capturando Color..", 0, 0);
		LCD.drawString("Color es..."+colorSensorHuman.getColorID(), 0, 3);
		LCD.clear();
	}
	
	
	
	
	
	public static int getColorHumanACoger()
	{
		int colorPiso =colorSensor.getColorID();
		int colorHuman=-1;
		
		if (colorPiso == coloPisoAzul){			
			colorHuman = colorHumanBlanco;}
		
		if (colorPiso==coloPisoVerde){
			colorHuman = colorHumanNegro;}
		
		return colorHuman;
	}
	
	
	
	public static void take()
	{
		LCD.drawString("Atrapando...", 0, 0);
		claw.rotate(-20);
		LCD.clear();
	}
	
	
	public static void pickUpColorPisoVerde()
	{
		if (getColorHumanACoger()==colorHumanNegro)
			pickUp();
	}
	
	public static void pickUpColorPisoAzul()
	{
		if (getColorHumanACoger()==colorHumanBlanco)
			pickUp();
	}
/*	
	public static void pickUpIf(int color)
	{
		LCD.drawString("Recogiendo...", 0, 0);
		if(colorSensorHuman.getColorIDMode() == color )
			take();
			claw.rotateTo(4350);
		LCD.clear();
	}*/
	
	public static void clawStar()
	{
		LCD.drawString("Recogiendo...", 0, 0);
		take();
		claw.rotateTo(-4450);
		LCD.clear();
	}
	
	public static void clockwise()
	{
		rightW.backward();
		letfW.forward();
	}
	
	public static void counterClockwise()
	{
		rightW.forward();
		letfW.backward();
	}
	/**
	 * @param args
	 */
	public static void advanceToDiffDistant(float distance, int speed )
	{
		LCD.drawString("Avanzando Hatas distancia minima...", 0, 0);
		Mypilot.backward();
		while (get_distance()>=distance){
				//rightW.forward();
				//letfW.forward();
			setAcceleratioBoth(speed);
			if(Button.ENTER.isDown())
				System.exit(0);
			
		}
		Mypilot.stop();
		//rightW.stop();
		//letfW.stop();
		LCD.clear();
	}
	
	public static void advanceToDiffDistant(float distance )
	{
		LCD.drawString("Avanzando Hatas distancia minima...", 0, 0);
		Mypilot.backward();
		while (get_distance()>=distance){
				//rightW.forward();
				//letfW.forward();
			if(Button.ENTER.isDown())
				System.exit(0);
			
		}
		Mypilot.stop();
		//rightW.stop();
		//letfW.stop();
		LCD.clear();
	}
	
	public static void moveToDiffColor(int color, int speed, boolean sentido)
	{
		LCD.drawString("Avanzando Hatas no ser...", 0, 0);
		
		if (sentido == true)
			Mypilot.backward();
		else 
			Mypilot.forward();
		
		while (differentColor(color) == true&&colorSensor.getColorID()!=-1){
				//rightW.forward();
				//letfW.forward();
			setAcceleratioBoth(speed);
			if(Button.ENTER.isDown())
				System.exit(0);
		}
		//rightW.stop();
		//letfW.stop();
		Mypilot.stop();
		LCD.clear();
	}
	
	public static void moveToDiffColor(int color, boolean sentido)
	{
		LCD.drawString("Avanzando Hatas no ser...", 0, 0);
		if (sentido == true)
			Mypilot.backward();
		else 
			Mypilot.forward();
		while (differentColor(color) == true&&colorSensor.getColorID()!=-1){
				//rightW.forward();
				//letfW.forward();
			
			if(Button.ENTER.isDown())
				System.exit(0);
		}
		//rightW.stop();
		//letfW.stop();
		Mypilot.stop();
		LCD.clear();
	}
	
	public static void advanceToDiffColorHuman(int color, int speed)
	{
		LCD.drawString("Avanzando Hatas no ser...", 0, 0);
		Mypilot.backward();
		while (colorSensorHuman.getColorID()!=color&&colorSensorHuman.getColorID()==-1){
				//rightW.forward();
				//letfW.forward();
			setAcceleratioBoth(speed);
			if(Button.ENTER.isDown())
				System.exit(0);
		}
		//rightW.stop();
		//letfW.stop();
		Mypilot.stop();
		LCD.clear();
	}
	
	public static void advanceToEqualColor(int color, int speed)
	{
		LCD.drawString("Avanzando Hatas no ser...", 0, 0);
		Mypilot.backward();
		while (colorSensorHuman.getColorID()==color){
				//rightW.forward();
				//letfW.forward();
			//setAcceleratioBoth(speed);
			if(Button.ENTER.isDown())
				System.exit(0);
		}
		/*
		while (get_distance()>=5)
		{	
			if(Button.ENTER.isDown())
				System.exit(0);

		}*/
		//rightW.stop();
		//letfW.stop();
		Mypilot.stop();
		LCD.clear();
	}
	
	public static void advanceToDiffColorHuman(int color)
	{
		LCD.drawString("Avanzando Hatas no ser...", 0, 0);
		Mypilot.backward();
		while (colorSensorHuman.getColorID()!=color&&colorSensorHuman.getColorID()==-1){
				//rightW.forward();
				//letfW.forward();
			
			if(Button.ENTER.isDown())
				System.exit(0);
		}
		//rightW.stop();
		//letfW.stop();
		Mypilot.stop();
		LCD.clear();
	}
	

	
	public static void goStartPosition()
	{
		Odometro.moveStarted(null, null);
	}
	
	public static void turnWhithPilot(int angule)
	{
		Mypilot.rotate(angule);
	}
	
	public static void pickUpCondicional(int color)
	{
		setAcceleratioBoth(50);
		advanceToDiffDistant(5,50);
		setStop();
		if(colorSensorHuman.getColorID() == color) {

			claw.setSpeed(1000);
			pickUp();
		}
	}
	
	static void showColor()
	{
	LCD.drawString("Vict Color..", 0, 0);
	LCD.drawString("Color es..."+get_colorAmbiental(), 0, 3);
	Delay.msDelay(500);
	LCD.clear();
	}
	public static float get_distance(){
		LCD.drawString("Capturando ....", 0, 0);
		SampleProvider spSonic = ultraSonic.getDistanceMode();
		float[] sampleIzq = new float[spSonic.sampleSize()];
		spSonic.fetchSample(sampleIzq, 0);
		LCD.drawString("CM"+sampleIzq[0]*100, 0, 4);
		LCD.drawString(".."+sampleIzq[0]*100, 0, 3);
		
		return sampleIzq[0]*100;
	}	
	
	public static float get_colorAmbiental(){
		LCD.drawString("Capturando Pokemon....", 0, 0);
		SampleProvider spSonic = colorSensorHuman.getAmbientMode();
		float[] sampleIzq = new float[spSonic.sampleSize()];
		spSonic.fetchSample(sampleIzq, 0);
		LCD.drawString(".."+sampleIzq[0], 0, 3);
		
		return sampleIzq[0]*100;
	}
	
	
	public static float getNxt_colorAmbiental(){
		LCD.drawString("color S4..", 0, 0);
		SampleProvider spSonic = colorSensorHumanOwer.getAmbientMode();
		float[] sampleIzq = new float[spSonic.sampleSize()];
		spSonic.fetchSample(sampleIzq, 0);
		LCD.drawString(" "+sampleIzq[0], 0, 2);
		
		return sampleIzq[0]*100;
	}
	
	public static void getColorHuman()
	{
		LCD.drawString("Color S3 "+colorSensorHuman.getColorID(), 0, 3);
		Delay.msDelay(700); 
		LCD.clear();
	}
	
	public static void pickUp()
	{
		claw.setSpeed(1000);
		LCD.drawString("Recogiendo...", 0, 0);
		take();
		claw.rotateTo(4550);
		LCD.clear();
	}
public static void main(String[] args) {
	// TODO Auto-generated method stub
/*	getNxt_colorAmbiental();
	getColorHuman();
	
	while(colorSensorHuman.isFloodlightOn()==false)
		Delay.msDelay(1);
	pickUp();*/
	/*while (get_distance()>)
	Mypilot.rotate(360);;
*/
}
}