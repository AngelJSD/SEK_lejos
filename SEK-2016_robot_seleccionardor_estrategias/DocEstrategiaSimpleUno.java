package testSEK;

//import sun.security.mscapi.KeyStore.MY;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.utility.Delay;

public class DocEstrategiaSimpleUno {
	
	static EV3LargeRegulatedMotor rightW = new EV3LargeRegulatedMotor(MotorPort.D);
	static EV3LargeRegulatedMotor letfW = new EV3LargeRegulatedMotor(MotorPort.A);
	static DifferentialPilot Mypilot = new DifferentialPilot(56, 128, rightW, letfW, true);
	static OdometryPoseProvider Odometro = new OdometryPoseProvider(Mypilot);
	
	
	static EV3UltrasonicSensor ultraSonic = new EV3UltrasonicSensor(SensorPort.S4);
	static EV3LargeRegulatedMotor claw = new EV3LargeRegulatedMotor(MotorPort.B);
	static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
	static EV3ColorSensor colorSensorHuman = new EV3ColorSensor(SensorPort.S1);
	static EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S3);
	static int coloPisoVerde = 1;
	static int coloPisoAzul = 2;
	static int colorHumanBlanco = 6;
	static int colorHumanNegro = 7;
	static int coloSintaNegra = 7;
	
	
	static float[] gyroDisn = new float[2];
	/*static Wheel leftWheel = WheeledChassis.modelWheel(Motor.A, 13.4).offset(72).gearRatio(2);
	static Wheel rightWheel = WheeledChassis.modelWheel(Motor.D, 13.4).offset(-72).gearRatio(2);
	static Chassis myChassis = new WheeledChassis (new Wheel[]{leftWheel,rightWheel}, WheeledChassis.TYPE_DIFFERENTIAL);*/
	
	/*

	//Girara en sentido horario o antihoracio
	public static void turnLoR( boolean sense )
	{
		if(sense == true)
			clockwise();
		else
			counterClockwise();
	}
	
	// usar el color blanco

/*	
	
	*/
	
	
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
	
	public static void getColorHuman()
	{
		LCD.drawString("Color- "+colorSensorHuman.getColorID(), 0, 0);
		Delay.msDelay(700); 
		LCD.clear();
	}
	
	public static void get_colorID()
	{
		LCD.drawString("Capturando Color..", 0, 0);
		LCD.drawString("Color es..."+colorSensorHuman.getColorID(), 0, 3);
		LCD.clear();
	}
	
	public static float get_distance(){
		LCD.drawString("Capturando Pokemon....", 0, 0);
		SampleProvider spSonic = ultraSonic.getDistanceMode();
		float[] sampleIzq = new float[spSonic.sampleSize()];
		spSonic.fetchSample(sampleIzq, 0);
		LCD.drawString("te la creite XD(Distancia).."+sampleIzq[0]*100, 0, 2);
		LCD.drawString(" "+sampleIzq[0]*100, 0, 3);
		
		return sampleIzq[0]*100;
	}	
	
	public static float get_Gyros(){
		LCD.drawString("Capturando Pokemon....", 0, 0);
		SampleProvider spSonic = gyroSensor.getAngleMode();
		float[] sampleIzq = new float[spSonic.sampleSize()];
		spSonic.fetchSample(sampleIzq, 0);
		LCD.drawString("te la creite XD(Distancia).."+sampleIzq[0]*100, 0, 2);
		LCD.drawString(".."+sampleIzq[0]*100, 0, 3);
		
		return sampleIzq[0]*100;
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
	
	public static void getGyroDistans()
	{
		gyroDisn[0] = get_distance();
		gyroDisn[1] = get_Gyros();	
	}
	
	public static void take()
	{
		LCD.drawString("Atrapando...", 0, 0);
		claw.rotate(-20);
		LCD.clear();
	}
	
	public static void pickUp()
	{
		claw.setSpeed(1000);
		LCD.drawString("Recogiendo...", 0, 0);
		take();
		claw.rotateTo(5000);
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
		claw.setSpeed(1000);
		claw.rotateTo(-5000);
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
	
	public static float get_ColorRedMode(){
		LCD.drawString("Color Red", 0, 0);
		SampleProvider spSonic = colorSensorHuman.getRedMode();
		float[] sampleIzq = new float[spSonic.sampleSize()];
		spSonic.fetchSample(sampleIzq, 0);
		LCD.drawString(".."+sampleIzq[0]*100, 0, 2);
		LCD.drawString("...."+sampleIzq[0], 0, 3);
		
		return sampleIzq[0]*100;
	}
	/*
	public static boolean moveToDiffColor(int color, boolean sentido)
	{
		boolean colorHuman;
		float iq_colorTest;
		float iq_color = (humanoBlanco+humanoNegro)/2;
		iq_colorTest = get_ColorRedMode();
		if( iq_colorTest < iq_color){
			LCD.drawString("Es Negro", 0, 5);
			colorHuman = true;}
		else{
			LCD.drawString("Es Blanco", 0, 5);
			colorHuman = false;
		if(Button.ENTER.isDown())
			System.exit(0);
		return colorHuman;	
		}		
			
	}*/
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
		//rightW.stop();
		//letfW.stop();
	//public static 
	
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
	
	static void StrategyaUno(int colorAtrapar)
	{ 
		moveToDiffColor(7,true);
	/*	while (get_distance()>30){
			turnWhithPilot(-10);
		}
		
		turnWhithPilot(-10);
		*/
		advanceToDiffDistant(10);
//		setAcceleratioBoth(350);
		
		if (getColorHumanACoger() == colorHumanBlanco){
			advanceToEqualColor(colorHumanNegro, 500);
			pickUpColorPisoAzul();
		}
		
		if (getColorHumanACoger() == colorHumanNegro){
			advanceToDiffDistant(5, 500);
			pickUpColorPisoVerde();
		}


	}
	
	static void strategyaUnoSimple( int distanciaParaAtrapar)
	{
		claw.setSpeed(50);
		moveToDiffColor(coloSintaNegra,true);
		advanceToDiffDistant(distanciaParaAtrapar);
		//pickUp();
		claw.rotate(200);
		moveToDiffColor(coloSintaNegra, false);
		carGo(-50);
		claw.rotate(-80);
		//	Mypilot.rotate(90);
//		clawStar();
	}
	
	static float humanoBlanco = 16;
	static float humanoNegro = 1;
	
	public static void main(String[] args) {
		// TODO Auto-genera
		
		while(Button.ESCAPE.isUp()){
			while (Button.DOWN.isUp())
				get_distance();
			strategyaUnoSimple(5);
		}
		
		//while(Button.ENTER.isUp())
		
	}

}
