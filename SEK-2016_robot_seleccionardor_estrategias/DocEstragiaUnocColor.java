package testSEK;

//import java.awt.List;
import java.util.List;
import java.util.ArrayList;

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

public class DocEstragiaUnocColor {

	static EV3LargeRegulatedMotor rightW = new EV3LargeRegulatedMotor(MotorPort.D);
	static EV3LargeRegulatedMotor letfW = new EV3LargeRegulatedMotor(MotorPort.A);
	static DifferentialPilot Mypilot = new DifferentialPilot(56, 128, rightW, letfW, true);
	static OdometryPoseProvider Odometro = new OdometryPoseProvider(Mypilot);
	
	
	static EV3UltrasonicSensor ultraSonic = new EV3UltrasonicSensor(SensorPort.S4);
	static EV3MediumRegulatedMotor claw = new EV3MediumRegulatedMotor(MotorPort.B);
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
	
	public static void imprimirMensaje(String mensaje)
	{
		LCD.drawString(mensaje, 0, 0);
		LCD.clear();
	}
	
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
		LCD.drawString(".."+sampleIzq[0]*100, 0, 3);
		
		return sampleIzq[0]*100;
	}	
	
	public static float get_Gyros(){
		
		SampleProvider spSonic = gyroSensor.getAngleMode();
		float[] sampleIzq = new float[spSonic.sampleSize()];
		spSonic.fetchSample(sampleIzq, 0);
		float grados = sampleIzq[0]%360;
		LCD.drawString("grados.. "+grados, 0, 5);
		LCD.clear();
		return grados;
	}
	
	public static int getColorHumanACoger()
	{
		int colorPiso =colorSensor.getColorID();
		int colorHuman=-1;
		
		if (colorPiso == coloPisoAzul){			
			colorHuman = colorHumanBlanco;
			LCD.drawString("Humano Blando",0,5);
			LCD.clear();}
		
		if (colorPiso==coloPisoVerde){
			colorHuman = colorHumanNegro;
			LCD.drawString("Black Humano",0,5);
			LCD.clear();}
		
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
		claw.rotateTo(3000);
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
		claw.rotateTo(0);
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
	
	public static void advanceToDiffDist_AndColor(int distance)
	{
		LCD.drawString("Avanzando Hatas no ser...", 0, 0);
		Mypilot.backward();
		while ( (colorSensorHuman.getColorID()==-1)|| get_distance() >= distance  ){
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
	
	public static float get_ColorRedMode(){
		LCD.drawString("Color Red", 0, 0);
		SampleProvider spSonic = colorSensorHuman.getRedMode();
		float[] sampleIzq = new float[spSonic.sampleSize()];
		spSonic.fetchSample(sampleIzq, 0);
		LCD.drawString(".."+sampleIzq[0]*100, 0, 2);
		LCD.drawString("...."+sampleIzq[0], 0, 3);
		
		return sampleIzq[0]*100;
	}
	
	public static void girarArcoPilot(int angulo,boolean sentido)
	{
		int count;
		if (sentido == true){	
			Mypilot.arcForward(angulo);
			while(get_distance()>=mejor_distancia)
				Mypilot.setAngularSpeed(speed_gyros);
				
		}
		
		else {
			Mypilot.arcBackward(angulo);
			while(get_distance()>=mejor_distancia)
				Mypilot.setAngularAcceleration(speed_gyros);
		}
			
	}
	
	public static void girarHataAngulo( boolean sentido,float angulo)
	{
		if (sentido == true){	
			Mypilot.arcForward(0);
			while(get_Gyros()!=angulo)
				Mypilot.setAngularSpeed(speed_gyros);
				
		}
		
		else {
			Mypilot.arcBackward(angulo);
			while(get_Gyros()!=angulo)
				Mypilot.setAngularAcceleration(speed_gyros);
		}
		
	}
	
	public static void Sujetar(int grados)
	{
		claw.setSpeed(750);
		claw.rotate(grados);
		
	}
	
	
	static void StrategyaUnoColor()
	{ 
		moveToDiffColor(7,true);
		carGo(40);
		setAcceleratioBoth(500);
		girarArcoPilot(6,false);
		advanceToDiffColorHuman(6);
		advanceToEqualColor(6, 500);// calibrarb colores
		pickUp();
		clawStar();
	}
	
	static void strategyaUnoSimple( int distanciaParaAtrapar)
	{
		moveToDiffColor(coloSintaNegra,true);
		advanceToDiffDistant(distanciaParaAtrapar);
		pickUp();
		moveToDiffColor(coloSintaNegra, false);
		carGo(-50);
		Mypilot.rotate(90);
		clawStar();
	}
	
	static int speed_gyros = 150;
	static int mejor_distancia = 30;
	static float humanoBlanco = 16;
	static float humanoNegro = 1;
	static boolean colorHuman;
	static float[] ArrayGyros = new float[100];
	
	
	public static void main(String[] args) {
		// TODO Auto-generated method stub

		get_Gyros();
		float iq_colorTest;
		float iq_color = (humanoBlanco+humanoNegro)/2;
		int posicionDeGyrosArray = 0;
		
		moveToDiffColor(7,true);
		carGo(40);
		
		girarArcoPilot(0, false);
		while(get_distance()>=mejor_distancia)
			Mypilot.rotate(5);
		
		// add a lista = get grades;0
		ArrayGyros[posicionDeGyrosArray] = get_distance(); 
		posicionDeGyrosArray++;
		
		advanceToDiffDist_AndColor(7);
		Sujetar(2500);
		//while(Button.ENTER.isUp()){
			iq_colorTest = get_ColorRedMode();
			if( iq_colorTest < iq_color){
				LCD.drawString("Es Negro", 0, 5);
				colorHuman = true;}
			else{
				LCD.drawString("Es Blanco", 0, 5);
				colorHuman = false;
			}
		//}
		if(colorHuman==true){
			Sujetar(4000);
//			carGo(-50);
			Sujetar(-5500);}
		else{
//			carGo(-50);
			Sujetar(-2500);
		}
		girarHataAngulo(true, ArrayGyros[0]);
		moveToDiffColor(7, false);
		carGo(-20);
		
		if(colorHuman==true)
			Sujetar(-5500);
		
		else
			Sujetar(-2500);
		
	}

}
