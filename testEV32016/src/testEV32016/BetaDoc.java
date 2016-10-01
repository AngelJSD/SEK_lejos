package testEV32016;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.*;

import java.util.Vector;

import lejos.hardware.lcd.*;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.*;
import lejos.utility.Delay;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;

public class BetaDoc {
	
	static EV3LargeRegulatedMotor rightW = new EV3LargeRegulatedMotor(MotorPort.D);
	static EV3LargeRegulatedMotor letfW = new EV3LargeRegulatedMotor(MotorPort.A);
	static Wheel leftWheel = WheeledChassis.modelWheel(Motor.A, 13.4).offset(72).gearRatio(2);
	static Wheel rightWheel = WheeledChassis.modelWheel(Motor.D, 13.4).offset(-72).gearRatio(2);
	static Chassis myChassis = new WheeledChassis (new Wheel[]{leftWheel,rightWheel}, WheeledChassis.TYPE_DIFFERENTIAL);
	static EV3UltrasonicSensor ultraSonic = new EV3UltrasonicSensor(SensorPort.S1);
	static EV3MediumRegulatedMotor claw = new EV3MediumRegulatedMotor(MotorPort.B);
	static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
	
	public static void take()
	{
		LCD.drawString("Atrapando...", 0, 0);
		claw.rotate(20);
		LCD.clear();
	}
	
	public static void pickUp()
	{
		LCD.drawString("Recogiendo...", 0, 0);
		take();
		claw.rotateTo(50);
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
	//Girara en sentido horario o antihoracio
	public static void turnLoR( boolean sense )
	{
		if(sense == true)
			clockwise();
		else
			counterClockwise();
	}
	
	// usar el color blanco
	public static boolean differentColor(int color)
	{
		if(colorSensor.getColorID()==color)
			return false;
		else
			return true;
	}
	
	public static void setAcceleratioBoth(int speed)
	{
		rightW.setAcceleration(speed);
		letfW.setAcceleration(speed);
	}
	
	public static void advanceToDiffColorWhite()
	{
		LCD.drawString("Avanzando Hatas no ser blanco...", 0, 0);
		while (differentColor(0) == true){
				rightW.forward();
				letfW.forward();
		}
		rightW.stop();
		letfW.stop();
		LCD.clear();
	}
	
	public static float get_distance(){
		LCD.drawString("Capturando Pokemon....", 5, 0);
		SampleProvider spSonic = ultraSonic.getDistanceMode();
		float[] sampleIzq = new float[spSonic.sampleSize()];
		spSonic.fetchSample(sampleIzq, 0);
		LCD.drawString("te la creite XD(Distancia).."+sampleIzq[0]*100, 6, 0);
		return sampleIzq[0]*100;
	}
	
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		
		// DifferentialPilot m = new DifferentialPilot(2.95f , 5.25f , Motor.D , Motor.B);
		//----- modificar---------------------
		/*
		public Person getLocatePerson(float angulo){
			//El vector devuelto tiene como primer dato la distancia y como segundo es el angulo
				getGyroSen().reset();
				
				getCar().setAcceleration(50);
			
				SampleProvider spGy = getGyroSen().getAngleMode();
				SampleProvider spUsDer = getSonSen().getDistanceMode();
				
				float[] sampleGyro = new float[spGy.sampleSize()];
				float[] sampleUltDer = new float[spUsDer.sampleSize()];
				
				Vector<Float> l = new Vector<Float>();
				Vector<Float> a = new Vector<Float>();
				
				spGy.fetchSample(sampleGyro, 0);
				
				SampleProvider ussample = getSonSen().getDistanceMode();
				SampleProvider gssample = getGyroSen().getAngleMode();
				
				float[] sampleUs = new float[ussample.sampleSize()];
				float[] sampleGs = new float[ussample.sampleSize()];
				
				getCar().arcForward(0);
				
				while(getCar().isMoving() ){
					ussample.fetchSample(sampleUs, 0);
					gssample.fetchSample(sampleGs, 0);
					Position pos = new Position(getOdometro().getPose().getX(), getOdometro().getPose().getY(), sampleGs[0]);
					getMap().pintNoOcup(pos, sampleUs[0]);
					getMap().pintOcupado(pos, sampleUs[0]);
					
					if(Button.ENTER.isDown()){
						System.exit(0);
					}
					
					spGy.fetchSample(sampleGyro, 0);
					spUsDer.fetchSample(sampleUltDer, 0);
					
					l.add(sampleUltDer[0]);
					a.add(sampleGyro[0]);
				
					if(sampleGyro[0] > 0){
						if(l.get(0)<l.get(1)){
							l.remove(1);
							a.remove(1);	
						}else{
							l.remove(0);
							a.remove(0);
						}
					}
					if(sampleGyro[0]!=angulo) getCar().stop();
				}
				getCar().forward();
				while(getCar().isMoving() ){
					ussample.fetchSample(sampleUs, 0);
					gssample.fetchSample(sampleGs, 0);
					Position pos = new Position(getOdometro().getPose().getX(), getOdometro().getPose().getY(), sampleGs[0]);
					getMap().pintNoOcup(pos, sampleUs[0]);
					getMap().pintOcupado(pos, sampleUs[0]);
					if(Button.ENTER.isDown()){
						System.exit(0);
					}
					if(getColGarrSen().getColorID() == -1) getCar().stop();
					
				}
				
				getCar().setAcceleration(5);
				
				getCar().arcForward(0);
				while(getColGarrSen().getColorID()==7){
					ussample.fetchSample(sampleUs, 0);
					gssample.fetchSample(sampleGs, 0);
					Position pos = new Position(getOdometro().getPose().getX(), getOdometro().getPose().getY(), sampleGs[0]);
					getMap().pintNoOcup(pos, sampleUs[0]);
					getMap().pintOcupado(pos, sampleUs[0]);
					if(Button.ENTER.isDown()){
						System.exit(0);
					}
				}
				getCar().stop();
				if( getColGarrSen().getColorID() == 0	){
					LCD.drawString("Si Si el COLOr",4,4);
					getGarrMot().setAcceleration(10);
					
					getCar().setTravelSpeed(90);
					getCar().travel(-5);
					
					getCar().travel(5);
					getGarrMot().rotateTo(300);
					
				}else{
					LCD.drawString("No es el COLOr",4,4);
//					Delay.msDelay(1000);
					}
					
		
					
				
				Person p = new Person(l.get(0)*Math.sin(75+a.get(0)), a.get(0), getColGarrSen().getColorID());
				return p;
		} */
	}

}
