import java.util.ArrayList;
import java.util.Random; 

import DifferentialPilot.GyroSensor;
import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;

public class Qlearning {
	
	static int n = 4;
	static int d = 0;
	static int goal, start;
	static boolean restart = false;
	static ArrayList<Integer> obstacles = new ArrayList<Integer>();
	static float [ ] [ ] Q = new float [ n*n ] [ n*n ] ;
	static float [ ] [ ] R = new float [ n*n ] [ n*n ] ;
	
	private static EV3GyroSensor gs;
	private static EV3LargeRegulatedMotor mB,mB1;
	private static SampleProvider gyroSamples;
	static float[] sample= { 0.0f };
	static Wheel wheel1,wheel2;
	static WheeledChassis chassis;
	static MovePilot pilot;
	
	public static void main(String[] args) {
		Qlearning test = new Qlearning();
	}
	
	public void ini(){
		
		for(int i=0; i<n*n; ++i){
	        
	        for(int j=0; j<n*n; ++j){
	            
	            Q[i][j]=0;
	        }
	    }
		
		for(int i=0; i<n*n; ++i){
	        
	        for(int j=0; j<n*n; ++j){
	            
	            if( (i%n!=0 && j==i-1) || ((i+1)%n!=0 && j==i+1) || (i-n>=0 && j==i-n) || (i+n<n*n && j==i+n) )
	                //R[i][j]=0;
	                R[i][j]=-0.04f;
	            else
	                R[i][j]=-1;
	        }
	    }
		
		/*for(int i=0; i<n*n; ++i){
	        
	        for(int j=0; j<n*n; ++j){
	            
	        	System.out.print(R[i][j]+" ");
	        }
	        System.out.print("\n");
	    }*/
		System.out.print("Iniciado\n");
	}
	
	public void setGoal(int x, int y){

	    goal=x*n+y;

	    if(goal%n !=0) R[goal-1][goal]=100;
	    if((goal+1)%n!=0) R[goal+1][goal]=100;
	    if(goal-n>=0) R[goal-n][goal]=100;
	    if(goal+n<n*n) R[goal+n][goal]=100;

	    R[goal][goal]=100;

	    System.out.print("Goal seteado\n");
	}
	
	void setStart(int x, int y){

	    start=x*n+y;

	    System.out.print("Start seteado\n");
	}
	
	void setObstacle(int x, int y){

	    int obstacle=x*n+y;

	    if(obstacle%n !=0) R[obstacle-1][obstacle]=-10;
	    if((obstacle+1)%n!=0) R[obstacle+1][obstacle]=-10;
	    if(obstacle-n>=0) R[obstacle-n][obstacle]=-10;
	    if(obstacle+n<n*n) R[obstacle+n][obstacle]=-10;

	    R[obstacle][obstacle]=-10;
	    obstacles.add(obstacle);
	    System.out.print("Obstacle seteado\n");
	}
	
	public float maxi(int state){

	    float maxi=-1;
	    ArrayList<Integer> posibles = new ArrayList<Integer>();
	    for(int i=0; i<n*n; ++i){
	        if(R[state][i] !=-1)
	            posibles.add(i);
	    }

	    for(int i=0; i<posibles.size(); ++i){

	        if(Q[state][posibles.get(i)]>maxi)
	            maxi = Q[state][posibles.get(i)];
	    }

	    return maxi;
	}
	
	int qlearning_step(int state){

	    int next_state;
	    if(state==goal){
	    	//System.out.print(state+" "+goal);
	    	next_state=start;
	    	System.out.print("\n");
	    	d=0;
	    	restart=true;
	    	Button.waitForAnyPress();
	    	return next_state;
	    }
	    for(int i=0; i<obstacles.size(); ++i){
	        if(state==obstacles.get(i)) {
	            next_state=start;
	            System.out.print("\n");
	            d=0;
	            restart=true;
		    	Button.waitForAnyPress();
		    	return next_state;
	        }
	    }

	    ArrayList<Integer> posibles = new ArrayList<Integer>();
	    for(int i=0; i<n*n; ++i){
	        if(R[state][i] !=-1)
	            posibles.add(i);
	    }

	    Random rand = new Random();
	        
        float aux=maxi(state);
        ArrayList<Integer> new_posibles = new ArrayList<Integer>();
        for(int i=0; i<posibles.size(); ++i){
            if(Q[state][posibles.get(i)]==aux) new_posibles.add(posibles.get(i));
        }
        next_state = new_posibles.get(rand.nextInt(new_posibles.size()));
	 
	    Q[state][next_state] = R[state][next_state] + 0.8f*maxi(next_state);
	    //Q[state][next_state] += 0.7*(R[state][next_state] + 0.9*maxi(next_state) - Q[state][next_state]);
	    System.out.print(next_state+" ");
	    return next_state;
	}
	
	public static void giro(int angule, boolean clockwise) {

	   gyroSamples.fetchSample(sample, 0);
	   System.out.println(sample[0]);
	   
	   System.out.println("TestChassis");
	   
	   if(clockwise == true){
		   pilot.rotate(-angule);   // agree clockwise
		   gyroSamples.fetchSample(sample, 0);
		   System.out.println(sample[0]);
		   
		   while(-sample[0]<angule){
			   pilot.rotate(-angule - sample[0]);
			   gyroSamples.fetchSample(sample, 0);
			   System.out.println(sample[0]);
		   }
		   while(-sample[0]>angule){
			   pilot.rotate(-angule - sample[0]);
			   gyroSamples.fetchSample(sample, 0);
			   System.out.println(sample[0]);
		   }
	   }

	   if(clockwise == false){
		   pilot.rotate(angule); // degree clockwise
		   System.out.println("giro");
		   gyroSamples.fetchSample(sample, 0);
		   System.out.println(sample[0]);
		   
		   while(sample[0]<angule){
			   pilot.rotate(angule - sample[0]);
			   gyroSamples.fetchSample(sample, 0);
			   System.out.println(sample[0]);
		   }
		   while(sample[0]>angule){
			   pilot.rotate(angule - sample[0]);
			   gyroSamples.fetchSample(sample, 0);
			   System.out.println(sample[0]);
		   }
	   }
	   
   }
	
	public void Move(int posini, int posfin){
		
		int xi= posini/n;
		int yi= posini%n;
		int xf= posfin/n;
		int yf= posfin%n;
		
		gs.reset();
		
		if(xi==xf && yi-yf==1){
			if(d==0){
				System.out.println("1 anti");
				giro(90,false);
				d=3;
			}
			else if(d==2){
				System.out.println("1 hora");
				giro(90,true);
				d=3;
			}
			else if(d==1){
				giro(180,true);
				d=3;
			}
		}
		else if(xi==xf && yi-yf==-1){
			if(d==0){
				System.out.println("2 hora");
				giro(90,true);
				d=1;
			}
			else if(d==3){
				giro(180,true);
				d=1;
			}
			else if(d==2){
				System.out.println("2 anti");
				giro(90,false);
				d=1;
			}
		}
		else if(xi-xf==1 && yi==yf){
			if(d==3){
				System.out.println("3 hora");
				giro(90,true);
				d=0;
			}
			if(d==2){
				giro(180,false);
				d=0;
			}
			else if(d==1){
				System.out.println("3 anti");
				giro(90,false);
				d=0;
			}
		}
		else if(xi-xf==-1 && yi==yf){
			if(d==0){
				giro(180,true);
				d=2;
			}
			if(d==3){
				System.out.println("4 anti");
				giro(90,false);
				d=2;
			}
			else if(d==1){
				System.out.println("4 hora");
				giro(90,true);
				d=2;
			}
		}
		
		pilot.travel(200);
	}
	
	public Qlearning(){
		
		ini();
		setGoal(0,0);
		setStart(3,3);
		setObstacle(2,2);
		
		int next_state=start;
		int actual_state=start;
		
		mB = new EV3LargeRegulatedMotor(MotorPort.B);
		mB1 = new EV3LargeRegulatedMotor(MotorPort.A);
		gs = new EV3GyroSensor(SensorPort.S2);
		gyroSamples = gs.getAngleMode();
		wheel1 = WheeledChassis.modelWheel(mB1, 56.0).offset(-55.0);
	   //offset es la distancia del centro a la rueda
	   wheel2 = WheeledChassis.modelWheel(mB, 56.0).offset(55.0);
	   chassis = new WheeledChassis(new Wheel[] { wheel1, wheel2 }, WheeledChassis.TYPE_DIFFERENTIAL);
	   pilot = new MovePilot(chassis);
		//giro(90,true);
		
		for(int i=0; i<200; ++i){
			System.out.print("Direccion " + d + "\n");
			//System.out.print("Iteracion" + i + "\n");
			next_state=qlearning_step(next_state);
			if(!restart) {
				System.out.print("Muevo de " + actual_state + "a" + next_state + "\n");
				Move(actual_state, next_state);
			}
			else{
				System.out.println("Restart");
				restart = false;
			}
			actual_state = next_state;
		}
	   
	   //Move(15,11);
	   //Move(11,10);
		
	}

}
