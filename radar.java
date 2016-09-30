import lejos.nxt.*;
//import lejos.nxt.Button;
//import lejos.nxt.LCD;
//import lejos.nxt.Motor;
import lejos.util.Delay;
import lejos.nxt.comm.*;
import lejos.nxt.addon.*;
import lejos.nxt.comm.BTConnection;
import lejos.nxt.comm.Bluetooth;
import java.io.*;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.localization.OdometryPoseProvider;

import javax.bluetooth.*;

public class radar {

     public static void main(String[] args)  throws Exception
     {
          //Bluetooth inicio
          String nombre = "CR5";
	  LCD.drawString("Conectando?", 2, 1);
	  LCD.refresh();

	  LCD.drawString("Pulsa un boton", 2, 1);
	  Button.waitForAnyPress();
	  Sound.beep();

	  LCD.clear();
	  LCD.drawString("Conectando", 2, 1);
	  LCD.refresh();

	  RemoteDevice bt2 = Bluetooth.getKnownDevice(nombre);

	  if (bt2 == null){
		  LCD.clear();
		  LCD.drawString("No existe ese dispositivo", 0, 1);
		  LCD.refresh();
		  Thread.sleep(2000);
		  System.exit(1);
		  }

	  BTConnection btc = Bluetooth.connect(bt2);

	  if (btc == null){
		  LCD.clear();
		  LCD.drawString("Conexión fallida", 1, 1);
		  LCD.refresh();
		  Thread.sleep(2000);
		  System.exit(1);
		  }


	  LCD.clear();
	  LCD.drawString("Conectado", 2, 1);
	  LCD.refresh();

	  DataOutputStream dos = btc.openDataOutputStream();
	  //Bluetooth fin

          UltrasonicSensor ultra = new UltrasonicSensor(SensorPort.S1);
          DifferentialPilot dp = new DifferentialPilot(56, 300, Motor.B, Motor.C);
	  OdometryPoseProvider opp = new OdometryPoseProvider(dp); //para dar X,Y avanzados (aun no usado)

          //Imprimir en consola (para usar, descomentar todo lo que diga RConsole y seguir las instrucciones)
          //RConsole.openUSB(40000);
          //RConsole.open();

          LCD.drawString("Maestro", 0, 0);

          //Motor del ultra
          Motor.A.setSpeed(30);

	  // n de victimas/ n de giros/ límite de giros (para empezar la salida)/ posible victima (contador)/ explorar a la derecha o a la izquierda (1 o -1)
	  int victimas,giros,lim,pv,direc;
	  int[] forward;// Array que almacena cuanto avanzar (mirar el grafico para entender mejor)
	  forward = new int[5];
	  double d,d1,t,tt,ant,nuevo;// variables para descomponer en X,Y el vector dado por el ultra
	  boolean control1,pw,salida,non_scanned;// variables de control

	  ant=0;
	  giros=0;
	  pv=0;
	  lim=3;
	  salida=false;
	  non_scanned=false;
	  forward[0]=0;
	  forward[1]=6;
	  forward[2]=0;
	  forward[3]=0;
	  forward[4]=0;
	  giros=0;
	  direc=1;

	  //Salir de zona de rescate
	  dp.travel(-600);//*
	  dp.rotate(90);//*
	  //Salir de zona de rescate

	  while(ultra.getDistance()>10){
          	dp.travel(-100);
          	++forward[0];
	  }
	  dp.rotate(-90);
	  dp.travel(300);
	  Motor.A.backward();
	  while (Motor.A.getTachoCount()>-45);
	  Motor.A.stop();
	  Motor.A.resetTachoCount();

	  giros=1;

	  while(true){

		if(salida==false){
			if(forward[giros]==0 || non_scanned){//Aun no está escaneado
				non_scanned=true;
				Motor.A.forward();
				LCD.clear();
				//Delay.msDelay(1000);
				while (Motor.A.getTachoCount()<90);// Acomoda el ultrasonido de ida
				Motor.A.stop();
				victimas=0;
				control1=false;
				//pw=false;
				pv=0;
				ant=0;
				Motor.A.backward();
				while (Motor.A.getTachoCount()>0) {// Barre de vuelta
				    //pw=false;
				    d=ultra.getDistance();
				    if(d<10){ //Hay algo cerca
					  //pw=true;
					  ++pv; //AUmento contador de posibles victimas

					  //Obtengo X,Y IGNORAR POR AHORA
					  t=180-Motor.A.getTachoCount()-45;
					  tt=t*Math.PI/180;// CONVIERTO
					  d1=d*Math.sin(tt);
					  if(ant==0) ant=d1;
					  nuevo=d1;
					  if(Math.abs(nuevo-ant)>7 && !control1){
						control1=true;
						victimas=victimas+1;
					  }
					  else if(Math.abs(nuevo-ant)>7 && control1) control1=false;
					  //RConsole.println(""+d1+" "+d*Math.cos(tt)+" "+d+" "+victimas+" "+pv); //Y,X
					  ant=d1;
					  //Obtengo X,Y IGNORAR POR AHORA
				    }
				}
				//RConsole.println("\n");
				Motor.A.stop();

				if (pv<20) { //No es pared
				  if(pv!=0){ //Es víctima debe abrir la puerta
					dos.writeInt(0);
					dos.flush();
					Thread.sleep(5000);
					dp.travel(-300);//*
					forward[giros]+=2;//solo aumento 2 porque al cerrar la puerta retrocederá 10cm
				  }
				  dp.travel(-100); //*//cuando no es víctima, simplemente avanza 10cm
				  dp.stop();
				  if(pv!=0){ //Es víctima debe cerrar la puerta
					dos.writeInt(1);
					dos.flush();
					Thread.sleep(5000);
					dp.travel(100);//*// Retrocede 10 cm
				  }
				  ++forward[giros];

				} else { //Es pared debe girar
				  dp.rotate(-90 *direc);//*
				  ++giros;
				  if(giros==lim) { //Si alcanza el límite de giros, debe empezar la regresada

					salida=true;
					dp.rotate(-90 * direc);//* // Giro 90 más y así completo 180
					dp.travel(300);//*// Se acomoda
					dp.travel(-100);//*// Avanza 10cm
					++lim;// Aumento el límite para la siguiente vez
				  }
				}
			}
			else{ // Ya está escaneado, sabe cuanto avanzar
				dp.travel(-forward[giros]*100);//* //Avanza *100 porque debe estar en milímitros
				if(giros==1){ //Para entrar a la zona
				  dp.rotate(90);//*
				  dp.travel(-500);//*
				  dp.rotate(90 * direc);//*
				}
				else if(giros==0){ //Siempre debe girar a la izquierda
				  dp.rotate(-90);//*
				  dp.travel(300);//*//Se acomoda
				}
				else {
				  dp.rotate(-90 * direc);//* //Siempre gira a la derecha o izquierda para recorrer por el rededor
				  dp.travel(300);//*//Se acomoda
				}
				++giros;
			}
		}
		else{
			//Salida está activada así que recorre el array forward de atras hacia adelante (usando a 'giros' como puntero) para volver por el camino que recorrió
			--giros;
			for(;giros>-1; --giros){

				//RConsole.println(""+giros);
				dp.travel(-forward[giros]*100); //*//avanza lo que almacenó *100 pues debe estar en milímetros
				if(giros==2) { //Para salir de la zona
					//RConsole.println("h");
					dp.rotate(-90 * direc);//*
					dp.travel(-500);//*
					dp.rotate(-90);//*

				}
				else if(giros==0) { //Para dejar los humanitos cerca de la zona de rescate
					dos.writeInt(0);
					dos.flush();
					Thread.sleep(5000);
					dp.travel(300);//*
					dos.writeInt(1);
					dos.flush();
					Thread.sleep(5000);
					dp.rotate(90);//* //Estos 90 mas *
				}
				else if(giros==1){ //En este caso siempre debe voltear a la derecha
				  dp.rotate(90);//*
				  dp.travel(300);//* //Se acomoda
				}
				else { //Aún está dentro de la zona
					dp.rotate(90 * direc);//* //siempre gira para la derecha o izquierda
					dp.travel(300);//* //Se acomoda
				}
			}
			dp.rotate(90);//* // * estos 90 completan 180 para volver a hacer el camino que ingresa a la zona
			giros=0;// Vuelve a empezar su camino
			salida=false;
			non_scanned=false;

		}
	  }
          //Button.waitForAnyPress();
     }

}
