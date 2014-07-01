import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import lejos.nxt.*;
import lejos.nxt.comm.Bluetooth;
import lejos.nxt.comm.NXTConnection;
import lejos.nxt.Sound;
import lejos.nxt.Motor;
import lejos.nxt.Battery;
import lejos.robotics.navigation.DifferentialPilot;


public class main {

	public static void main(String[] args) {
		
		
		boolean taskActive = false;
		int stepDriveTacho = 0;
		int stepDriveCurrentTacho = 0;
		int stepCameraSteps = 0;
		int stepCameraCurrentStep = 0;
		int stepDriveDelay = 0;
		int stepCameraDelay = 0;
		long stepMillis = 0;
		int steps = 0;
		int currentStep = 0;
		int taskData[][] = new int[20][4];
		
		DifferentialPilot pilot = new DifferentialPilot(2.1f, 4.4f, Motor.A, Motor.B, false);  // parameters in inches
		
		
		
		
		
		
		LCD.drawString("waiting for Connection",0,0);

	    LCD.refresh();

	    //Listen for incoming connection

	        NXTConnection btc = Bluetooth.waitForConnection();

	        btc.setIOMode(NXTConnection.RAW);
	    Sound.beep();

	    LCD.clear();

	    LCD.drawString("connected",0,0);

	    LCD.refresh();  
	    
	    DataInputStream dis=btc.openDataInputStream();
	    DataOutputStream dos=btc.openDataOutputStream();
	    
	    while(!Button.ESCAPE.isDown()){
	    	
	    	byte msg = 0;
	    	
	    	
	    	try{
	    		if (dis.available() != 0) {
	    			msg=dis.readByte();
		    	}
	    	} catch (IOException e) {
	    		
	    	}
	    	
	    	//Read Tachocount MSG 10 (First byte = 10, Second Byte MotorID (0=A, 1=B, 2=C))
	    	if (msg == 10){
	    		int tachocount = -1;
	    		byte motor = -1;
	    		
	    		try{
	    			motor=dis.readByte();
		    	} catch (IOException e) {
		    		
		    	}
	    		
	    		//read Tachocount
	    		if (motor == 0) {
	    			tachocount = Motor.A.getTachoCount();
	    		} else if ( motor == 1) {
	    			tachocount = Motor.B.getTachoCount();
	    		} else if ( motor == 2) {
	    			tachocount = Motor.C.getTachoCount();
	    		}
	    		
	    		//Write Tachocount
	    		try {
	    			dos.writeInt(tachocount);
	    			dos.flush();
	    			LCD.clear();
	    			LCD.drawString("Send Tachocount:", 0, 4);
	    			LCD.drawInt(tachocount, 0, 5);
	    		} catch (IOException e) {
	    			
	    		}
	    		
	    	//Rotate to	MSG 11 (First byte = 10, Second Byte MotorID (0=A, 1=B, 2=C), Third Integer Tacho)
	    	} else if (msg == 11){
	    		
	    		byte motor = -1;
	    		int tacho = 0;
	    		
	    		try{
	    			motor = dis.readByte();
	    			tacho = dis.readInt();
		    	} catch (IOException e) {
		    		
		    	}
	    		
	    		//rotate to
	    		if (motor == 0) {
	    			Motor.A.rotateTo(tacho, true);
	    		} else if ( motor == 1) {
	    			Motor.B.rotateTo(tacho, true);
	    		} else if ( motor == 2) {
	    			Motor.C.rotateTo(tacho, true);
	    		}
	    		
	    		//Send Succes
	    		try {
	    			dos.writeByte(1);
	    			dos.flush();
	    			LCD.clear();
	    			LCD.drawString("Rotate to:", 0, 4);
	    			LCD.drawInt(tacho, 0, 5);
	    		} catch (IOException e) {
	    			
	    		}
	    		
	    	//MSG -1 Quit	
	    	} else if (msg == -1) {
	    		System.exit(0);
	    		
	    	//MSG 1 Status	
	    	}	else if (msg == 1) {
	    		
	    	//MSG 2 Step Status
	    	} else if (msg == 2) {
	    		try {
	    			dos.writeBoolean(taskActive);
	    			dos.writeInt(currentStep);
	    			dos.flush();
	    			
	    		} catch (IOException e) {
	    			
	    		}
	    		
	    	//MSG 20 Receive Task
	    	} else if (msg == 20) {
	    		try {
	    			steps = dis.readInt();
	    			for (int i = 0; i <= steps; i++) {
	    				taskData[i][0] = dis.readInt();
	    				taskData[i][1] = dis.readInt();
	    				taskData[i][2] = dis.readInt();
	    				taskData[i][3] = dis.readInt();
	    			}
	    			dos.writeByte(1);
	    			dos.flush();
	    			
	    			
	    			
	    		} catch (IOException e) {
	    			
	    		}
	    	//MSG 21 Start Task	
	    	} else if (msg== 21) {
	    		try {
	    			taskActive = true;
	    			stepMillis = System.currentTimeMillis();
	    			stepDriveTacho = taskData[0][0];
	    			stepDriveDelay = taskData[0][1];
	    			stepDriveCurrentTacho = 0;
	    			currentStep = 1;
	    			dos.writeByte(1);
	    			dos.flush();
	    			
	    			
	    			
	    		} catch (IOException e) {
	    			
	    		}
	    		
	    	} else if (msg == 5) {
	    		try {
	    			dos.writeFloat(Battery.getVoltage());;
	    			
	    			dos.flush();
	    			
	    		} catch (IOException e) {
	    			
	    		}
	    	//MSG 50 Motor forward (First: Byte = 50, Second Byte MotorID (0=A, 1=B, 2=C), Third Int Motorspeed;
	    	} else if (msg==50) {
	    		byte motor = -1;
	    		int motorSpeed = 0;
	    		try {
	    			motor = dis.readByte();
	    			motorSpeed = dis.readInt();
	    		} catch (IOException e) {
	    			
	    		}
	    		
	    		if (motor == 0) {
	    			Motor.A.setSpeed(motorSpeed);
	    			Motor.A.forward();
	    		} else if ( motor == 1) {
	    			Motor.B.setSpeed(motorSpeed);
	    			Motor.B.forward();
	    		} else if ( motor == 2) {
	    			Motor.C.setSpeed(motorSpeed);
	    			Motor.C.forward();
	    		}
	    		try {
	    			dos.writeBoolean(true);
	    			dos.flush();
	    		} catch (IOException e) {
	    			
	    		}
	    		
	    	//MSG 51 Motor backward
	    	} else if (msg==51) {
	    		byte motor = -1;
	    		int motorSpeed = 0;
	    		try {
	    			motor = dis.readByte();
	    			motorSpeed = dis.readInt();
	    		} catch (IOException e) {
	    			
	    		}
	    		
	    		if (motor == 0) {
	    			Motor.A.setSpeed(motorSpeed);
	    			Motor.A.backward();
	    		} else if ( motor == 1) {
	    			Motor.B.setSpeed(motorSpeed);
	    			Motor.B.backward();
	    		} else if ( motor == 2) {
	    			Motor.C.setSpeed(motorSpeed);
	    			Motor.C.backward();
	    		}
	    		try {
	    			dos.writeBoolean(true);
	    			dos.flush();
	    		} catch (IOException e) {
	    			
	    		}
	    	//Msg 52 Motor Stop (First Byte = 52, Second Byte MotorID)
	    	} else if (msg == 52) {
	    		byte motor = -1;
	    		
	    		try {
	    			motor = dis.readByte();
	    			
	    		} catch (IOException e) {
	    			
	    		}
	    		
	    		if (motor == 0) {
	    			
	    			Motor.A.stop();;
	    		} else if ( motor == 1) {
	    			
	    			Motor.B.stop();
	    		} else if ( motor == 2) {
	    			
	    			Motor.C.stop();
	    		}
	    		try {
	    			dos.writeBoolean(true);
	    			dos.flush();
	    		} catch (IOException e) {
	    			
	    		}
	    	//MSG 53 Drive Forward
	    	} else if (msg == 53) {
	    		int motorSpeed = 0;
	    		try {
	    			motorSpeed = dis.readInt();
	    		} catch (IOException e) {
	    			
	    		}
	    		pilot.setTravelSpeed(10);
	    		pilot.setAcceleration(10);
	    		pilot.forward();
	    		
	    		try {
	    			dos.writeBoolean(true);
	    			dos.flush();
	    		} catch (IOException e) {
	    			
	    		}
	    	//MSG 54 Drive Backward
	    	} else if (msg == 54) {
	    		int motorSpeed = 0;
	    		try {
	    			motorSpeed = dis.readInt();
	    		} catch (IOException e) {
	    			
	    		}
	    		pilot.setTravelSpeed(10);
	    		pilot.setAcceleration(10);
	    		pilot.backward();
	    		
	    		try {
	    			dos.writeBoolean(true);
	    			dos.flush();
	    		} catch (IOException e) {
	    			
	    		}
	    	//MSG 55 Drive Stop
	    	} else if (msg == 55) {
	    		byte motor = -1;
	    		
	    		
	    		pilot.stop();
	    		
	    		try {
	    			dos.writeBoolean(true);
	    			dos.flush();
	    		} catch (IOException e) {
	    			
	    		}
	    	}
	    	
	    	
	    	
	    	
	    	
	    	
	    	
	    	
	    	
	    	
	    	
	    	
	    	
	    	
	    	if (taskActive==true) {
	    		
	    		if (stepMillis + stepDriveDelay <= System.currentTimeMillis()) {
	    			
	    			
	    			stepMillis = stepMillis + stepDriveDelay;
	    			if (stepDriveCurrentTacho < stepDriveTacho) {
	    				stepDriveCurrentTacho++;
	    			} else {
	    				stepDriveCurrentTacho--;
	    			}
	    			
	    			Motor.A.rotateTo(stepDriveCurrentTacho, true);
	    			Motor.B.rotateTo(stepDriveCurrentTacho, true);
	    			if (stepDriveCurrentTacho == stepDriveTacho) {
	    				currentStep++;
	    				stepDriveTacho = taskData[currentStep-1][0];
		    			stepDriveDelay = taskData[currentStep-1][1];
	    				LCD.clear();
	    				LCD.drawString("Step finished", 0, 4);
	    				if (currentStep > steps) {
	    					taskActive = false;
	    					LCD.clear();
		    				LCD.drawString("Task finished", 0, 4);
	    				}
	    			}
	    		}
	    	}
	    
	    
	    
	    
	    
	    }
	}
}
