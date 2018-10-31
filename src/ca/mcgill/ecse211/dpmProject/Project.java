package ca.mcgill.ecse211.dpmProject;

import lejos.hardware.sensor.*;
import ca.mcgill.ecse211.dpmProject.LightLocalizer;
import ca.mcgill.ecse211.dpmProject.Odometer;
import ca.mcgill.ecse211.dpmProject.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;

/**
 * this is the main class for Lab 5; 
 * it contains almost all the parameters/sensor initiations 
 * it will interact with the user through the EV3 module's display and buttons
 * to ask the user whether to initiate testing routine or the demo routine.
 * The routines are performed as consecutive methods within threads
 * @author Team12
 *
 */

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////NEED TO CHANGE/////////////////////////////
////////////////////////////////////////////////////////////////////////////////

public class Project {
	//inputs and parameters  that will be aquired from the wifi class
	public static final int SC = 0;
	public static final int TR = 2;
	public static final double LLx = 2;		//left lower x
	public static final double LLy = 2;		//left lower y
	public static final double URx = 5;		//upper right x
	public static final double URy = 6;		//upper right y
	public static final double usRange = 1.75;
	
	// The parameters for driving the robot
	public static final double OFF_SET = 14.65; //this is the offset from the back line-detecting light sensor to the wheelbase
	public static final double OFF_SET_R = 4; //this is the offset from the right side ultrasonic sensor to the wheelbase
	public static final double HOOK_SPEED = 10; //this is the speed used for the motor 
	public static final int LOW_SPEED = 50; //this is the slow speed for precious movement 
	public static final int MEDIUM_SPEED = 100; //this is the medium speed for intermediate movement
	public static final int HIGH_SPEED = 200; //this is the fast motor speed for less precious, faster movement (long distace travel)
	public static final double WHEEL_RAD = 2.085; 
	public static final double TRACK = 14.2;
	public static final double TILE_SIZE = 30.48;
	public static final int DETECT_DISTANCE = (int)(usRange*TILE_SIZE); //detection bandcenter for the right side ultrasonic sensor /// ahmed: you can modify this 
	public static final int RING_BAND = 20; //detection bandcenter for moving lose up to the ring for color identification 
	public static final int THRESHOLD = 300; //this is the line detection threshold for the line deteting color sensors
	
	//create port and object for the motors (4 in total)
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A")); //the motor for the left wheel
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B")); //the motor for the right wheel
	public static final EV3LargeRegulatedMotor armMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C")); //the motor for raising/lowering the arm 
	public static final EV3MediumRegulatedMotor hookMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("D")); //the motor for motorizing the hooks 
	
	//create object for the lcd display
	public static final TextLCD lcd = LocalEV3.get().getTextLCD();
	
	// create port and object for the ultrasonic sensor in the front (used for ultrasonic localization and when approaching the ring)
	public static final Port usPort = LocalEV3.get().getPort("S1");
	@SuppressWarnings("resource") // Because we don't bother to close this resource
	public static SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
	public static SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
	public static final float[] usData = new float[usDistance.sampleSize()];
	
	// create port and object for the light sensor for color recognition which is installed on the arm
	public static final Port portColor = LocalEV3.get().getPort("S2"); // get the port for the light (color sensor)
	public static final SensorModes myColor = new EV3ColorSensor(portColor); // create the color sensor object
	public static final SampleProvider myColorSample = myColor.getMode("RGB"); //set to RGB mode
	public static final float[] sampleColor = new float[3]; // create an array for the sensor readings
	
	//create port and object for the first light sensor for line detection in the left front
	public static final Port portLeftLine = LocalEV3.get().getPort("S3"); // get the port for the left front light sensor
	public static final SensorModes myLeftLine = new EV3ColorSensor(portLeftLine); // create the light sensor object
	public static final SampleProvider myLeftLineSample = myLeftLine.getMode("Red"); //set to Red mode
	public static final float[] sampleLeftLine = new float[myLeftLine.sampleSize()]; // create an array for the sensor readings 
	
	//create port and object for the second light sensor for line detection in the right front
	public static final Port portRightLine = LocalEV3.get().getPort("S4"); // get the port for the right front light sensor
	public static final SensorModes myRightLine = new EV3ColorSensor(portRightLine); // create the light sensor object
	public static final SampleProvider myRightLineSample = myRightLine.getMode("Red"); //set to Red mode
	public static final float[] sampleRightLine = new float[myRightLine.sampleSize()]; // create an array for the sensor readings 
																			

	
	/**
	 * This is the main method for the lab, it will prompt the user to choose which functionality to execute
	 * in a menu, and initiates the threads needed for the lab
	 * @param args
	 * @throws OdometerExceptions
	 */
	public static void main(String[] args) throws OdometerExceptions{
		final Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		int buttonChoice;
	
		do {
			lcd.clear(); //clear the display
			//prompt the user to select whether to run tests (press up), demo color classification (press left),
			//or perform the search and localize procedure
			lcd.drawString("^ tests", 0, 1);
			lcd.drawString("< Color", 0,2); 
		    lcd.drawString("> SandL", 0,3);
		    buttonChoice = Button.waitForAnyPress(); 
		} while (buttonChoice!=Button.ID_UP && buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
		
		//initiate the tests if pressed up
		if (buttonChoice ==Button.ID_UP) {
			int testChoice;
			do {
				lcd.clear();
				lcd.drawString("^ Color", 0, 1);
				lcd.drawString("< Radius", 0,2);
			    lcd.drawString("> Track", 0,3);
			    lcd.drawString("US sampling", 0,4);
				testChoice = Button.waitForAnyPress();
			}while(testChoice!=Button.ID_UP && testChoice != Button.ID_LEFT && testChoice != Button.ID_RIGHT && testChoice != Button.ID_DOWN);
			if(testChoice==Button.ID_LEFT) {
				//initiate the wheel radius tuning 
				lcd.clear();
				(new Thread() {
					public void run() {
						Tester.wheelRadCheck();
					}
				}).start();
			}
			if(testChoice==Button.ID_RIGHT) {
				//initiate the wheel base tuning 
				lcd.clear();
				(new Thread() {
					public void run() {
						Tester.trackCheck();
					}
				}).start();

			}
			if(testChoice==Button.ID_UP) {
				//initiate the color data collection
				lcd.clear();
				(new Thread() {
					public void run() {
						Tester.sample();
					}
				}).start();
			}
			if(testChoice==Button.ID_DOWN) {
				//initiate ultrasonic sensor data collection
				System.out.println("press down");
				lcd.clear();
				(new Thread() {
					public void run() {
						Tester.usSample(odometer);
					}
				}).start();
			}
		    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		    System.exit(0);
		}
		
		//initiate demo color calibration is left is pressed
		if(buttonChoice == Button.ID_LEFT) {
			// initiate the color data collection
			lcd.clear();
			(new Thread() {
				public void run() {
					try {
						Color.colorDemo();
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
			}).start();
		}
		
		//initiate the search and localize if right is pressed
		if(buttonChoice == Button.ID_RIGHT) {
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			
			(new Thread() {
				public void run() {
					//The following are the routine performed for search and localize
					//perform the localization routine
					UltrasonicLocalizer.risingEdge(usDistance, usData, odometer, leftMotor, rightMotor);
					try {
						System.out.println("....................light localization");
						LightLocalizer.lightLocalize(odometer, leftMotor, rightMotor);
					} catch (OdometerExceptions e) {
						e.printStackTrace();
					}	
					//navigate to the lower left corner of the search area
					
					
					
					leftMotor.stop(true);
					rightMotor.stop(false);
					for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
					      motor.setAcceleration(3000);
					    }
					try {
				      Thread.sleep(1000);
				    } catch (InterruptedException e) {}
				    
				    //move the robot to the way point
				    
					Navigation.travelTo(Project.LLx, odometer.getXYT()[1]/TILE_SIZE, odometer, leftMotor, rightMotor); //travel to takes integer coordinates as doubles 
					Navigation.travelTo(Project.LLx, Project.LLy, odometer, leftMotor, rightMotor);
					
					Sound.beep();
				}
			}).start();
	
		}
		
		//stop the system when the exit button is pressed
	    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
	    System.exit(0);
	}

}
