package ca.mcgill.ecse211.dpmProject;

import lejos.hardware.sensor.*;

import java.util.Map;
import java.util.concurrent.TimeUnit;

import ca.mcgill.ecse211.WiFiClient.WiFi;
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
 * This is the main class for project SCARAB; 
 * It contains almost all the parameters/sensor initiations, as well as the parameters needed for the operation 
 * It will interact with the user through the EV3 module's display and buttons
 * The routines are performed as consecutive methods within threads
 * @author Team12
 *
 */

public class Project {
	//The parameters for driving the robot
	//The island parameters
	public static double Island_LL_x = 0; //x coordinate of the lower left corner of the island
	public static double Island_LL_y = 5; //y coordinate of the lower left corner of the island
	public static double Island_UR_x = 6; //x coordinate of the upper right corner of the island
	public static double Island_UR_y = 8; //y coordinate of the upper right corner of the island
	
	//The team-specific parameters
	public static int corner = 1; //the starting corner
	public static double LL_x = 2; //x coordinate of the lower left corner of the home section
	public static double LL_y = 0; //y coordinate of the lower left corner of the home section
	public static double UR_x = 8; //x coordinate of the upper right corner of the home section
	public static double UR_y = 3; //y coordinate of the upper right corner of the home section
	public static double TN_LL_x = 2; //x coordinate of the lower left of the tunnel
	public static double TN_LL_y = 3; //y coordinate of the lower left of the tunnel
	public static double TN_UR_x = 3; //x coordinate of the upper right of the tunnel
	public static double TN_UR_y = 5; //y coordinate of the upper right of the tunnel
	public static double T_x = 5; //x coordinate of the ring tree
	public static double T_y = 7; //y coordinate of the ring tree
	
	//The operating parameters for arm and hook (grabbing mechanism)
	public static final int HOOK_SPEED = 10; //this is the speed used for the motor 
	public static final int ARM_SPEED = 60; //this is the speed for the arm for the arm motor 
	public static final int HOOK_ANGLE = 50; //this is the angle which the hook will open/close	
	public static final int LOW_ANGLE = 160; //the angle the arm motor needs to turn to reach lowly-hanged rings, with respect to the initial position 
	public static final int HIGH_ANGLE = 40; //the angle the arm motor needs to turn to reach highly-hanged rings, with respect to the initial position 
	public static final int UNLOAD_ANGLE = 110; //the angle the arm motor needs to turn to unload the ring(s), with respect to the initial position 
	
	//The operating parameters for the navigation and diver system
	public static final double OFF_SET = 2.5; //this is the offset from the 2 line-detecting light sensors to the wheel base
	public static final int LOW_SPEED = 65; //this is the slow speed for precise movement 
	public static final int MEDIUM_SPEED = 100; //this is the medium speed for intermediate movement
	public static final int HIGH_SPEED = 150; //this is the fast motor speed for less precious, faster movement (long distance travel)
	public static final double WHEEL_RAD = 2.085; //the wheel radius of the wheels
	public static final double TRACK = 14.42; //the wheel base of the robot
	public static final double TILE_SIZE = 30.48; //the tile length of the grid
	public static final double HIGH_PROBE = 14;
	public static final double LOW_PROBE = 4.5;
	public static final int DISTANCE = 45; //distance from the wall used by the ultrasonic sensor during the ultrasonic localization 
	public static final int THRESHOLD = 300; //this is the line detection threshold for the line detecting color sensors
	
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
	public static final float[] usData = new float[usDistance.sampleSize()]; //create an array for the sensor readings
	
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
	 * This is the main method for the project
	 * <p>
	 * It will take the parameters from the lab server, and initiates them to be used by the robot
	 * <p>
	 * Then it will start the odometer thread, and execute the full project routine in another thread as a sequence of methods
	 * @param args
	 * @throws OdometerExceptions
	 */
	public static void main(String[] args) throws OdometerExceptions{
		Map data = WiFi.Wifi();
		
		int redTeam = ((Long) data.get("RedTeam")).intValue();
		int greenTeam = ((Long) data.get("GreenTeam")).intValue();
		
		if (redTeam == 12) {
			
			corner = ((Long) data.get("RedCorner")).intValue(); //the starting corner
			LL_x = ((Long) data.get("Red_LL_x")).intValue(); //x coordinate of the lower left corner of the home section
			LL_y = ((Long) data.get("Red_LL_y")).intValue(); //y coordinate of the lower left corner of the home section
			UR_x = ((Long) data.get("Red_UR_x")).intValue(); //x coordinate of the upper right corner of the home section
			UR_y = ((Long) data.get("Red_UR_y")).intValue(); //y coordinate of the upper right corner of the home section
			TN_LL_x = ((Long) data.get("TNR_LL_x")).intValue(); //x coordinate of the lower left of the tunnel
			TN_LL_y = ((Long) data.get("TNR_LL_y")).intValue(); //y coordinate of the lower left of the tunnel
			TN_UR_x = ((Long) data.get("TNR_UR_x")).intValue(); //x coordinate of the upper right of the tunnel
			TN_UR_y = ((Long) data.get("TNR_UR_y")).intValue(); //y coordinate of the upper right of the tunnel
			T_x = ((Long) data.get("TR_x")).intValue(); //x coordinate of the ring tree
			T_y = ((Long) data.get("TR_y")).intValue(); //y coordinate of the ring tree
			
		} else {
			
			corner = ((Long) data.get("GreenCorner")).intValue(); //the starting corner
			LL_x = ((Long) data.get("Green_LL_x")).intValue(); //x coordinate of the lower left corner of the home section
			LL_y = ((Long) data.get("Green_LL_y")).intValue(); //y coordinate of the lower left corner of the home section
			UR_x = ((Long) data.get("Green_UR_x")).intValue(); //x coordinate of the upper right corner of the home section
			UR_y = ((Long) data.get("Green_UR_y")).intValue(); //y coordinate of the upper right corner of the home section
			TN_LL_x = ((Long) data.get("TNG_LL_x")).intValue(); //x coordinate of the lower left of the tunnel
			TN_LL_y = ((Long) data.get("TNG_LL_y")).intValue(); //y coordinate of the lower left of the tunnel
			TN_UR_x = ((Long) data.get("TNG_UR_x")).intValue(); //x coordinate of the upper right of the tunnel
			TN_UR_y = ((Long) data.get("TNG_UR_y")).intValue(); //y coordinate of the upper right of the tunnel
			T_x = ((Long) data.get("TG_x")).intValue(); //x coordinate of the ring tree
			T_y = ((Long) data.get("TG_y")).intValue(); //y coordinate of the ring tree	
			
		}
		
		System.out.println(TN_LL_x + TN_LL_y);
		
		Island_LL_x = ((Long) data.get("Island_LL_x")).intValue(); //x coordinate of the lower left corner of the island
		Island_LL_y = ((Long) data.get("Island_LL_y")).intValue(); //y coordinate of the lower left corner of the island
		Island_UR_x = ((Long) data.get("Island_UR_x")).intValue(); //x coordinate of the upper right corner of the island
		Island_UR_y = ((Long) data.get("Island_UR_y")).intValue(); //y coordinate of the upper right corner of the island
		
		final Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		int buttonChoice;
		
		System.out.println("corner" + corner);
		System.out.println("llx" + TN_LL_x);
		System.out.println("lly" + TN_LL_y);
		
		Odometer odomter;
		Thread odoThread = new Thread(odometer);
		odoThread.start();
		
		(new Thread() {
			public void run() {
				//add method : 
				Localizer.fallingEdge(odometer);
				Localizer.lightLocalizeLite(odometer);
				Sound.beep();
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				Navigation.tunnelTravel(odometer);
				System.out.println("x is  " + odometer.getXYT()[0] + "  y is " + odometer.getXYT()[1] + "  T is " +odometer.getXYT()[2]);
				Grabber.travelToTree(odometer);
			
			}
		}).start();

		
	}	
	
}
