package testsyallyeeha;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Testers_Test {
	public static final Port usPort = Project_Test.usPort;
	@SuppressWarnings("resource") // Because we don't bother to close this resource
	private static SensorModes usSensor = Project_Test.usSensor; // usSensor is the instance
	private static SampleProvider usDistance = Project_Test.usDistance; // usDistance provides samples from
	private static final float[] usData = Project_Test.usData;

	private static final Port portLeftLine = Project_Test.portLeftLine; // get the port for the left front light sensor
	private static final SensorModes myLeftLine = Project_Test.myLeftLine; // create the light sensor object
	private static final SampleProvider myLeftLineSample = Project_Test.myLeftLineSample; //set to Red mode
	private static final float[] sampleLeftLine = Project_Test.sampleLeftLine; // create an array for the sensor readings 
	
	private static final Port portRightLine = Project_Test.portRightLine; // get the port for the right front light sensor
	private static final SensorModes myRightLine = Project_Test.myRightLine; // create the light sensor object
	private static final SampleProvider myRightLineSample = Project_Test.myRightLineSample; //set to Red mode
	private static final float[] sampleRightLine = Project_Test.sampleRightLine; // create an array for the sensor readings 
	
	public static final Port portColor = Project_Test.portColor; // get the port for the light (color sensor)
	public static final SensorModes myColor = Project_Test.myColor; // create the color sensor object
	public static final SampleProvider myColorSample = Project_Test.myColorSample; //set to RGB mode
	public static final float[] sampleColor = Project_Test.sampleColor; // create an array for the sensor readings
	
	private static final EV3LargeRegulatedMotor leftMotor = Project_Test.leftMotor;
	private static final EV3LargeRegulatedMotor rightMotor = Project_Test.rightMotor;
	private static final EV3LargeRegulatedMotor armMotor = Project_Test.armMotor;
	private static final EV3MediumRegulatedMotor hookMotor = Project_Test.hookMotor;
	private static final TextLCD lcd = Project_Test.lcd;
	
	/**
	 * hook motor angle test
	 */
	public static void hookMotor() {
		hookMotor.rotate(45);
	}
	
	
	/**
	 * this method is used for rgb reading collection for mean and std calculations 
	 */
	public static void sample() {
		lcd.clear();
		int counter =0;
		//run 100 samples
		while(counter<100) {
			myColorSample.fetchSample(sampleColor, 0); 
			float r = sampleColor[0]*1000; 
			float g = sampleColor[1]*1000; 
			float b = sampleColor[2]*1000; 
			System.out.print(r +",");
			System.out.print(g+",");
			System.out.println(b);
			counter ++;
		}		
	}
	
	/**
	 * This method calibrates the wheel radius value of the robot
	 * since the method convertDistance only uses wheel radius
	 * @param leftMotor left motor of the robot
	 * @param rightMotor right motor of the robot 
	 */
	public static void wheelRadCheck() {
		// reset the motor
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(2000);
		}
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// There is nothing to be done here
		}
		//move the robot forward until the Y asis is detected
		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);
		leftMotor.rotate(Navigation_Test.convertDistance(Project_Test.WHEEL_RAD, 2*Project_Test.TILE_SIZE), true);
		rightMotor.rotate(Navigation_Test.convertDistance(Project_Test.WHEEL_RAD, 2*Project_Test.TILE_SIZE), false);
	}
	
	/**
	 * This method calibrates the wheelbase value of the robot once the 
	 * wheel radius is calibrated
	 * @param leftMotor left motor of the robot
	 * @param rightMotor right motor of the robot 
	 */
	public static void trackCheck() {
		// reset the motor
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(2000);
		}
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// There is nothing to be done here
		}
		
		//move the robot forward until the Y asis is detected
		leftMotor.setSpeed(150);
		rightMotor.setSpeed(150);
		leftMotor.rotate(Navigation_Test.convertAngle(Project_Test.WHEEL_RAD, Project_Test.TRACK, 360), true);
		rightMotor.rotate(-Navigation_Test.convertAngle(Project_Test.WHEEL_RAD, Project_Test.TRACK, 360), false);
	}
	
	
	
	
	public static void line() {
//		System.out.println("start line sampling");
//		int count = 0;
//		while (true) {
//			myLeftLineSample.fetchSample(sampleLeftLine, 0);
//			int reading = (int) (sampleLeftLine[0] * 1000.0);
//			System.out.println(reading);
//			count++;
//		}
	}
	
	public static void us() {		
//		System.out.println("start US sampling");
//		int count = 0;
//		while (true) {
//			usDistance.fetchSample(usData, 0);
//			int distance = (int) (usData[0] * 100.0);
//			System.out.println(distance);
//			count++;
//		}
	}
	
	// the following section is designed for making new testing routines
	/*
	 * for odometer testing 
	 * place the robot at (1,1), and facing at exactly zero degree
	 odometer.setXYT(TILE_SIZE, TILE_SIZE, 0);
	 Navigation_Test.travelTo(2,2);
	 System.out.println(odometer.getXYT()[0], odometer.getXYT()[1], odometer.getXYT()[2]);
	 Thread.sleep(15000);
	 Navigation_Test.travelTo(3,5);
	 System.out.println(odometer.getXYT()[0], odometer.getXYT()[1], odometer.getXYT()[2]);
	 Thread.sleep(15000);
	 Navigation_Test.travelTo(3.5, 5);
	 System.out.println(odometer.getXYT()[0], odometer.getXYT()[1], odometer.getXYT()[2]);
	 * 
	 */
	
	//add method: light localization accuracy test, repeat 3 times with 5 sec between reps
	/*Localizer_Test.lightLocalizeLite(odometer);
	try {
		Thread.sleep(5000);
	} catch (InterruptedException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}
	Localizer_Test.lightLocalizeLite(odometer);
	try {
		Thread.sleep(5000);
	} catch (InterruptedException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}
	Localizer_Test.lightLocalizeLite(odometer);
	*/
	
	
	//ultrasonicn test
	/*
	Localizer_Test.fallingEdge(odometer);
	try {
		Thread.sleep(5000);
	} catch (InterruptedException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}
	Localizer_Test.fallingEdge(odometer);
	try {
		Thread.sleep(5000);
	} catch (InterruptedException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}
	Localizer_Test.fallingEdge(odometer);
	*/
}
