package ca.mcgill.ecse211.dpmProject;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;

/**
 * This class is used for localizing the robot using the ultrasonic sensor and the 2 line detection sensors
 * @author team12
 */

public class Localizer {

	private static final Port portLeftLine = Project.portLeftLine;
	private static final SensorModes myLeftLine = Project.myLeftLine;
	private static final SampleProvider myLeftLineSample = Project.myLeftLineSample;
	private static final float[] sampleLeftLine = Project.sampleLeftLine;
	
	private static final Port portRightLine = Project.portRightLine;
	private static final SensorModes myRightLine = Project.myRightLine;
	private static final SampleProvider myRightLineSample = Project.myRightLineSample;
	private static final float[] sampleRightLine = Project.sampleRightLine;
	
	private static final Port portColor = Project.portColor; // get the port for the light (color sensor)
	private static final SensorModes myColor = Project.myColor; // create the color sensor object
	private static final SampleProvider myColorSample = Project.myColorSample; //set to RGB mode
	private static final float[] sampleColor = Project.sampleColor; // create an array for the sensor readings
	
	public static final Port usPort = Project.usPort ;
	@SuppressWarnings("resource") // Because we don't bother to close this resource
	public static SensorModes usSensor = Project.usSensor; // usSensor is the instance
	public static SampleProvider usDistance = Project.usDistance; // usDistance provides samples from
	public static final float[] usData = Project.usData; //create an array for the sensor readings
	
	private static final EV3LargeRegulatedMotor leftMotor = Project.leftMotor; //the motor for the left wheel
	private static final EV3LargeRegulatedMotor rightMotor = Project.rightMotor; //the motor for the right wheel
	private static final EV3LargeRegulatedMotor armMotor = Project.armMotor; //the motor for raising/lowering the arm 
	private static final EV3MediumRegulatedMotor hookMotor =Project.hookMotor; //the motor for motorizing the hooks 
	
	private static final double OFF_SET = Project.OFF_SET;
	private static final double TILE_SIZE = Project.TILE_SIZE;
	private static final int THRESHOLD = Project.THRESHOLD;
	private static final double DISTANCE = Project.DISTANCE;
	private static final double WHEEL_RAD = Project.WHEEL_RAD; 
	private static final double TRACK = Project.TRACK;  
	private static final int LOW_SPEED = Project.LOW_SPEED; //this is the slow speed for precise movement 
	private static final int MEDIUM_SPEED = Project.MEDIUM_SPEED; //this is the medium speed for intermediate movement
	private static final int HIGH_SPEED = Project.HIGH_SPEED; //this is the fast motor speed for less precious, faster movement (long distance travel)
	
	/**
	 * This is the falling edge method used when the robots starts facing away from the wall (distance larger than D) 
	 * It will first turn clockwise to detect the back wall (angle: alpha)
	 * then counter-clockwise to detect the left wall (angle: beta)
	 * @param usDistance the sensor data sampler
	 * @param usData the sensor data buffer
	 * @param odometer the odometer used to determine the robots orientation
	 * @param leftMotor the left motor of the robot
	 * @param rightMotor the right motor of the robot
	 */
	static void fallingEdge(Odometer odometer) {
		int distance;
		usDistance.fetchSample(usData, 0);
		distance = (int) (usData[0] * 100.0); 
		
		//check is it is indeed facing away from the wall
		if (distance > DISTANCE) {
		    double alpha =0; //the angle when the back wall is detected
		    double beta =0; //the angle when the left wall is detected
		    
		    //turn clockwise to find the back wall and the corresponding angle alpha
	        //reset the motor
		  	for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		  	      motor.stop();
			      motor.setAcceleration(3000);
	  		    }
	  		// Sleep for 1 second
	  	    try {
		  	      Thread.sleep(500);
		    } catch (InterruptedException e) {      // There is nothing to be done here
		  	}    
	  	    leftMotor.setSpeed(HIGH_SPEED); 
	  	    rightMotor.setSpeed(HIGH_SPEED);
	  	    
	  	    //detect alpha
	  	    boolean back_detected = false;
	  	    while(back_detected == false) {
			    leftMotor.forward();
			    rightMotor.backward();
	  	    	
	  	    	usDistance.fetchSample(usData, 0);
	  		    distance = (int) (usData[0] * 100.0); 		    
	  		    if(distance <= DISTANCE) {
	  		    	alpha = odometer.getXYT()[2];
	  		    	back_detected = true;
	  		    }
	  	    }	  
	  	    
	  	    //turn the robot counter-clockwise out of the alpha detection zone
	  	    leftMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, 45), true);
	  	    rightMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, 45), false);
	  	    
	  	    //turn counter-clockwise to find the left wall and the corresponding angle beta
	  	    //reset the motor
		  	for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		  	      motor.stop();
			      motor.setAcceleration(3000);
	  		    }
	  		// Sleep for 1 second
	  	    try {
		  	      Thread.sleep(500);
		    } catch (InterruptedException e) {     
		  	}  
	  	    leftMotor.setSpeed(HIGH_SPEED); 
	  	    rightMotor.setSpeed(HIGH_SPEED);
	  	    
	  	    //detect beta
	  	    boolean left_detected = false;
	  	    while(left_detected == false) {
			    leftMotor.backward();
			    rightMotor.forward();
			    
	  	    	usDistance.fetchSample(usData, 0);
	  		    distance = (int) (usData[0] * 100.0); 		    
	  		    if(distance <= DISTANCE) {
	  		    	beta = odometer.getXYT()[2];
	  		    	left_detected = true;
	  		    }
	  		 
	  	    }	  
	  	    
	  	    //move the sensor away from the beta detection zone
	  	    leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, 45), true);
	  	    rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, 45), false);
	  	    
	  	    //reset the motor
		  	for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		  	      motor.stop();
			      motor.setAcceleration(3000);
	  		    }
	  		// Sleep for 1 second
	  	    try {
		  	      Thread.sleep(500);
		    } catch (InterruptedException e) {      
		  	}	  	    
	  	    leftMotor.setSpeed(HIGH_SPEED); 
	  	    rightMotor.setSpeed(HIGH_SPEED);
	  	    
	  	    //calculate the change in angle and then turn to the adjusted orientation
	  	    //delta is the angle of the real 0 axis when we use initial orientation as 0 axis 
	  	    double delta = (alpha+beta)/2 -219; 
	  	    if (delta<0) delta = 360+delta;
	  	    //turn to the real zero axis
	  	    leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, (delta-odometer.getXYT()[2])), true);
		    rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, (delta-odometer.getXYT()[2])), false);	    
		    
		    //correct the odometer orientation to zero
		    odometer.setTheta(0);
		}
		//invoke rising edge if it is facing the wall; use rising edge instead
		else {
			risingEdge(odometer);
		}
		
		

	}
	

	/**
	 * This is the rising edge method used when the robots starts facing the wall (distance smaller than D); 
	 * It will first turn clockwise to detect the left wall (angle: beta)
	 * then counter-clockwise to detect the back wall (angle: alpha)
	 * @param usDistance the sensor data sampler
	 * @param usData the sensor data buffer
	 * @param odometer the odometer used to determine the robots orientation
	 * @param leftMotor the left motor of the robot
	 * @param rightMotor the right motor of the robot
	 */	
	static void risingEdge(Odometer odometer) {
	    int distance;    
	    usDistance.fetchSample(usData, 0);
		distance = (int) (usData[0] * 100.0); 	
		
		//check if indeed is facing the wall, only then use rising edge
        if(distance < DISTANCE) {
    	    double alpha =0; //the angle when the back wall is detected
    	    double beta =0; //the angle when the left wall is detected
    	    
    	    //turn clockwise to find the left wall and the corresponding angle beta
            //reset the motor
    	  	for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
    	  	      motor.stop();
    		      motor.setAcceleration(3000);
      		    }
      		// Sleep for 1 second
      	    try {
    	  	      Thread.sleep(500);
    	    } catch (InterruptedException e) {      // There is nothing to be done here
    	  	}
    	    leftMotor.setSpeed(HIGH_SPEED); 
      	    rightMotor.setSpeed(HIGH_SPEED);
      	    
      	    //detect beta
      	    boolean left_detected = false;
      	    while(left_detected == false) {
    		    leftMotor.forward();
    		    rightMotor.backward();
      	    	
      	    	usDistance.fetchSample(usData, 0);
      		    distance = (int) (usData[0] * 100.0); 		    
      		    if(distance >= DISTANCE) {
      		    	System.out.println("detect 1st rising");
      		    	beta = odometer.getXYT()[2];
      		    	left_detected = true;
      		    }
      	    }	  

      	    
      	    //turn the robot counter-clockwise out of the beta detection zone
      	    leftMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, 45), true);
      	    rightMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, 45), false);
      	    
      	    //turn counter-clockwise to find the back wall and the corresponding angle alpha
      	    //reset the motor
    	  	for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
    	  	      motor.stop();
    		      motor.setAcceleration(3000);
      		    }
      		// Sleep for 1 second
      	    try {
    	  	      Thread.sleep(500);
    	    } catch (InterruptedException e) {      
    	  	}	    
      	    leftMotor.setSpeed(HIGH_SPEED); 
      	    rightMotor.setSpeed(HIGH_SPEED);
    	    
      	    //detect alpha
      	    boolean back_detected = false;
      	    while(back_detected == false) {
    		    leftMotor.backward();
    		    rightMotor.forward();
    		    
      	    	usDistance.fetchSample(usData, 0);
      		    distance = (int) (usData[0] * 100.0); 		    
      		    if(distance >= DISTANCE) {
      		    	System.out.println("detect 2nd rising");
      		    	alpha = odometer.getXYT()[2];
      		    	back_detected = true;
      		    }
      		 
      	    }	  
      	    
      	    //turn the robot counter-clockwise out of the beta detection zone
      	    leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, 45), true);
      	    rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, 45), false);
      	    
      	    //reset the motor
    	  	for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
    	  	      motor.stop();
    		      motor.setAcceleration(3000);
      		    }
      		// Sleep for 1 second
      	    try {
    	  	      Thread.sleep(500);
    	    } catch (InterruptedException e) {      
    	  	}	    	    
      	    leftMotor.setSpeed(HIGH_SPEED); 
      	    rightMotor.setSpeed(HIGH_SPEED);
      	    
      	    //calculate the change in angle and then turn to the adjusted orientation
      	    //delta is the angle of the real 0 axis in the system where the original heading was the zero axis.
      	    double delta = (alpha+beta)/2 -41;
      	    if(delta <0) delta = 360+delta;
      	    //turn to the real 0 axis
      	    leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, (delta-odometer.getXYT()[2])), true);
    	    rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, (delta-odometer.getXYT()[2])), false);
    	    
    	    //correct the odometer orientation to zero
    	    odometer.setTheta(0);

        }
        //if the robot is actually facing away from the wall rather than facing the wall, it will correct itself and use fallingEdge instead
		else {
			fallingEdge(odometer);
		}

	}
	
	
	public static void ultrasonicLite() {
		int distance;
		usDistance.fetchSample(usData, 0);
		distance = (int) (usData[0] * 100.0); 
		
		//rotate counter clockwise until no wall is found (can be optimized with a check to see whether the turn should be clockwise
		//or counter clockwise depending on initial distacne 
		while (distance < 255) {
			
			for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		  	      motor.stop();
			      motor.setAcceleration(3000);
	  		    }
			
			leftMotor.setSpeed(MEDIUM_SPEED); 
	  	    rightMotor.setSpeed(MEDIUM_SPEED);
	  	    
	  	    leftMotor.backward();
		    rightMotor.forward();
			
		    usDistance.fetchSample(usData, 0);
  		    distance = (int) (usData[0] * 100.0); 		    
		    
		}
		
		//once no wall is found continue to turn counter clockwise until the wall is found
		//this garentees that we are facing the back left wall
		while (distance > DISTANCE) {
			
			for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		  	      motor.stop();
			      motor.setAcceleration(3000);
	  		    }
			
			leftMotor.setSpeed(MEDIUM_SPEED); 
	  	    rightMotor.setSpeed(MEDIUM_SPEED);
	  	    
	  	    leftMotor.backward();
		    rightMotor.forward();
			
		    usDistance.fetchSample(usData, 0);
  		    distance = (int) (usData[0] * 100.0); 		    
		    
		}
		
		//hard coded value "50" to be determined by test
		leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, 50), true);
  	    rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, 50), false);
	}
	
	public static void lightLocalizeLite(Odometer odometer) {
		leftMotor.forward();
		rightMotor.forward();

		boolean leftDetected = false;
		boolean rightDetected = false;

		int localizationCount = 0;

		int leftTurn = 1;
		int rightTurn = -1;

		while (localizationCount != 2) {

			while (leftDetected && rightDetected) {

				leftMotor.forward();
				rightMotor.forward();

				myLeftLine.fetchSample(sampleLeftLine, 0); // get the reading from the sensor
				float leftRead = sampleLeftLine[0] * 1000; // multiply the read by 1000 as suggested in the class slides
				if (leftRead < THRESHOLD) {
					leftDetected = true;
					leftMotor.stop();
				}
				myLeftLine.fetchSample(sampleLeftLine, 0); // get the reading from the sensor
				float rightRead = sampleLeftLine[0] * 1000; // multiply the read by 1000 as suggested in the class
															// slides
				if (rightRead < THRESHOLD) {
					rightDetected = true;
					rightMotor.stop();
				}

			}

			leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, OFF_SET), true);
			rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, OFF_SET), false);

			leftMotor.stop(true);
			rightMotor.stop(false);

			leftMotor.rotate(leftTurn * Navigation.convertAngle(WHEEL_RAD, TRACK, 90), true);
			rightMotor.rotate(rightTurn * Navigation.convertAngle(WHEEL_RAD, TRACK, 90), false);

			leftMotor.stop(true);
			rightMotor.stop(false);

			localizationCount++;

			leftDetected = false;
			rightDetected = false;

			leftTurn = -1;
			rightTurn = 1;

			odometer.setXYT(TILE_SIZE, TILE_SIZE, 0);

		}
	}
	
}
