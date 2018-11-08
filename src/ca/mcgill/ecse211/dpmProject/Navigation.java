package ca.mcgill.ecse211.dpmProject;

import ca.mcgill.ecse211.dpmProject.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * this class is used for navigating the robot to a specific point on the grid (coordinates in cm)
 * @author Team12
 *
 */
public class Navigation {
	
	  private static final int FORWARD_SPEED = Project.HIGH_SPEED; 
	  private static final int ROTATE_SPEED = Project.MEDIUM_SPEED;
	  private static final double WHEEL_RAD = Project.WHEEL_RAD;
	  private static final double TRACK = Project.TRACK;
	  private static final double TILE_SIZE = Project.TILE_SIZE;
	  private static final double OFF_SET = Project.OFF_SET;
	  
	  private static final Port portLeftLine = Project.portLeftLine;
	  private static final SensorModes myLeftLine = Project.myLeftLine;
	  private static final SampleProvider myLeftLineSample = Project.myLeftLineSample;
	  private static final float[] sampleLeftLine = Project.sampleLeftLine;
		
	  private static final Port portRightLine = Project.portRightLine;
	  private static final SensorModes myRightLine = Project.myRightLine;
	  private static final SampleProvider myRightLineSample = Project.myRightLineSample;
	  private static final float[] sampleRightLine = Project.sampleRightLine;
	  
	private static final EV3LargeRegulatedMotor leftMotor = Project.leftMotor; //the motor for the left wheel
	private static final EV3LargeRegulatedMotor rightMotor = Project.rightMotor; //the motor for the right wheel
		
		
	/**
	 * This method is used to drive the robot to the destination point which is
	 * marked as an absolute coordinate (X, Y) 
	 * <P>
	 * The method constantly calls the turnTo method to first adjust to the angle
	 * it needs to turn to before moving
	 * <P>
	 * It will stop before reaching the destination to leave enough space for an odometer correction
	 * @param x the absolute x-coordinate of the destination, an integer value in double format
	 * @param y the absolute y-coordinate of the destination, an integer value in double format
	 * @param odometer the odometer object created in the main class

	 */	  	  
	public static void travelTo(double x, double y, Odometer odometer) {
		
		//get the odometer readings to determine the action
		double currentX = odometer.getXYT()[0]; //get the current x position in cm
		double currentY = odometer.getXYT()[1]; //get the current y position in cm
		double currentT = odometer.getXYT()[2]; //get the current direction in degrees
		
		//calculate the moving distance and turning angle
		double x1 = x*TILE_SIZE +15 ; //waypoint x coordinate in cm
		double y1 = y*TILE_SIZE -15 ; //waypoint y coordinate in cm
		double dDistance = Math.sqrt(Math.pow((x1 - currentX), 2) + Math.pow((y1 - currentY), 2));
		
		double dAngle = getDAngle(x1, y1, currentX, currentY);
		
		// reset the motor
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(3000);
		}
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
		}
		
		turnTo(dAngle, currentT); //turn the robot to the direction of the new way point
		
		//move the robot towards the new way point
		//reset the motor
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		      motor.setAcceleration(3000);
		    }
	    try {
	      Thread.sleep(500);
	    } catch (InterruptedException e) {
	    }

	    leftMotor.setSpeed(FORWARD_SPEED);
	    rightMotor.setSpeed(FORWARD_SPEED);

	    leftMotor.rotate(convertDistance(WHEEL_RAD, dDistance), true);
	    rightMotor.rotate(convertDistance(WHEEL_RAD, dDistance), false);
	    
	    double correctAngle = Math.toDegrees(Math.atan(-(y1-currentY)/(x1-currentX)));
	    
		leftMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, correctAngle), true);
		rightMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, correctAngle), false);
	    
	    leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		      motor.setAcceleration(3000);
		}
	    try {
	      Thread.sleep(500);
	    } catch (InterruptedException e) {
	    }
	    
	    leftMotor.setSpeed(75);
		rightMotor.setSpeed(75);
		boolean left = false;
		boolean right = false;
		leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, 90), true);
		rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, 90), false);
		
		while (left == false && right == false) {
			leftMotor.forward();
			rightMotor.forward();
			if (Localizer.lineDetection() ==3) {
				leftMotor.stop();
				rightMotor.stop();
				break;
			}
			else if (Localizer.lineDetection()==1) {
				leftMotor.stop();
				left = true;
				//break;
		
			}
			else if (Localizer.lineDetection()==2) {
				rightMotor.stop();
				right = true;
				//break;
			}
		}
		

		leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, OFF_SET), true);
		rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, OFF_SET), false);
		
		leftMotor.stop();
		rightMotor.stop();
		
		leftMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, 90), true);
		rightMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, 90), false);
		
		left = false;
		right = false;
		
		while (left == false && right == false) {
			leftMotor.forward();
			rightMotor.forward();
			if (Localizer.lineDetection() ==3) {
				leftMotor.stop();
				rightMotor.stop();
				break;
			}
			
			else if (Localizer.lineDetection()==1) {
				leftMotor.stop();
				left = true;
				//break;
			}
			else if (Localizer.lineDetection()==2) {
				rightMotor.stop();
				right = true;
				//break;
			}
		}
		

		leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, OFF_SET), true);
		rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, OFF_SET), false);
		
		odometer.setXYT(x*TILE_SIZE, y*TILE_SIZE, 0);
		
	}
	
	/**
	 * This method returns the angle the robot needs to turn (the smallest angle) to the next point
	 * @param dAngle the angle to turn towards
	 * @param currentT the current direction
	 */
	public static void turnTo (double dAngle, double currentT) {
		//reset the motor
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		      motor.setAcceleration(2000);
		    }
	    try {
	      Thread.sleep(1000);
	    } catch (InterruptedException e) {
	      // There is nothing to be done here
	    }
	    
	    //find the smallest angle to turn to the destination
	    double angle1 = dAngle - currentT;
	    double angle2 = (angle1>=0 ? -(360-(Math.abs(angle1))) : (360-(Math.abs(angle1))));
	    double angle = (Math.abs(angle1) < Math.abs(angle2) ? angle1 : angle2); 
	    
	    //start the motors and make the turn
	    leftMotor.setSpeed(ROTATE_SPEED);
	    rightMotor.setSpeed(ROTATE_SPEED);

	    leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, angle), true);
	    rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, angle), false);
	    
	}
	
	/**
	 * This method returns the smallest angle needed to turn
	 * @param x1 the x of way point
	 * @param y1 the y of way point
	 * @param xc the current x
	 * @param yc the current y
	 * @return the angle to turn in degrees
	 */
	public static double getDAngle(double x1, double y1, double xc, double yc) {
		double xr = x1 - xc;
		double yr = y1 - yc;
		
		//make the angle within 0 to 360
		if(xr == 0 && yr!=0) {
			if(yr>0) return 0; 
			else return 180;
		}
		if(xr != 0 && yr==0) {
			if(xr>0) return 90;
			else return 270;
		}
		if(xr != 0 && yr!=0) {
			if(xr>0 && yr>0) return Math.toDegrees(Math.atan(xr/yr));
			if(xr>0 && yr<0) return Math.toDegrees(Math.atan(Math.abs(yr/xr)))+90;
			if(xr<0 && yr<0) return Math.toDegrees(Math.atan(xr/yr))+180;
			if(xr<0 && yr>0) return Math.toDegrees(Math.atan(Math.abs(yr/xr)))+270;;
		}
		return 0;
	}
	
	/**
	 * converts distance to angle the wheel needs to turn in deg
	 * @param radius of the robot
	 * @param distance of the robot
	 * @return deg to turn
	 */
	public static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
	  }
	
	/**
	 * converts angle to the actual angle needs to turn 
	 * @param radius radius of the robot 
	 * @param width radius of the car
	 * @param angle to turn
	 * @return angle that need to turn in deg
	 */
	public static int convertAngle(double radius, double width, double angle) {
	    return convertDistance(radius, Math.PI * width * angle / 360.0);
	    
	    
	  }
	
	/**
	 * This methods is used for traveling through the tunnel 
	 * @param x the absolute x-coordinate of the destination, an integer value in double format
	 * @param y the absolute y-coordinate of the destination, an integer value in double format
	 * @param odometer The odometer used by the robot
	 */
	public static void tunnelTravel(double x, double y, Odometer odometer) {
		
		double currentX = odometer.getXYT()[0]; //get the current x position in cm
		double currentY = odometer.getXYT()[1]; //get the current y position in cm
		double currentT = odometer.getXYT()[2]; //get the current direction in degrees
		
		leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, 90),true);
  	    rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, 90),false);	
		
  	    leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE/2 + OFF_SET),true);
  	  	rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE/2 + OFF_SET),false);
  	  	
  	  	leftMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, 90),true);
	    rightMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, 90),false);
	    
	    boolean leftDetected = false;
  	    boolean rightDetected = false;
	    
	    while (leftDetected && rightDetected) {
  	    	
			leftMotor.backward();
			rightMotor.backward();
  	    	
  	    	myLeftLine.fetchSample(sampleLeftLine, 0); //get the reading from the sensor
		    float leftRead = sampleLeftLine[0]*1000;  //multiply the read by 1000 as suggested in the class slides
//			if (leftRead<THRESHOLD) {
//				leftDetected = true;
//				leftMotor.stop();
//			}
//			myLeftLine.fetchSample(sampleLeftLine, 0); //get the reading from the sensor
//		    float rightRead = sampleLeftLine[0]*1000;  //multiply the read by 1000 as suggested in the class slides
//			if (rightRead<THRESHOLD) {
//				rightDetected = true;
//				rightMotor.stop();
//			}
  	    	
  	    }
	    
	    leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, 2.5*TILE_SIZE),true);
  	  	rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, 2.5*TILE_SIZE),false);
  	  	
  	  	int localizationCount = 0;
  	  	
  	  	int leftTurn = 1;
	    int rightTurn = -1;
  	  			
  	  	while (localizationCount != 2) {
    	    
	  	    while (leftDetected && rightDetected) {
	  	    	
				leftMotor.forward();
				rightMotor.forward();
	  	    	
	  	    	myLeftLine.fetchSample(sampleLeftLine, 0); //get the reading from the sensor
			    float leftRead = sampleLeftLine[0]*1000;  //multiply the read by 1000 as suggested in the class slides
//				if (leftRead<THRESHOLD) {
//					leftDetected = true;
//					leftMotor.stop();
//				}
//				myLeftLine.fetchSample(sampleLeftLine, 0); //get the reading from the sensor
//			    float rightRead = sampleLeftLine[0]*1000;  //multiply the read by 1000 as suggested in the class slides
//				if (rightRead<THRESHOLD) {
//					rightDetected = true;
//					rightMotor.stop();
//				}
	  	    	
	  	    }
	  	    
	  	    leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, OFF_SET),true);
	  	  	rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, OFF_SET),false);
	  	  	
	  	  	leftMotor.stop(true);
	  	    rightMotor.stop(false);
	  	    
	  	    leftMotor.rotate(leftTurn * Navigation.convertAngle(WHEEL_RAD, TRACK, 90),true);
	  	    rightMotor.rotate(rightTurn * Navigation.convertAngle(WHEEL_RAD, TRACK, 90),false);
	  	    
	  	    leftMotor.stop(true);
	  	    rightMotor.stop(false);
	  	    
	  	    localizationCount++;
	  	    
	  	    leftDetected = false;
	  	    rightDetected = false;
	  	    
	  	    leftTurn = -1;
	  	    rightTurn = 1;
	    
	    }
	    
  	  odometer.setXYT(currentX + TILE_SIZE*1, currentY + TILE_SIZE*3, 0);
		
	}
	
	
	public void intersectionCorrection(Odometer odometer) {
		
		
	}
	
	public void lineCorrection(Odometer odometer) {
		
		
	}
 
}
