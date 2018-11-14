package testsyallyeeha;

import testsyallyeeha.Odometer_Test;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * this class is used for navigating the robot to a specific point on the grid
 * (coordinates in cm)
 * 
 * @author Team12
 *
 */
public class Navigation_Test {

	private static final EV3LargeRegulatedMotor leftMotor = Project_Test.leftMotor; // the motor for the left wheel
	private static final EV3LargeRegulatedMotor rightMotor = Project_Test.rightMotor; // the motor for the right wheel
	private static final EV3LargeRegulatedMotor armMotor = Project_Test.armMotor; // the motor for raising/lowering the
																					// // arm
	private static final EV3MediumRegulatedMotor hookMotor = Project_Test.hookMotor; // the motor for motorizing the
																						// hooks

	private static final int FORWARD_SPEED = Project_Test.HIGH_SPEED;
	private static final int ROTATE_SPEED = Project_Test.MEDIUM_SPEED;
	private static final int CORRECT_SPEED = Project_Test.LOW_SPEED;
	private static final double WHEEL_RAD = Project_Test.WHEEL_RAD;
	private static final double TRACK = Project_Test.TRACK;
	private static final double TILE_SIZE = Project_Test.TILE_SIZE;
	private static final double OFF_SET = Project_Test.OFF_SET;

	private static final Port portLine = Project_Test.portLeftLine;
	private static final SensorModes myLeftLine = Project_Test.myLeftLine;
	private static final SampleProvider myLeftLineSample = Project_Test.myLeftLineSample;
	private static final float[] sampleLeftLine = Project_Test.sampleLeftLine;

	private static final Port portRightLine = Project_Test.portRightLine;
	private static final SensorModes myRightLine = Project_Test.myRightLine;
	private static final SampleProvider myRightLineSample = Project_Test.myRightLineSample;
	private static final float[] sampleRightLine = Project_Test.sampleRightLine;

	private static final double Island_LL_x = Project_Test.Island_LL_x; // x coordinate of the lower left corner of the
																		// island
	private static final double Island_LL_y = Project_Test.Island_LL_y; // y coordinate of the lower left corner of the
																		// island
	private static final double Island_UR_x = Project_Test.Island_UR_x; // x coordinate of the upper right corner of the
																		// island
	private static final double Island_UR_y = Project_Test.Island_UR_y; // y coordinate of the upper right corner of the
																		// island

	private static final int corner = Project_Test.corner; // the starting corner
	private static final double LL_x = Project_Test.LL_x; // x coordinate of the lower left corner of the home section
	private static final double LL_y = Project_Test.LL_y; // y coordinate of the lower left corner of the home section
	private static final double UR_x = Project_Test.UR_x; // x coordinate of the upper right corner of the home section
	private static final double UR_y = Project_Test.UR_y; // y coordinate of the upper right corner of the home section
	private static final double TN_LL_x = Project_Test.TN_LL_x; // x coordinate of the lower left of the tunnel
	private static final double TN_LL_y = Project_Test.TN_LL_y; // y coordinate of the lower left of the tunnel
	private static final double TN_UR_x = Project_Test.TN_UR_x; // x coordinate of the upper right of the tunnel
	private static final double TN_UR_y = Project_Test.TN_UR_y; // y coordinate of the upper right of the tunnel
	private static final double T_x = Project_Test.T_x; // x coordinate of the ring tree
	private static final double T_y = Project_Test.T_y; // y coordinate of the ring tree

	private static double currentX;
	private static double currentY;
	private static double currentT;

	/**
	 * This method is used to drive the robot to the destination point which is
	 * marked as an absolute coordinate (X, Y) The method constantly calls the
	 * turnTo method to first adjust to the angle it needs to turn to before moving
	 * 
	 * @param x        the absolute x-coordinate of the destination, an integer
	 *                 value in double format
	 * @param y        the absolute y-coordinate of the destination, an integer
	 *                 value in double format
	 * @param odometer the odometer object created in the main class
	 */
	public static void travelTo(double x, double y, Odometer_Test odometer) {

		// get the odometer readings to determine the action
		currentX = odometer.getXYT()[0]; // get the current x position in cm
		currentY = odometer.getXYT()[1]; // get the current y position in cm
		currentT = odometer.getXYT()[2]; // get the current direction in degrees
		boolean alongLine = Math.abs(x * TILE_SIZE - currentX) < 5 || Math.abs(y * TILE_SIZE - currentY) < 5;
		if (alongLine == true) {
			double x1 = x * TILE_SIZE;
			double y1 = y * TILE_SIZE;
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
			System.out.println("dAngle is "+dAngle);
			turnTo(dAngle, currentT); // turn the robot to the direction of the new way point

			// move the robot towards the new way point
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

			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);
			
//			double travelTimes = Math.floor(dDistance/TILE_SIZE);
//			
//			lineCorrection(odometer);
//			
//			for (int i = 0; i < travelTimes/TILE_SIZE; i++) {
//				
//				leftMotor.rotate(convertDistance(WHEEL_RAD,TILE_SIZE - 5), true);
//				rightMotor.rotate(convertDistance(WHEEL_RAD, TILE_SIZE - 5), false);
//				
//				lineCorrection(odometer);
//				
//			}

			leftMotor.rotate(convertDistance(WHEEL_RAD, dDistance - 6), true);
			rightMotor.rotate(convertDistance(WHEEL_RAD, dDistance - 6), false);
			
			lineCorrection(odometer);
			
			///////
			System.out.println("x is "+x+",  y is "+y);
			//////
		} else {
			// four points
			double X0 = x - 0.5; // waypoint x coordinate in cm
			double Y0 = y - 0.5; // waypoint y coordinate in cm
			double X1 = x + 0.5; // waypoint x coordinate in cm
			double Y1 = y - 0.5; // waypoint y coordinate in cm
			double X2 = x + 0.5; // waypoint x coordinate in cm
			double Y2 = y + 0.5; // waypoint y coordinate in cm
			double X3 = x - 0.5; // waypoint x coordinate in cm
			double Y3 = y + 0.5; // waypoint y coordinate in cm

			double x1 = 0;
			double y1 = 0;
			int point = closestPoint(X0, Y0, X1, Y1, X2, Y2, X3, Y3, currentX, currentY);

			if (point == 0) {
				x1 = X0 * TILE_SIZE;
				y1 = Y0 * TILE_SIZE;
			}
			if (point == 1) {
				x1 = X1 * TILE_SIZE;
				y1 = Y1 * TILE_SIZE;
			}
			if (point == 2) {
				x1 = X2 * TILE_SIZE;
				y1 = Y2 * TILE_SIZE;
			}
			if (point == 3) {
				x1 = X3 * TILE_SIZE;
				y1 = Y3 * TILE_SIZE;
			}

			// calculate the moving distance and turning angle
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

			turnTo(dAngle, currentT); // turn the robot to the direction of the new way point

			// move the robot towards the new way point
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

			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);

			leftMotor.rotate(convertDistance(WHEEL_RAD, dDistance), true);
			rightMotor.rotate(convertDistance(WHEEL_RAD, dDistance), false);

			intersectionCorrection(odometer);
			odometer.setX(x * TILE_SIZE);
			odometer.setY(y * TILE_SIZE);
		}

	}

	/**
	 * This method returns the angle the robot needs to turn (the smallest angle) to
	 * the next point
	 * 
	 * @param dAngle     the angle to turn towards
	 * @param currentT   the current direction
	 * @param leftMotor  the left motor
	 * @param rightMotor the right motor
	 */
	public static void turnTo(double dAngle, double currentT) {
		// reset the motor
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(2000);
		}
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// There is nothing to be done here
		}

		// find the smallest angle to turn to the destination
		double angle1 = dAngle - currentT;
		double angle2 = (angle1 >= 0 ? -(360 - (Math.abs(angle1))) : (360 - (Math.abs(angle1))));
		double angle = (Math.abs(angle1) < Math.abs(angle2) ? angle1 : angle2);

		// start the motors and make the turn
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, angle), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, angle), false);

	}

	/**
	 * This method returns the smallest angle needed to turn to the next waypoint
	 * 
	 * @param x1 the x of way point
	 * @param y1 the y of way point
	 * @param xc the current x
	 * @param yc the current y
	 * @return the angle to turn in degrees
	 */
	public static double getDAngle(double x1, double y1, double xc, double yc) {
		double xr = x1 - xc;
		double yr = y1 - yc;

		// make the angle within 0 to 360
		if (xr == 0 && yr != 0) {
			if (yr > 0)
				return 0;
			else
				return 180;
		}
		if (xr != 0 && yr == 0) {
			if (xr > 0)
				return 90;
			else
				return 270;
		}
		if (xr != 0 && yr != 0) {
			if (xr > 0 && yr > 0)
				return Math.toDegrees(Math.atan(xr / yr));
			if (xr > 0 && yr < 0)
				return Math.toDegrees(Math.atan(Math.abs(yr / xr))) + 90;
			if (xr < 0 && yr < 0)
				return Math.toDegrees(Math.atan(xr / yr)) + 180;
			if (xr < 0 && yr > 0)
				return Math.toDegrees(Math.atan(Math.abs(yr / xr))) + 270;
			;
		}
		return 0;
	}

	/**
	 * converts distance to angle the wheel needs to turn in deg
	 * 
	 * @param radius   of the robot
	 * @param distance of the robot
	 * @return deg to turn
	 */
	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * converts angle to the actual angle needs to turn
	 * 
	 * @param radius radius of the robot
	 * @param width  radius of the car
	 * @param angle  to turn
	 * @return angle that need to turn in deg
	 */
	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	public static void tunnelTravel(Odometer_Test odometer) {
		boolean isTunnelVertical;
		if ((corner == 0 || corner == 3) && TN_UR_x > UR_x)
			isTunnelVertical = false;
		else if ((corner == 1 || corner == 2) && TN_LL_x < LL_x)
			isTunnelVertical = false;
		else
			isTunnelVertical = true;

		double pointT0_x, pointT0_y, pointT1_x, pointT1_y, pointT2_x, pointT2_y, pointT3_x, pointT3_y;
		double tunnelLength;
		System.out.println(isTunnelVertical);
		System.out.println("TR is x "+Project_Test.T_x);
		System.out.println("TR is y"+Project_Test.T_y);
		if (isTunnelVertical) {
			tunnelLength = Math.abs(TN_LL_y - TN_UR_y);

			pointT0_x = TN_LL_x;
			pointT0_y = TN_LL_y - 1;

			pointT1_x = TN_LL_x + 1;
			pointT1_y = TN_LL_y - 1;

			pointT2_x = TN_UR_x;
			pointT2_y = TN_UR_y + 1;

			pointT3_x = TN_UR_x - 1;
			pointT3_y = TN_UR_y + 1;

		} else {
			tunnelLength = Math.abs(TN_LL_x - TN_UR_x);

			pointT0_x = TN_LL_x - 1;
			pointT0_y = TN_LL_y;

			pointT1_x = TN_UR_x + 1;
			pointT1_y = TN_UR_y - 1;

			pointT2_x = TN_UR_x + 1;
			pointT2_y = TN_UR_y;

			pointT3_x = TN_LL_x - 1;
			pointT3_y = TN_LL_y + 1;

		}
		currentX = odometer.getXYT()[0];
		currentY = odometer.getXYT()[1];
		currentT = odometer.getXYT()[2];
		int pointT = closestPoint(pointT0_x, pointT0_y, pointT1_x, pointT1_y, pointT2_x, pointT2_y, pointT3_x,
				pointT3_y, currentX, currentY);

		if (pointT == 0) {
			Navigation_Test.travelTo(pointT0_x, pointT0_y, odometer);
			if (isTunnelVertical) {
				currentT = odometer.getXYT()[2];
				double turn = smallAngle(currentT, 90);
				leftMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, turn), true);
				rightMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, turn), false);
			} else {
				currentT = odometer.getXYT()[2];
				double turn = smallAngle(currentT, 0);
				leftMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, turn), true);
				rightMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, turn), false);
			}
		} else if (pointT == 1) {
			Navigation_Test.travelTo(pointT1_x, pointT1_y, odometer);
			if (isTunnelVertical) {
				currentT = odometer.getXYT()[2];
				double turn = smallAngle(currentT, 270);
				leftMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, turn), true);
				rightMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, turn), false);
			} else {
				currentT = odometer.getXYT()[2];
				double turn = smallAngle(currentT, 0);
				leftMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, turn), true);
				rightMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, turn), false);
			}
		} else if (pointT == 2) {
			Navigation_Test.travelTo(pointT2_x, pointT2_y, odometer);
			if (isTunnelVertical) {
				currentT = odometer.getXYT()[2];
				double turn = smallAngle(currentT, 270);
				leftMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, turn), true);
				rightMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, turn), false);
			} else {
				currentT = odometer.getXYT()[2];
				double turn = smallAngle(currentT, 180);
				leftMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, turn), true);
				rightMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, turn), false);
			}
		} else {
			Navigation_Test.travelTo(pointT3_x, pointT3_y, odometer);
			if (isTunnelVertical) {
				currentT = odometer.getXYT()[2];
				double turn = smallAngle(currentT, 90);
				leftMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, turn), true);
				rightMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, turn), false);
			} else {
				currentT = odometer.getXYT()[2];
				double turn = smallAngle(currentT, 180);
				leftMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, turn), true);
				rightMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, turn), false);
			}
		}
		
		currentT = odometer.getXYT()[2];
		if ((currentT <= 100 && currentT>= 80) || (currentT >= 170 && currentT<=190 ) ) {
		
		leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, TILE_SIZE / 2 - 2), true);
		rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, TILE_SIZE / 2 - 2), false);
		}
		
		if ((currentT <= 10 || currentT>= 350) || (currentT >= 260 && currentT<=280 )) {
			leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, TILE_SIZE / 2 + 2), true);
			rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, TILE_SIZE / 2 + 2), false);
		}
		
		leftMotor.stop(true);
		rightMotor.stop(false);

		currentX = odometer.getXYT()[0];
		currentY = odometer.getXYT()[1];
		currentT = odometer.getXYT()[2];

		if (isTunnelVertical) {
			if (pointT == 0) {
				leftMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 90), true);
				rightMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 90), false);
			}

			else if (pointT == 1) {
				leftMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 90), true);
				rightMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 90), false);
			}

			else if (pointT == 2) {
				leftMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 90), true);
				rightMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 90), false);
			} else {
				leftMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 90), true);
				rightMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 90), false);
			}
		} else {
			if (pointT == 0) {
				leftMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 90), true);
				rightMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 90), false);
			}

			else if (pointT == 1) {
				leftMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 90), true);
				rightMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 90), false);
			}

			else if (pointT == 2) {
				leftMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 90), true);
				rightMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 90), false);
			} else {
				leftMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 90), true);
				rightMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 90), false);
			}
		}

		leftMotor.rotate(-Navigation_Test.convertDistance(WHEEL_RAD, 10), true);
		rightMotor.rotate(-Navigation_Test.convertDistance(WHEEL_RAD, 10), false);

		lineCorrection(odometer);

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, (0.8) * TILE_SIZE), true);
		rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, (0.8) * TILE_SIZE), false);

		lineCorrection(odometer);

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, (tunnelLength + 0.8) * TILE_SIZE), true);
		rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, (tunnelLength + 0.8) * TILE_SIZE), false);

		lineCorrection(odometer);

//		leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, (0.8)*TILE_SIZE), true);
//		rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, (0.8)*TILE_SIZE), false);
//		
//		lineCorrection(odometer);

	}

	public static void intersectionCorrection(Odometer_Test odometer) {
		leftMotor.stop(true);
		rightMotor.stop(false);
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
		}
		
		leftMotor.setSpeed(CORRECT_SPEED);
		rightMotor.setSpeed(CORRECT_SPEED);
		double fromOrientation = odometer.getXYT()[2];
		double toOrientation;
		if (fromOrientation >= 0 && fromOrientation < 90)
			toOrientation = 90;
		else if (fromOrientation >= 90 && fromOrientation < 180)
			toOrientation = 180;
		else if (fromOrientation >= 180 && fromOrientation < 270)
			toOrientation = 270;
		else
			toOrientation = 0;
		leftMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, smallAngle(fromOrientation, toOrientation)),
				true);
		rightMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, smallAngle(fromOrientation, toOrientation)),
				false);

		adjustment(odometer);
		Sound.beep();
		leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, OFF_SET), true);
		rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, OFF_SET), false);

		leftMotor.stop();
		rightMotor.stop();

		leftMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 90), true);
		rightMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 90), false);

		adjustment(odometer);
		Sound.beep();
		leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, OFF_SET), true);
		rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, OFF_SET), false);
		leftMotor.stop();
		rightMotor.stop();
		double currentT = odometer.getXYT()[2];
		if (currentT >= 45 && currentT < 135)
			odometer.setTheta(90);
		else if (currentT >= 135 && currentT < 225)
			odometer.setTheta(180);
		else if (currentT >= 225 && currentT < 315)
			odometer.setTheta(270);
		else
			odometer.setTheta(0);
	}

	public static void lineCorrection(Odometer_Test odometer) {
				
		leftMotor.stop(true);
		rightMotor.stop(false);
		
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
		}
		
		leftMotor.setSpeed(CORRECT_SPEED);
		rightMotor.setSpeed(CORRECT_SPEED);

		adjustment(odometer);

		Sound.beep();
		leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, OFF_SET), true);
		rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, OFF_SET), false);

		leftMotor.stop(true);
		rightMotor.stop(false);
		double currentT = odometer.getXYT()[2];
		if (currentT >= 45 && currentT < 135)
			odometer.setTheta(90);
		else if (currentT >= 135 && currentT < 225)
			odometer.setTheta(180);
		else if (currentT >= 225 && currentT < 315)
			odometer.setTheta(270);
		else
			odometer.setTheta(0);
		
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
		}

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
	}

	/**
	 * This method is used for fast calculation of the smallest angle required to
	 * turn to turn from one orientation to another
	 * <P>
	 * This method differs from the getDAngle() and turnTo() methods in that it
	 * turns to arbitrary orientations rather than to a specific waypoint point
	 * 
	 * @param fromOrientation the current orientation
	 * @param toOrientation   the orientation to orient to
	 * @return
	 */
	public static double smallAngle(double fromOrientation, double toOrientation) {
		double angle1 = toOrientation - fromOrientation;
		double angle2 = (angle1 >= 0 ? -(360 - (Math.abs(angle1))) : (360 - (Math.abs(angle1))));
		return (Math.abs(angle1) < Math.abs(angle2) ? angle1 : angle2);
	}

	public static int closestPoint(double X0, double Y0, double X1, double Y1, double X2, double Y2, double X3,
			double Y3, double x, double y) {

		double[] distance = new double[4];

		distance[0] = Math.sqrt(Math.pow((X0 * TILE_SIZE - x), 2) + Math.pow((Y0 * TILE_SIZE - y), 2));
		distance[1] = Math.sqrt(Math.pow((X1 * TILE_SIZE - x), 2) + Math.pow((Y1 * TILE_SIZE - y), 2));
		distance[2] = Math.sqrt(Math.pow((X2 * TILE_SIZE - x), 2) + Math.pow((Y2 * TILE_SIZE - y), 2));
		distance[3] = Math.sqrt(Math.pow((X3 * TILE_SIZE - x), 2) + Math.pow((Y3 * TILE_SIZE - y), 2));

		int point = 0;
		for (int i = 1; i < 4; i++) {
			if (distance[point] > distance[i]) {
				point = i;
			}
		}
		return point;

	}

	public static void adjustment(Odometer_Test odometer) {
		boolean left = false;
		boolean right = false;
		boolean firstStop = true;
//		boolean leftMissed = false;
//		boolean rightMissed = false;
//		double firstT = odometer.getXYT()[2];
//		double secondT;
//		double delT;
		
		
		
///////		Ethan's
//		leftMotor.forward();
//		rightMotor.forward();
//		while (!(left && right)) {
//			int lineStatus = Localizer_Test.lineDetection();
//			if(lineStatus == 3) {
//				leftMotor.stop(true);
//				rightMotor.stop(false);
//				left = true;
//				right = true;
//			} else if(lineStatus == 1) {
//				
//				leftMotor.stop();
//				left = true;
//				
//			} else if (lineStatus == 2) {
//				
//				rightMotor.stop();
//				right = true;
//				
//			} 
//			
//		}
		
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
		}
		
		
		while (left == false || right == false) {
			int lineStatus = Localizer_Test.lineDetection();
			if (lineStatus == 3) {
				// System.out.print("return 3");
				if (left == false) {
					if (firstStop) {
						leftMotor.stop(true);
						firstStop = false;
					} else {
						leftMotor.stop(false);
					}
				}
				if (right == false) {
					if (firstStop) {
						rightMotor.stop(true);
					} else {
						rightMotor.stop(false);
					}
				}
				break;
			} else if (lineStatus == 1) {
				if (left == false) {
					if (firstStop) {
						leftMotor.stop(true);
						firstStop = false;
					} else {
						leftMotor.stop(true);
//						secondT = odometer.getXYT()[2]; //
//					delT = (firstT>=secondT? firstT-secondT : firstT-secondT +360); //4
//					System.out.println(delT);
//					if(delT>=45 ) { //
//						rightMissed = true;//
//						break; //
//					}//
						// what is the logoc here

					}
					left = true;
				}

			} else if (lineStatus == 2) { // right detected
				if (right == false) {
					if (firstStop) {
						rightMotor.stop(true);
						firstStop = false;
					} else {
						rightMotor.stop(true);
//						secondT = odometer.getXYT()[2];
//						delT = (secondT >= firstT ? secondT - firstT : secondT - firstT + 360);
//						if (delT >= 45) {
//							leftMissed = true;
//							break;
//						}
					}
					right = true;
				} 
			}
			else {
					if (left == false) {
						leftMotor.forward();
					}
					if (right == false) {
						rightMotor.forward();
					}
				}

//				if (leftMissed == true) {
//					rightMotor.stop(true);
//					leftMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 90), false);
//				} else if (rightMissed == true) {
//					leftMotor.stop(true);
//					rightMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 90), false);
//				}
			}

		

//		boolean left = false;
//		boolean right = false;
//		boolean firstStop = true;
//		leftMotor.forward();
//		rightMotor.forward();
//		while (left == false || right == false) {
//			int lineStatus = Localizer_Test.lineDetection();
//			if (lineStatus == 3) {
//				if (firstStop == true) {
//					leftMotor.stop(true);
//					rightMotor.stop(false);
//					firstStop = false;
//				} else if (left == true) {
//					rightMotor.stop(false);
//					right = true;
//				} else if (right == true) {
//					leftMotor.stop(false);
//					left = true;
//				}
//			} else if (lineStatus == 1) {
//				if (firstStop == true) {
//					leftMotor.stop(true);
//					left = true;
//					firstStop = false;
//				} else {
//					if(right == false) {
//						leftMotor.stop(true);
//						rightMotor.forward();					
//					}
//				}
//			} else if (lineStatus == 2) {
//				if (firstStop == true) {
//					rightMotor.stop(true);
//					right = true;
//					firstStop = false;
//				} else {
//					if(left ==false) {
//						rightMotor.stop(true);
//						leftMotor.forward();
//					}
//				}

//		}

	}

}
