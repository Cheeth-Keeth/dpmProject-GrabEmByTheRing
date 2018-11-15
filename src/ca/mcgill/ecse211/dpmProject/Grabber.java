package ca.mcgill.ecse211.dpmProject;

import java.awt.List;
import java.util.ArrayList;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

/**
 * This class contains methods for the ring fetching routine and mechanism
 * 
 * @author Team12
 *
 */
public class Grabber {
	private static final EV3LargeRegulatedMotor leftMotor = Project.leftMotor; // the motor for the left wheel
	private static final EV3LargeRegulatedMotor rightMotor = Project.rightMotor; // the motor for the right wheel
	private static final EV3LargeRegulatedMotor armMotor = Project.armMotor; // the motor for raising/lowering the
																					// arm
	private static final EV3MediumRegulatedMotor hookMotor = Project.hookMotor; // the motor for motorizing the
																						// hooks
	private static final int ARM_SPEED = Project.ARM_SPEED; // this is the speed for the arm for the arm motor
	private static final int HOOK_SPEED = Project.HOOK_SPEED; // this is the angle which the hook will open/close
	private static final int HOOK_ANGLE = Project.HOOK_ANGLE; // this is the angle which the hook will open/close
	private static final int LOW_ANGLE = Project.LOW_ANGLE; // the angle the arm motor needs to turn to reach
																	// lowly-hanged rings, with respect to the initial
																	// position
	private static final int HIGH_ANGLE = Project.HIGH_ANGLE; // the angle the arm motor needs to turn to reach
																	// highly-hanged rings, with respect to the initial
																	// position
	private static final int UNLOAD_ANGLE = Project.UNLOAD_ANGLE; // the angle the arm motor needs to turn to
																		// unload the ring(s), with respect to the
																		// initial position

	private static final int FORWARD_SPEED = Project.HIGH_SPEED;
	private static final int ROTATE_SPEED = Project.MEDIUM_SPEED;
	private static final double WHEEL_RAD = Project.WHEEL_RAD;
	private static final double TRACK = Project.TRACK;
	private static final double TILE_SIZE = Project.TILE_SIZE;
	private static final double OFF_SET = Project.OFF_SET;
	private static final double HIGH_PROBE = Project.HIGH_PROBE;
	private static final double LOW_PROBE = Project.LOW_PROBE;
	
	private static final double T_x = Project.T_x;
	private static final double T_y = Project.T_y;
	
	public static boolean FOUND = Color.FOUND;
	
	/**
	 * This method is used to travel to the tree after coming out of the tunnel
	 * <p>
	 * It will always go the nearest closest side of the three, as each side is defined by first intersection the branches on that side are facing
	 * @param odometer the odometer used by the robot
	 */
	public static void travelToTree(Odometer odometer) {
		
		double[] odometerData = odometer.getXYT();
		double x = odometerData[0];
		double y = odometerData[1];
		double t;

		int point;

		double X0 = T_x;
		double Y0 = T_y - 1;

		double X1 = T_x + 1;
		double Y1 = T_y;

		double X2 = T_x;
		double Y2 = T_y + 1;

		double X3 = T_x - 1;
		double Y3 = T_y;

		int color;
	
		point = Navigation.closestPoint(X0, Y0, X1, Y1, X2, Y2, X3, Y3, x, y);
		
		if (point == 0) {
			
			Navigation.travelTo(X0, Y0, odometer);

		} else if (point == 1) {

			Navigation.travelTo(X1, Y1, odometer);

		} else if (point == 2) {

			Navigation.travelTo(X2, Y2, odometer);

		} else if (point == 3) {

			Navigation.travelTo(X3, Y3, odometer);

		}
		
		probe(odometer, point);
		
		if (!FOUND) {
		
			findRoute(point, odometer);
		
		}
	}
	
	/**
	 * This method is used for probing the rings once the robot arrives at a side of the tree
	 * @param odometer the odometer used by the robot
	 * @param point the intersection corresponding to the side the robot is probing
	 */
	public static void probe(Odometer odometer, int point) {
		
			double[] odometerData = odometer.getXYT();
			double x = odometerData[0];
			double y = odometerData[1];
			double t;
			
			int color;
		

			odometerData = odometer.getXYT();
			x = odometerData[0];
			y = odometerData[1];
			t = odometerData[2];

			//double dAngle = Navigation_Test.getDAngle(x, y, T_x, T_y);
			double treeOrientation = 0;
			if (point == 1) treeOrientation = 270;
			if (point == 2) treeOrientation = 180;
			if (point == 3) treeOrientation = 90;
			double angle = Navigation.smallAngle(t, treeOrientation);

			leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, angle), true);
			rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, angle), false);
			
			leftMotor.rotate(-Navigation.convertDistance(WHEEL_RAD, 5), true);
			rightMotor.rotate(-Navigation.convertDistance(WHEEL_RAD, 5), false);
			
			Navigation.lineCorrection(odometer);
			
			color = Grabber.highLevel();
			Navigation.lineCorrection(odometer);
			if (color == 0) {
				color = Grabber.lowLevel();
				Navigation.lineCorrection(odometer);
			}

		}


	/**
	 * This method is used for turning the arm to fetch the rings on the upper level
	 * of the tree
	 */
	public static int lowLevel() {
		armMotor.setAcceleration(15000);
		armMotor.setSpeed(ARM_SPEED);
		armMotor.rotate(LOW_ANGLE);
		//move forward.////////////////////
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, LOW_PROBE), true);
		rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, LOW_PROBE), false);
	//	Navigation_Test.lineCorrection();
		leftMotor.stop(true);
		rightMotor.stop(false);
		///////////////////////////////////
		int color = Color.color();

		if (color == 1 || color == 2 || color == 3 || color == 4) { 	// high level fetching
			if (color == 1) {
				Sound.beep();
				openHook();
			} else if (color == 2) {
				Sound.beep();
				Sound.beep();
				openHook();
			} else if (color == 3) {
				Sound.beep();
				Sound.beep();
				Sound.beep();
				openHook();
			} else {
				Sound.beep();
				Sound.beep();
				Sound.beep();
				Sound.beep();
			}

		}
		
		//move backward /////////////////////////
		leftMotor.setSpeed(150);
		rightMotor.setSpeed(150);
		leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, -LOW_PROBE - 5), true);
		rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, -LOW_PROBE - 5), false);
		leftMotor.stop(true);
		rightMotor.stop(false);
		///////////////////////////////////////
		armMotor.setAcceleration(3000);
		resetArm();
		
		return color;
		
	}

	/**
	 * This method is used for turning the arm to fetch the rings on the lower level
	 * of the tree
	 */
	public static int highLevel() {
		armMotor.setSpeed(ARM_SPEED);
		armMotor.rotate(HIGH_ANGLE);
		//move forward.////////////////////
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, HIGH_PROBE), true);
		rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, HIGH_PROBE), false);
		leftMotor.stop(true);
		rightMotor.stop(false);
		///////////////////////////////////
		int color = Color.color();

		if (color == 1 || color == 2 || color == 3 || color == 4) { 	// high level fetching
			if (color == 1) {
				Sound.beep();
				openHook();
			} else if (color == 2) {
				Sound.beep();
				Sound.beep();
				openHook();
			} else if (color == 3) {
				Sound.beep();
				Sound.beep();
				Sound.beep();
				openHook();
				
			} else {
				Sound.beep();
				Sound.beep();
				Sound.beep();
				Sound.beep();
				
			}

		}

		
		//move backward /////////////////////////
		leftMotor.setSpeed(150);
		rightMotor.setSpeed(150);
		leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, -HIGH_PROBE -5), true);
		rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, -HIGH_PROBE -5), false);
		leftMotor.stop(true);
		rightMotor.stop(false);
		/////////////////////////////////////////
		resetArm();
		System.out.println("end highLevel");
		return color;
		
	}

	/**
	 * This method is used for opening the a the hook to fetch the ring
	 */
	public static void openHook() {
		hookMotor.rotate(HOOK_ANGLE);
		leftMotor.rotate(-Navigation.convertDistance(WHEEL_RAD, 15), true);
		rightMotor.rotate(-Navigation.convertDistance(WHEEL_RAD, 15), false);
		
		while(true) {
			
			
			
		}
		
	}

	/**
	 * This method is used for closing the hook to either probe though hole or drop
	 * the rings
	 */
	public static void closeHook() {
		hookMotor.rotate(-HOOK_ANGLE);
	}

	/**
	 * this method is used to unload the ring using the arm motor
	 */
	public static void unload() {
		armMotor.setSpeed(ARM_SPEED);
		armMotor.rotate(UNLOAD_ANGLE);
	}

	/**
	 * this method is used to reset the arm to the initial position (falling on the
	 * back support)
	 */
	public static void resetArm() {
		armMotor.setSpeed(ARM_SPEED);
		while (armMotor.getTachoCount() != 0) {
			armMotor.backward();
		}
		armMotor.resetTachoCount();
	}
	
	/**
	 * This method defines and determines the further probing route, after probing at the closed side of the tree from the tunnel
	 * <p>
	 * The route needs to be determined for specific maps as the avaliability of the sides (branches) varies based on the position of the tree
	 * @param point the intersection the robot is currenting on, corresponding to the side of the tree it is at
	 * @param odometer the odometer used by the robot 
	 */
	public static void findRoute(int point, Odometer odometer) {

		double T_x = Project.T_x; //x coordinate of the ring tree
		double T_y = Project.T_y; //y coordinate of the ring tree
		double Island_LL_y = Project.Island_LL_y;
		double Island_LL_x = Project.Island_LL_x;
		double Island_UR_y = Project.Island_UR_y;
		double Island_UR_x = Project.Island_UR_x;
		
		int nextPoint1 = (point + 1)%4; 
		int nextPoint2 = (point + 2)%4;
		int nextPoint3 = (point + 3)%4;
		
		boolean found;
		
		boolean[] availability = {(T_y != 1 && T_y != Island_LL_y), (T_x != 7 && T_x !=Island_UR_x), (T_y != 7 && T_y != Island_UR_y) , (T_x != 1 && T_x != Island_LL_x)};
		
		
		if (!availability[nextPoint1] && !availability[nextPoint2] && !availability[nextPoint3]) {
			
			
			
		} else if (availability[nextPoint1] && availability[nextPoint2] && availability[nextPoint3]) {
			
			
			treeTravel(point, nextPoint1, odometer);
			probe(odometer, nextPoint1);
			 
			if (!FOUND) {
				treeTravel(nextPoint1, nextPoint2, odometer);
				probe(odometer, nextPoint2);
				
				if (!FOUND) {
				
					treeTravel(nextPoint2, nextPoint3, odometer);
					probe(odometer, nextPoint3);
				
				}
			}
			
		} else if (availability[nextPoint1] && !availability[nextPoint2] && availability[nextPoint3]) {
			
			treeTravel(point, nextPoint1, odometer);
			probe(odometer, nextPoint1);
			
			if (!FOUND) {
			
				treeTravel(nextPoint1, nextPoint3, odometer);
				probe(odometer, nextPoint3);
			}
			
		} else if (availability[nextPoint1] && availability[nextPoint2] && !availability[nextPoint3]) {
			
			treeTravel(point, nextPoint1, odometer);
			probe(odometer, nextPoint1);
			
			if (!FOUND) {
			
				treeTravel(nextPoint1, nextPoint2, odometer);
				probe(odometer, nextPoint2);
			}
			
		} else if (availability[nextPoint1] && !availability[nextPoint2] && !availability[nextPoint3]) {
			
			treeTravel(point, nextPoint1, odometer);
			probe(odometer, nextPoint1);
			
		} else if (!availability[nextPoint1] && availability[nextPoint2] && availability[nextPoint3]) {
			
			treeTravel(point, nextPoint3, odometer);
			probe(odometer, nextPoint3);
			
			if (!FOUND) {
			
				treeTravel(nextPoint3, nextPoint2, odometer);
				probe(odometer, nextPoint2);
			}
				
		} else if (!availability[nextPoint1] && !availability[nextPoint2] && availability[nextPoint3]) {
			
			treeTravel(point, nextPoint3, odometer);
			probe(odometer, nextPoint3);
			
		}
		
		
	}
	
	/**
	 * This method defines the routine for traveling between the sides, after the avalability of the side is determined 
	 * @param startPoint the side traveling from
	 * @param endPoint the side traveling to 
	 * @param odometer the odometer used by the robot
	 */
	public static void treeTravel(int startPoint, int endPoint, Odometer odometer) {
		
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
		}
		
		leftMotor.setSpeed(125);
		rightMotor.setSpeed(125);
		
		int direction;
		
		if (endPoint == (startPoint + 1)%4 || endPoint == (startPoint - 1)%4) {
			
			if (endPoint == (startPoint + 1)%4) {
				
				direction = -1;
				
			} else {
				
				direction = 1;
				
			}
			
			leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, -direction*90), true);
			rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, -direction*90), false);
			leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE - 8), true);
			rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE -8), false);
			Navigation.lineCorrection(odometer);
			leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, direction*90), true);
			rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, direction*90), false);
			leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE - 8), true);
			rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE -8), false);
			Navigation.lineCorrection(odometer);
			
			
		} else if (endPoint == (startPoint + 2)%4 || endPoint == (startPoint - 2)%4) {
			
			if (endPoint == (startPoint + 2)%4) {
				
				direction = -1;
				
			} else {
				
				direction = 1;
				
			}
			
			leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, direction*90), true);
			rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, direction*90), false);
			leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE - 8), true);
			rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE -8), false);
			Navigation.lineCorrection(odometer);
			leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, -direction*90), true);
			rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, -direction*90), false);
			leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE - 8), true);
			rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE -8), false);
			Navigation.lineCorrection(odometer);
			leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE - 8), true);
			rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE -8), false);
			Navigation.lineCorrection(odometer);
			leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, -direction*90), true);
			rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, -direction*90), false);
			leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE - 8), true);
			rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE -8), false);
			Navigation.lineCorrection(odometer);
			
		}
		
		
	}
	

}