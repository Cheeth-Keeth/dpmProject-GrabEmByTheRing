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
	private static final int PROBE_SPEED = Project.LOW_SPEED;
	private static final double WHEEL_RAD = Project.WHEEL_RAD;
	private static final double TRACK = Project.TRACK;
	private static final double TILE_SIZE = Project.TILE_SIZE;
	private static final double OFF_SET = Project.OFF_SET;
	private static final double HIGH_PROBE = Project.HIGH_PROBE;
	private static final double LOW_PROBE = Project.LOW_PROBE;

	private static final double T_x = Project.T_x;
	private static final double T_y = Project.T_y;
	private static final double TN_LL_x = Project.TN_LL_x; // x coordinate of the lower left of the tunnel
	private static final double TN_LL_y = Project.TN_LL_y; // y coordinate of the lower left of the tunnel
	private static final double TN_UR_x = Project.TN_UR_x; // x coordinate of the upper right of the tunnel
	private static final double TN_UR_y = Project.TN_UR_y; // y coordinate of the upper right of the tunnel

	public static int rings = 0;
	public static final int TOTALRING = 3;
	public static int currentPoint;
	public static boolean goHome = false;

	private static boolean blue = false;
	private static boolean orange = false;
	private static boolean green = false;
	private static boolean yellow = false;

	/**
	 * This method is used to travel to the tree after coming out of the tunnel
	 * <p>
	 * It will always go the nearest closest side of the three, as each side is
	 * defined by first intersection the branches on that side are facing
	 * 
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

		// beep three times after arriving at the point
		Sound.beep();
		Sound.beep();
		Sound.beep();

		openHook();
		probe(odometer, point);

		findRoute(point, odometer);

	}

	/**
	 * This method is used for probing the rings once the robot arrives at a side of
	 * the tree
	 * 
	 * @param odometer the odometer used by the robot
	 * @param point    the intersection corresponding to the side the robot is
	 *                 probing
	 */
	public static void probe(Odometer odometer, int point) {
		if (rings < TOTALRING) {
			double[] odometerData = odometer.getXYT();
			double x = odometerData[0];
			double y = odometerData[1];
			double t;

			int color;

			odometerData = odometer.getXYT();
			x = odometerData[0];
			y = odometerData[1];
			t = odometerData[2];

			double treeOrientation = 0;
			if (point == 1)
				treeOrientation = 270;
			if (point == 2)
				treeOrientation = 180;
			if (point == 3)
				treeOrientation = 90;
			double angle = Navigation.smallAngle(t, treeOrientation);

			// reset motor before rotating
			leftMotor.stop(true);
			rightMotor.stop(false);
			for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
				motor.setAcceleration(3000);
			}
			try {
				Thread.sleep(200);
			} catch (InterruptedException e) {
			}
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);

			leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, angle), true);
			rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, angle), false);

			leftMotor.rotate(-Navigation.convertDistance(WHEEL_RAD, 5), true);
			rightMotor.rotate(-Navigation.convertDistance(WHEEL_RAD, 5), false);

			Navigation.lineCorrection(odometer);
			if (rings < 2) {
				color = Grabber.lowLevel();
				Navigation.lineCorrection(odometer);
			}
			if (rings < TOTALRING) {
				color = Grabber.highLevel();
				Navigation.lineCorrection(odometer);
			}
		}
	}

	/**
	 * This method is used for turning the arm to fetch the rings on the upper level
	 * of the tree
	 */
	public static int lowLevel() {
		armMotor.setAcceleration(1000);
		armMotor.setSpeed(200);

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(-Navigation.convertDistance(WHEEL_RAD, 10), true);
		rightMotor.rotate(-Navigation.convertDistance(WHEEL_RAD, 10), false);
		armMotor.rotate(LOW_ANGLE);
		closeHook(); //////////////////////////////////////////////////////////////////////////////////
		// reset motor before rotating
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(3000);
		}
		try {
			Thread.sleep(200);
		} catch (InterruptedException e) {
		}
		leftMotor.setSpeed(PROBE_SPEED);
		rightMotor.setSpeed(PROBE_SPEED);
		leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, LOW_PROBE), true);
		rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, LOW_PROBE), false);
		///////////////////////////////////
		int color = Color.color();

		if (color == 1 || color == 2 || color == 3 || color == 4) {
			if (color == 1) {
				if (blue == false) {
					Sound.beep();
					rings++;
				}
				blue = true;

			} else if (color == 2) {
				if (green == false) {
					Sound.beep();
					Sound.beep();
					rings++;
				}
				green = true;
			} else if (color == 3) {
				if (yellow == false) {
					Sound.beep();
					Sound.beep();
					Sound.beep();
					rings++;
				}
				yellow = true;

			} else {
				if (orange == false) {
					Sound.beep();
					Sound.beep();
					Sound.beep();
					Sound.beep();
					rings++;
				}
				orange = true;
			}

		}
		openHook(); /////////////////////////////////////////////////////////////////
		// reset motor before rotating
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(3000);
		}
		try {
			Thread.sleep(200);
		} catch (InterruptedException e) {
		}
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, -LOW_PROBE), true);
		rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, -LOW_PROBE), false);

		resetArm();

		// reset motor before rotating
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(3000);
		}
		try {
			Thread.sleep(200);
		} catch (InterruptedException e) {
		}
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, 5), true);
		rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, 5), false);

		return color;

	}

	/**
	 * This method is used for turning the arm to fetch the rings on the lower level
	 * of the tree
	 */
	public static int highLevel() {

//		armMotor.setAcceleration(500);
//		armMotor.setSpeed(100);
//		armMotor.rotate(HIGH_ANGLE);

		closeHook();/////////////////////////////////////////////////////////////
		leftMotor.setSpeed(PROBE_SPEED);
		rightMotor.setSpeed(PROBE_SPEED);
		leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, HIGH_PROBE), true);
		rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, HIGH_PROBE), false);
		leftMotor.stop(true);
		rightMotor.stop(false);
		///////////////////////////////////
		int color = Color.color();

		if (color == 1 || color == 2 || color == 3 || color == 4) {
			if (color == 1) {
				if (blue == false) {
					Sound.beep();
					rings++;
				}
				blue = true;

			} else if (color == 2) {
				if (green == false) {
					Sound.beep();
					Sound.beep();
					rings++;
				}
				green = true;
			} else if (color == 3) {
				if (yellow == false) {
					Sound.beep();
					Sound.beep();
					Sound.beep();
					rings++;
				}
				yellow = true;

			} else {
				if (orange == false) {
					Sound.beep();
					Sound.beep();
					Sound.beep();
					Sound.beep();
					rings++;
				}
				orange = true;
			}

		}

		openHook(); /////////////////////////////
		
		// reset motor before rotating backward
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(3000);
		}
		try {
			Thread.sleep(200);
		} catch (InterruptedException e) {
		}
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, -HIGH_PROBE - 5), true);
		rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, -HIGH_PROBE - 5), false);
		/////////////////////////////////////////
		// resetArm();
		return color;

	}

	/**
	 * This method is used for opening the a the hook to fetch the ring
	 */
	public static void openHook() {
		hookMotor.setSpeed(80);
		hookMotor.rotate(HOOK_ANGLE);

	}

	/**
	 * This method is used for closing the hook to either probe though hole or drop
	 * the rings
	 */
	public static void closeHook() {
		hookMotor.setSpeed(30);
		hookMotor.rotate(-HOOK_ANGLE);
	}

	/**
	 * this method is used to unload the ring using the arm motor
	 */
	public static void unload() {

		// reset motor before rotating
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(3000);
		}
		try {
			Thread.sleep(200);
		} catch (InterruptedException e) {
		}
		leftMotor.setSpeed(FORWARD_SPEED + 100);
		rightMotor.setSpeed(FORWARD_SPEED + 100);
		closeHook();
		armMotor.setAcceleration(2000);
		armMotor.setSpeed(250);
		armMotor.rotate(110);
		leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, -5), true);
		rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, -5), false);
		// reset motor before rotating
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(5000);
		}
		try {
			Thread.sleep(50);
		} catch (InterruptedException e) {
		}
		leftMotor.setSpeed(FORWARD_SPEED + 200);
		rightMotor.setSpeed(FORWARD_SPEED + 200);
		leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, 0), true);
		rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, 10), false);
		leftMotor.stop(true);
		rightMotor.stop(false);
	}

	/**
	 * this method is used to reset the arm to the initial position (falling on the
	 * back support)
	 */
	public static void resetArm() {
		armMotor.setAcceleration(3000);
		armMotor.setSpeed(ARM_SPEED);
		while (armMotor.getTachoCount() != 0) {
			armMotor.backward();
		}
		armMotor.resetTachoCount();
	}

	/**
	 * This method defines and determines the further probing route, after probing
	 * at the closed side of the tree from the tunnel
	 * <p>
	 * The route needs to be determined for specific maps as the avaliability of the
	 * sides (branches) varies based on the position of the tree
	 * 
	 * @param point    the intersection the robot is currenting on, corresponding to
	 *                 the side of the tree it is at
	 * @param odometer the odometer used by the robot
	 */
	public static void findRoute(int point, Odometer odometer) {

		double T_x = Project.T_x; // x coordinate of the ring tree
		double T_y = Project.T_y; // y coordinate of the ring tree
		double Island_LL_y = Project.Island_LL_y;
		double Island_LL_x = Project.Island_LL_x;
		double Island_UR_y = Project.Island_UR_y;
		double Island_UR_x = Project.Island_UR_x;

		int nextPoint1 = (point + 1) % 4;
		int nextPoint2 = (point + 2) % 4;
		int nextPoint3 = (point + 3) % 4;

		boolean[] availability = {
				((T_y != 1 && T_y != Island_LL_y) && !(T_x == TN_LL_x && T_y - 1 == TN_LL_y)
						&& !(T_x == TN_LL_x && T_y - 1 == TN_UR_y) && !(T_x == TN_UR_x && T_y - 1 == TN_LL_y)
						&& !(T_x == TN_UR_x && T_y - 1 == TN_UR_y)),
				((T_x != 7 && T_x != Island_UR_x) && !(T_x + 1 == TN_LL_x && T_y == TN_LL_y)
						&& !(T_x + 1 == TN_LL_x && T_y == TN_UR_y) && !(T_x + 1 == TN_UR_x && T_y == TN_LL_y)
						&& !(T_x + 1 == TN_UR_x && T_y == TN_UR_y)),
				((T_y != 7 && T_y != Island_UR_y) && !(T_x == TN_LL_x && T_y + 1 == TN_LL_y)
						&& !(T_x == TN_LL_x && T_y + 1 == TN_UR_y) && !(T_x == TN_UR_x && T_y + 1 == TN_LL_y)
						&& !(T_x == TN_UR_x && T_y + 1 == TN_UR_y)),
				((T_x != 1 && T_x != Island_LL_x) && !(T_x - 1 == TN_LL_x && T_y == TN_LL_y)
						&& !(T_x - 1 == TN_LL_x && T_y == TN_UR_y) && !(T_x - 1 == TN_UR_x && T_y == TN_LL_y)
						&& !(T_x - 1 == TN_UR_x && T_y == TN_UR_y)) };

		if (!availability[nextPoint1] && !availability[nextPoint2] && !availability[nextPoint3]) {
			// do nothing
		} else if (availability[nextPoint1] && availability[nextPoint2] && availability[nextPoint3]) {
			treeTravel(point, nextPoint1, odometer);
			probe(odometer, nextPoint1);

			treeTravel(nextPoint1, nextPoint2, odometer);
			probe(odometer, nextPoint2);

			treeTravel(nextPoint2, nextPoint3, odometer);
			probe(odometer, nextPoint3);

			goHome = true;
			treeTravel(currentPoint, point, odometer);

		} else if (availability[nextPoint1] && !availability[nextPoint2] && availability[nextPoint3]) {

			treeTravel(point, nextPoint1, odometer);
			probe(odometer, nextPoint1);

			treeTravel(nextPoint1, nextPoint3, odometer);
			probe(odometer, nextPoint3);

			goHome = true;
			treeTravel(currentPoint, point, odometer);

		} else if (availability[nextPoint1] && availability[nextPoint2] && !availability[nextPoint3]) {

			System.out.println("point is" + point); ////////////////
			System.out.println("next point 1 is" + nextPoint1); ////////////////

			treeTravel(point, nextPoint1, odometer);
			probe(odometer, nextPoint1);

			System.out.println("current point is" + currentPoint); ////////////////
			System.out.println("Next point 1 is" + nextPoint1);/////////////////
			System.out.println("Next point 2 is" + nextPoint2);////////////////
			treeTravel(nextPoint1, nextPoint2, odometer);
			probe(odometer, nextPoint2);

			goHome = true;

			System.out.println("current point is" + currentPoint); ////////////////
			System.out.println("point is" + point); ////////////////

			treeTravel(currentPoint, point, odometer);

			System.out.println("current point is" + currentPoint); ////////////////

		} else if (availability[nextPoint1] && !availability[nextPoint2] && !availability[nextPoint3]) {

			treeTravel(point, nextPoint1, odometer);
			probe(odometer, nextPoint1);

			goHome = true;
			treeTravel(currentPoint, point, odometer);
		} else if (!availability[nextPoint1] && availability[nextPoint2] && availability[nextPoint3]) {

			treeTravel(point, nextPoint3, odometer);
			probe(odometer, nextPoint3);

			treeTravel(nextPoint3, nextPoint2, odometer);
			probe(odometer, nextPoint2);

			goHome = true;
			treeTravel(currentPoint, point, odometer);

		} else if (!availability[nextPoint1] && !availability[nextPoint2] && availability[nextPoint3]) {

			treeTravel(point, nextPoint3, odometer);
			probe(odometer, nextPoint3);

			goHome = true;
			treeTravel(currentPoint, point, odometer);

		}

		if (point == 0) {
			odometer.setX((T_x) * TILE_SIZE);
			odometer.setY((T_y - 1) * TILE_SIZE);
		} else if (point == 1) {
			odometer.setX((T_x + 1) * TILE_SIZE);
			odometer.setY((T_y) * TILE_SIZE);
		} else if (point == 2) {
			odometer.setX((T_x) * TILE_SIZE);
			odometer.setY((T_y + 1) * TILE_SIZE);
		} else if (point == 3) {
			odometer.setX((T_x - 1) * TILE_SIZE);
			odometer.setY((T_y) * TILE_SIZE);
		}

	}

	/**
	 * This method defines the routine for traveling between the sides, after the
	 * avalability of the side is determined
	 * 
	 * @param startPoint the side traveling from
	 * @param endPoint   the side traveling to
	 * @param odometer   the odometer used by the robot
	 */
	public static void treeTravel(int startPoint, int endPoint, Odometer odometer) {
		if (rings < TOTALRING || goHome == true) {
			// reset motor before rotating
			leftMotor.stop(true);
			rightMotor.stop(false);
			for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
				motor.setAcceleration(3000);
			}
			try {
				Thread.sleep(200);
			} catch (InterruptedException e) {
			}
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);

			int direction;

			if (endPoint == (((startPoint + 1) % 4) + 4) % 4 || endPoint == (((startPoint - 1) % 4) + 4) % 4) {

				if (endPoint == (((startPoint + 1) % 4) + 4) % 4) {

					direction = -1;

				} else {

					direction = 1;

				}

				leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, -direction * 90), true);
				rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, -direction * 90), false);
				leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE - 8), true);
				rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE - 8), false);
				Navigation.lineCorrection(odometer);

				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);

				leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, direction * 90), true);
				rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, direction * 90), false);
				leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE - 8), true);
				rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE - 8), false);
				Navigation.lineCorrection(odometer);

			} else if (endPoint == (startPoint + 2) % 4 || endPoint == (startPoint - 2) % 4) {

				if (endPoint == (startPoint + 2) % 4) {

					direction = -1;

				} else {

					direction = 1;

				}

				leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, direction * 90), true);
				rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, direction * 90), false);
				leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE - 8), true);
				rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE - 8), false);
				Navigation.lineCorrection(odometer);
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, -direction * 90), true);
				rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, -direction * 90), false);
				leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE - 8), true);
				rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE - 8), false);
				Navigation.lineCorrection(odometer);
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE - 8), true);
				rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE - 8), false);
				Navigation.lineCorrection(odometer);
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(Navigation.convertAngle(WHEEL_RAD, TRACK, -direction * 90), true);
				rightMotor.rotate(-Navigation.convertAngle(WHEEL_RAD, TRACK, -direction * 90), false);
				leftMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE - 8), true);
				rightMotor.rotate(Navigation.convertDistance(WHEEL_RAD, TILE_SIZE - 8), false);
				Navigation.lineCorrection(odometer);

			}

			currentPoint = endPoint;
		}
	}

}