package ca.mcgill.ecse211.dpmProject;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

/**
 * This class contains methods for the ring fetching routine and mechanism 
 * @author Team12
 *
 */
public class Grabber {
	private static final EV3LargeRegulatedMotor leftMotor  = Project.leftMotor; //the motor for the left wheel
	private static final EV3LargeRegulatedMotor rightMotor = Project.rightMotor; //the motor for the right wheel
	private static final EV3LargeRegulatedMotor armMotor = Project.armMotor; //the motor for raising/lowering the arm 
	private static final EV3MediumRegulatedMotor hookMotor = Project.hookMotor; //the motor for motorizing the hooks 
	private static final int ARM_SPEED = Project.ARM_SPEED; //this is the speed for the arm for the arm motor 
	private static final int HOOK_SPEED = Project.HOOK_SPEED; //this is the angle which the hook will open/close	
	private static final int HOOK_ANGLE = Project.HOOK_ANGLE; //this is the angle which the hook will open/close	
	private static final int LOW_ANGLE = Project.LOW_ANGLE; //the angle the arm motor needs to turn to reach lowly-hanged rings, with respect to the initial position 
	private static final int HIGH_ANGLE = Project.HIGH_ANGLE; //the angle the arm motor needs to turn to reach highly-hanged rings, with respect to the initial position 
	private static final int UNLOAD_ANGLE = Project.UNLOAD_ANGLE; //the angle the arm motor needs to turn to unload the ring(s), with respect to the initial position 
	/**
	 * This method is used for turning the arm to fetch the rings on the upper level of the tree
	 */
	public static void lowLevel() {
		armMotor.setSpeed(ARM_SPEED);
		armMotor.rotate(LOW_ANGLE);
	}
	
	/**
	 * This method is used for turning the arm to fetch the rings on the lower level of the tree
	 */
	public static void highLevel() {
		armMotor.setSpeed(ARM_SPEED);
		armMotor.rotate(HIGH_ANGLE);
	}

	/**
	 * This method is used for opening the a the hook to fetch the ring
	 */
	public static void openHook() {
		hookMotor.rotate(HOOK_ANGLE);
	}
	/**
	 * This method is used for closing the hook to either probe though hole or drop the rings
	 */
	public static void closeHook() {
		hookMotor.rotate(-HOOK_ANGLE);
	}
	
	/**
	 * this method is used to unload the ring using the arm motor
	 */
	public static void unload(){
		armMotor.setSpeed(ARM_SPEED);
		armMotor.rotate(UNLOAD_ANGLE);
	}
	
	/**
	 * this method is used to reset the arm to the initial position (falling on the back support)
	 */
	public static void resetArm(EV3LargeRegulatedMotor armMotor) {
		armMotor.setSpeed(ARM_SPEED);
	}
	

	
}
