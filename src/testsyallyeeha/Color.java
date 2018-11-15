package testsyallyeeha;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

import java.util.concurrent.TimeUnit;

import lejos.hardware.Sound;

/**
 * This class contains methods need for color identification This method is
 * achieved by ranking the RGB space Euclidean distance to find the most likely
 * color of the ring
 * 
 * @author Team 12
 *
 */
public class Color {
	// the fields in the Color class contain mean and standard deviation values for
	// each color
	// all data are collected using the dataObtain() and sample() methods in the
	// tester class
	// All mean values are normalized after sampling (eliminated the effect of
	// ambient light )
	public static boolean FOUND = false;
	// the mean value and standard deviation of RGB values for the color Blue
	// (normalized)
	private static final double blueRMean = 0.149069;
	private static final double blueRDiv = 0.004008;
	private static final double blueGMean = 0.780432;
	private static final double blueGDiv = 0.006812;
	private static final double blueBMean = 0.60721;
	private static final double blueBDiv = 0.004681;

	// the mean value and standard deviation of RGB values for the color Green
	// (normalized)
	private static final double greenRMean = 0.398574;
	private static final double greenRDiv = 0.005483;
	private static final double greenGMean = 0.904758;
	private static final double greenGDiv = 0.005425;
	private static final double greenBMean = 0.15017;
	private static final double greenBDiv = 0.006374;

	// the mean value and standard deviation of RGB values for the color Yellow
	// (normalized)
	private static final double yellowRMean = 0.82158;
	private static final double yellowRDiv = 0.0043;
	private static final double yellowGMean = 0.55636;
	private static final double yellowGDiv = 0.00289;
	private static final double yellowBMean = 0.1244;
	private static final double yellowBDiv = 0.0026;

	// the mean value and standard deviation of RGB values for the color Orange
	// (normalized)
	private static final double orangeRMean = 0.959771;
	private static final double orangeRDiv = 0.007887;
	private static final double orangeGMean = 0.26671;
	private static final double orangeGDiv = 0.004966;
	private static final double orangeBMean = 0.08776;
	private static final double orangeBDiv = 0.004811;

	public static final Port portColor = Project.portColor; // get the port for the light (color sensor)
	public static final SensorModes myColor = Project.myColor; // create the color sensor object;
	public static final SampleProvider myColorSample = Project.myColorSample;
	public static final float[] sampleColor = Project.sampleColor; // create an array for the sensor
																		// readings
	private static SampleProvider usDistance = Project.usDistance;
	private static final float[] usData = Project.usData;

	private static double smallest = 1; // the max possible value for a normalized reading is 1
	private static double colorThreshold = 0.2; // all correct readings are smaller than thus thresold, obtained in the
												// color sampling process

	/**
	 * calculates the euclidean distance in RGB space
	 * 
	 * @param rN normalized red reading
	 * @param gN normalized green reading
	 * @param bN normalized blue reading
	 * @param rM mean red value for a color
	 * @param gM mean green value for a color
	 * @param bM mean blue value for a color
	 * @return the euclidean distance in double
	 */
	public static double euclidean(double rN, double gN, double bN, double rM, double gM, double bM) {
		return Math.sqrt(Math.pow((rN - rM), 2) + Math.pow((gN - gM), 2) + Math.pow((bN - bM), 2));
	}

	/**
	 * This is used for normalizing the r value of a new reading
	 * 
	 * @param r
	 *            red reading
	 * @param g
	 *            green reading
	 * @param b
	 *            blue reading
	 * @return normalized reading
	 */
	public static double rNormalize(float r, float g, float b) {
		return r / (Math.sqrt(Math.pow(r, 2) + Math.pow(g, 2) + Math.pow(b, 2)));
	}

	/**
	 * This is used for normalizing the g value of a new reading
	 * 
	 * @param r red reading
	 * @param g green reading
	 * @param b blue reading
	 * @return normalized reading
	 */
	public static double gNormalize(float r, float g, float b) {
		return g / (Math.sqrt(Math.pow(r, 2) + Math.pow(g, 2) + Math.pow(b, 2)));
	}

	/**
	 * This is used for normalizing the b value of a new reading
	 * 
	 * @param r red reading
	 * @param g green reading
	 * @param b blue reading
	 * @return normalized reading
	 */
	public static double bNormalize(float r, float g, float b) {
		return b / (Math.sqrt(Math.pow(r, 2) + Math.pow(g, 2) + Math.pow(b, 2)));
	}

	/**
	 * This method identifies the color (in TR int) of the ring
	 * 
	 * @return int color value of the target rang, 1 is blue, 2 is green, 3 is
	 *         yellow and 4 is orange
	 */
	public static int color() {
		// obtain reading from sensor and normalize them
		myColorSample.fetchSample(sampleColor, 0);
		double r = rNormalize(sampleColor[0] * 1000, sampleColor[1] * 1000, sampleColor[2] * 1000);
		double g = gNormalize(sampleColor[0] * 1000, sampleColor[1] * 1000, sampleColor[2] * 1000);
		double b = bNormalize(sampleColor[0] * 1000, sampleColor[1] * 1000, sampleColor[2] * 1000);

		double dBlue = euclidean(r, g, b, blueRMean, blueGMean, blueBMean); // check Euclidean distance from being Blue
		double dGreen = euclidean(r, g, b, greenRMean, greenGMean, greenBMean);// check Euclidean distance from being
																				// Green
		double dYellow = euclidean(r, g, b, yellowRMean, yellowGMean, yellowBMean);// check Euclidean distance from
																					// being Yellow
		double dOrange = euclidean(r, g, b, orangeRMean, orangeGMean, orangeBMean); // check Euclidean distance from
																					// being Orange

		// rank the distances and choose the color --smallest euclidean
		double[] d = { dBlue, dGreen, dYellow, dOrange };
		for (int i = 0; i < 4; i++) {
			if (i == 0)
				smallest = d[i];
			else if (d[i] < smallest)
				smallest = d[i];
		}
		System.out.println(r + ", " + g + ", " + b);
		System.out.println(dBlue + ", " + dGreen + ", " + dYellow + ", " + dOrange);
		// return the TR value
		if (smallest <= colorThreshold) {
			FOUND = true;
			if (smallest == dBlue)
				return 1; // return blue
			if (smallest == dGreen)
				return 2; // return green
			if (smallest == dYellow)
				return 3; // return yellow
			if (smallest == dOrange) {
				FOUND = false;
				return 4; // return orange
			} else {
				FOUND = false;
				return 0;
			}
		} else
			return 0;

	}

}
