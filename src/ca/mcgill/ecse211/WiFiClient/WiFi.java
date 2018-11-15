package ca.mcgill.ecse211.WiFiClient;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import lejos.hardware.Button;


/**
 * This class contains fields and methods for getting from wifi
 * @author Team 12
 */
public class WiFi {
	
	//** Set these as appropriate for your team and current situation **
	//put the IP of the laptop we are using
	  private static final String SERVER_IP = "192.168.2.26";
	  private static final int TEAM_NUMBER = 12;

	  // Enable/disable printing of debug info from the WiFi class
	  private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

	/**
	 * When called this method will receive all wifi data and return it
	 * @return the Map containing all wifi data
	 */
	@SuppressWarnings("rawtypes")
	public static Map Wifi() {

	    System.out.println("Running..");

	    // Initialize WifiConnection class
	    WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);
	    
	    Map data = null;
	    
	    // Connect to server and get the data, catching any errors that might occur
	    try {
	      /*
	       * getData() will connect to the server and wait until the user/TA presses the "Start" button
	       * in the GUI on their laptop with the data filled in. Once it's waiting, you can kill it by
	       * pressing the upper left hand corner button (back/escape) on the EV3. getData() will throw
	       * exceptions if it can't connect to the server (e.g. wrong IP address, server not running on
	       * laptop, not connected to WiFi router, etc.). It will also throw an exception if it connects
	       * but receives corrupted data or a message from the server saying something went wrong. For
	       * example, if TEAM_NUMBER is set to 1 above but the server expects teams 17 and 5, this robot
	       * will receive a message saying an invalid team number was specified and getData() will throw
	       * an exception letting you know.
	       */
	      data = conn.getData();

//	      // Example 1: Print out all received data
//	      System.out.println("Map:\n" + data);
//
//	      // Example 2 : Print out specific values
//	      int redTeam = ((Long) data.get("RedTeam")).intValue();
//	      System.out.println("Red Team: " + redTeam);
//
//	      int TR_x = ((Long) data.get("TR_x")).intValue();
//	      System.out.println("X component of the red ring tree: " + TR_x);
//
//	      // Example 3: Compare value
//	      int tn_ll_x =  ((Long) data.get("TNR_LL_x")).intValue();
//	      if (tn_ll_x < 5) {
//	        System.out.println("Red Tunnel LL corner X < 5");
//	      }
//	      else {
//	        System.out.println("Red Tunnel LL corner X >= 5");
//	      }
//
	    } catch (Exception e) {
	      System.err.println("Error: " + e.getMessage());
	    }
	    
	    return data;

	  }

}
