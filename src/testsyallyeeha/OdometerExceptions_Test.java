package testsyallyeeha;

/**
 * This class is used to handle errors regarding the singleton pattern used for the odometer and odometerData
 * @author Team 12
 */
@SuppressWarnings("serial")
public class OdometerExceptions_Test extends Exception {

  public OdometerExceptions_Test(String Error) {
    super(Error);
  }

}
