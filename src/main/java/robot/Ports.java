package robot;

import java.util.Map;

public final class Ports {
  // TODO: Add and change all ports as needed.

  public static final Map<Integer, String> idToName =
      Map.ofEntries(
          Map.entry(Drive.RIGHT_LEADER, "RL drive"),
          Map.entry(Drive.RIGHT_FOLLOWER, "RF drive"),
          Map.entry(Drive.LEFT_LEADER, "LL drive"),
          Map.entry(Drive.LEFT_FOLLOWER, "LF drive"),
          Map.entry(Drive.GYRO_CHANNEL, "gyro"));

  public static final class OI {
    public static final int OPERATOR = 0;
    public static final int DRIVER = 1;
  }

  public static final class Drive {
    /** Port of the Front-Left Drivetrain motor. */
    public static final int LEFT_LEADER = 11;

    /** Port of the Front-Right Drivetrain motor. */
    public static final int RIGHT_LEADER = 13;

    /** Port of the Rear-Left Drivetrain motor. */
    public static final int LEFT_FOLLOWER = 18;

    /** Port of the Rear-Right Drivetrain motor. */
    public static final int RIGHT_FOLLOWER = 25;
    public static final int GYRO_CHANNEL = 1;
    // etc
  }
}
