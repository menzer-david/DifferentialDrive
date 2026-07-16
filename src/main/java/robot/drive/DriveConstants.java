package robot.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N7;

public class DriveConstants {
  public static final double WHEEL_RADIUS = 0.08; // Meters
  public static final double CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS;
  public static final double GEARING = 8.0;
  public static final double POSITION_FACTOR = CIRCUMFERENCE * GEARING;
  public static final double VELOCITY_FACTOR = POSITION_FACTOR / 60.0;
  public static final double MAX_VOLTAGE = 12;

  public static final class FF {
    public static final double kS = 0;
    public static final double kV = 0;
  }

  public static final class PID {
    public static final double kP = 2;
    public static final double kI = 0.0;
    public static final double kD = 0.15;
  }

  public static final double MAX_SPEED = 2; // Meters per second

  public static final double TRACK_WIDTH = 0.7112; // Meters
  public static final double MOI = 7.5;
  public static final double DRIVE_MASS = 60.0; // kg
  public static final Matrix<N7, N1> STD_DEVS = VecBuilder.fill(0, 0, 0, 0, 0, 0, 0);
}
