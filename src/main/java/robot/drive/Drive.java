package robot.drive;

import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import robot.Constants;
import robot.Ports;
import robot.Robot;
import robot.drive.DriveConstants.FF;
import robot.drive.DriveConstants.PID;

public class Drive extends SubsystemBase {
  private final SparkMax leftLeader = new SparkMax(Ports.Drive.LEFT_LEADER, MotorType.kBrushless);
  private final SparkMax leftFollower =
      new SparkMax(Ports.Drive.LEFT_FOLLOWER, MotorType.kBrushless);
  private final SparkMax rightLeader = new SparkMax(Ports.Drive.RIGHT_LEADER, MotorType.kBrushless);
  private final SparkMax rightFollower =
      new SparkMax(Ports.Drive.RIGHT_FOLLOWER, MotorType.kBrushless);
  private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
  private final RelativeEncoder rightEncoder = rightLeader.getEncoder();
  private final AnalogGyro gyro = new AnalogGyro(Ports.Drive.GYRO_CHANNEL);
  private DifferentialDriveOdometry odometry;
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(FF.kS, FF.kV);
  private final PIDController leftPIDController = new PIDController(PID.kP, PID.kI, PID.kD);
  private final PIDController rightPIDController = new PIDController(PID.kP, PID.kI, PID.kD);
  private DifferentialDrivetrainSim driveSim;
  @Logged private final Field2d field2d = new Field2d();

  public Drive() {
    SparkMaxConfig baseConfig = new SparkMaxConfig();
    baseConfig.idleMode(IdleMode.kBrake);
    baseConfig.encoder.positionConversionFactor(DriveConstants.POSITION_FACTOR);
    baseConfig.encoder.velocityConversionFactor(DriveConstants.VELOCITY_FACTOR);
    SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
    leftLeaderConfig.apply(baseConfig).inverted(true);
    leftLeader.configure(
        leftLeaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    rightLeaderConfig.apply(baseConfig);
    rightLeader.configure(
        rightLeaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    leftFollowerConfig.apply(baseConfig);
    leftFollowerConfig.follow(leftLeader);
    leftFollower.configure(
        leftFollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
    rightFollowerConfig.apply(baseConfig);
    rightFollowerConfig.follow(rightLeader);
    rightFollower.configure(
        rightFollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    gyro.reset();

    odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0, new Pose2d());

    driveSim =
        new DifferentialDrivetrainSim(
            DCMotor.getMiniCIM(2),
            DriveConstants.GEARING,
            DriveConstants.MOI,
            DriveConstants.DRIVE_MASS,
            DriveConstants.WHEEL_RADIUS,
            DriveConstants.TRACK_WIDTH,
            DriveConstants.STD_DEVS);
  }

  public void drive(double leftSpeed, double rightSpeed) {
    final double realLeftSpeed = leftSpeed * DriveConstants.MAX_SPEED;
    final double realRightSpeed = rightSpeed * DriveConstants.MAX_SPEED;
    
    final double leftFeedforward = feedforward.calculate(realLeftSpeed);
    final double rightFeedforward = feedforward.calculate(realRightSpeed);

    final double leftPID = leftPIDController.calculate(leftEncoder.getVelocity(), realLeftSpeed);
    final double rightPID =
        rightPIDController.calculate(rightEncoder.getVelocity(), realRightSpeed);
    double leftVoltage = leftPID + leftFeedforward;
    double rightVoltage = rightPID + rightFeedforward;
    DogLog.log("leftVoltage", leftVoltage);
    DogLog.log("rightVoltage", rightVoltage);
    driveSim.setInputs(leftVoltage, rightVoltage);
    leftLeader.setVoltage(leftVoltage);
    rightLeader.setVoltage(rightVoltage);
  }

  private void updateOdometry(Rotation2d rotation) {
    odometry.update(rotation, leftEncoder.getPosition(), rightEncoder.getPosition());
  }

  @Override
  public void periodic() {
    updateOdometry(Robot.isReal() ? gyro.getRotation2d() : driveSim.getHeading());
    field2d.setRobotPose(pose());
  }

  public Pose2d pose() {
    return odometry.getPoseMeters();
  }

  public Command drive(DoubleSupplier vLeft, DoubleSupplier vRight) {
    DogLog.log("leftSpeed", vLeft.getAsDouble());
    DogLog.log("rightSpeed", vRight.getAsDouble());
    return run(() -> drive(vLeft.getAsDouble(), vRight.getAsDouble()));
  }

  @Override
  public void simulationPeriodic() {
    // sim.update() tells the simulation how much time has passed
    driveSim.update(Constants.PERIOD.in(Seconds));
    DogLog.log("Pose", pose());
    leftEncoder.setPosition(driveSim.getLeftPositionMeters());
    rightEncoder.setPosition(driveSim.getRightPositionMeters());
  }
}
