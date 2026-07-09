package lib;

import static lib.UnitTestingUtil.setupTests;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class SparkUtilsTest {

  @BeforeEach
  public void setup() {
    setupTests();
  }

  @Test
  void configure() {
    SparkFlex motor = new SparkFlex(1, MotorType.kBrushless);
    SparkFlexConfig config = new SparkFlexConfig();

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    config.apply(config.idleMode(IdleMode.kBrake).smartCurrentLimit(30));

    config.apply(
        config
            .encoder
            .positionConversionFactor(0.5)
            .velocityConversionFactor(0.25)
            .uvwMeasurementPeriod(8)
            .uvwAverageDepth(2));

    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    assertEquals(IdleMode.kBrake, motor.configAccessor.getIdleMode());
    assertEquals(0.5, motor.configAccessor.encoder.getPositionConversionFactor());
    assertEquals(0.25, motor.configAccessor.encoder.getVelocityConversionFactor());
    assertEquals(8, motor.configAccessor.encoder.getUvwMeasurementPeriod());
    assertEquals(2, motor.configAccessor.encoder.getUvwAverageDepth());

    motor.close();
  }
}
