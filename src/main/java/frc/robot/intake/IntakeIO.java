package frc.robot.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public class IntakeIOInputs {
    public double topRollerVelocityRadPerSec = 0.0;
    public double topRollerAppliedVolts = 0.0;
    public double topRollerCurrentAmps = 0.0;

    public double bottomRollerVelocityRadPerSec = 0.0;
    public double bottomRollerAppliedVolts = 0.0;
    public double bottomRollerCurrentAmps = 0.0;

    public double centeringBagMotorsAppliedVolts = 0.0;
    public double centeringBagMotorsCurrentAmps = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setTopRollerDutyCycle(double outputDutyCycle) {}

  public default void setBottomRollerDutyCycle(double outputDutyCycle) {}

  public default void setCenteringMotorsDutyCycle(double outputDutyCycle) {}

  public default void stopRollers() {}
}
