package frc.robot.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public class IntakeIOInputs {
    public double topRollerVelocityRpm = 0.0;
    public double topRollerAppliedVolts = 0.0;
    public double topRollerCurrentAmps = 0.0;

    public double bottomRollerVelocityRpm = 0.0;
    public double bottomRollerAppliedVolts = 0.0;
    public double bottomRollerCurrentAmps = 0.0;

    public double centeringBagMotorsAppliedVolts = 0.0;
    public double centeringBagMotorsCurrentAmps = 0.0;

    public boolean rollerMotorsConnected = false;
    public boolean centeringMotorConnected = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setFrontRollersVoltage(double volts) {}

  public default void setBackRollersVoltage(double volts) {}

  public default void setCenteringMotorsVoltage(double volts) {}

  public default void stopRollers() {}
}
