package frc.robot.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public class ClimberIOInputs {
    public double leftWinchPositionRot = 0.0;
    public double leftWinchVelocityRpm = 0.0;
    public double leftWinchAppliedVolts = 0.0;
    public double leftWinchCurrentAmps = 0.0;

    public double rightWinchPositionRot = 0.0;
    public double rightWinchVelocityRpm = 0.0;
    public double rightWinchAppliedVolts = 0.0;
    public double rightWinchCurrentAmps = 0.0;

    public boolean allMotorsConnected = false;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setLeftVoltage(double volts) {}

  public default void setRightVoltage(double volts) {}
}
