package frc.robot.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public class ClimberIOInputs {
    public double leftWinchPositionRad = 0.0;
    public double leftWinchVelocityRadPerSec = 0.0;
    public double leftWinchAppliedVolts = 0.0;
    public double leftWinchCurrentAmps = 0.0;

    public double rightWinchPositionRad = 0.0;
    public double rightWinchVelocityRadPerSec = 0.0;
    public double rightWinchAppliedVolts = 0.0;
    public double rightWinchCurrentAmps = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setLeftDutyCycle(double outputDutyCycle) {}

  public default void setRightDutyCycle(double outputDutyCycle) {}
}
