package frc.robot.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {

  @AutoLog
  public static class FeederIOInputs {
    public double feederVelocityRps = 0.0;
    public double feederAppliedVolts = 0.0;
    public double feederCurrentAmps = 0.0;

    public boolean topNoteSensorTripped = false;
    public boolean bottomNoteSensorTripped = false;

    public boolean feederMotorConnected = false;
  }
  public default void updateInputs(FeederIOInputs inputs) {}

  public default void setFeederDutyCycle(double dutyCycle) {}
}
