package frc.robot.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {

  @AutoLog
  public static class FeederIOInputs {
    public double rollerPositionRot = 0.0;
    public double rollerVelocityRps = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerCurrentAmps = 0.0;

    public boolean topNoteSensorTripped = false;

    public boolean feederMotorConnected = false;
  }

  public default void updateInputs(FeederIOInputs inputs) {}

  public default void setRollerSpeedVolts(double volts) {}

  public default void setBrakeMode(boolean enableBrakeMode) {}

  public default void stopRoller() {}
}
