package frc.robot.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double topRollerPositionRot = 0.0;
    public double topRollerVelocityRps = 0.0;
    public double topRollerAppliedVolts = 0.0;
    public double topRollerCurrentAmps = 0.0;

    public double bottomRollerPositionRot = 0.0;
    public double bottomRollerVelocityRps = 0.0;
    public double bottomRollerAppliedVolts = 0.0;
    public double bottomRollerCurrentAmps = 0.0;

    public boolean rollerMotorsConnected = false;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setTopRollerVoltage(double volts) {}

  public default void setBottomRollerVoltage(double volts) {}

  public default void stopRollers() {}
}
