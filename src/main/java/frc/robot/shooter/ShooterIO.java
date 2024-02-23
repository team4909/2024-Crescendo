package frc.robot.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double topRollerVelocityRpm = 0.0;
    public double topRollerAccelerationRpmSq = 0.0;
    public double topRollerAppliedVolts = 0.0;
    public double topRollerCurrentAmps = 0.0;

    public double bottomRollerVelocityRpm = 0.0;
    public double bottomRollerAccelerationRpmSq = 0.0;
    public double bottomRollerAppliedVolts = 0.0;
    public double bottomRollerCurrentAmps = 0.0;

    public boolean rollerMotorsConnected = false;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setRollersRPS(double velocityRPS) {}

  public default void setRollerDutyCycle(double volts) {}
}
