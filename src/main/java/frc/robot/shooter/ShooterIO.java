package frc.robot.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double topRollerVelocityRps = 0.0;
    public double topRollerAccelerationRpsSq = 0.0;
    public double topRollerAppliedVolts = 0.0;
    public double topRollerCurrentAmps = 0.0;

    public double bottomRollerVelocityRps = 0.0;
    public double bottomRollerAccelerationRpsSq = 0.0;
    public double bottomRollerAppliedVolts = 0.0;
    public double bottomRollerCurrentAmps = 0.0;

    public double feederVelocityRps = 0.0;
    public double feederAccelerationRpsSq = 0.0;
    public double feederAppliedVolts = 0.0;
    public double feederCurrentAmps = 0.0;

    public boolean noteSensorTripped = false;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setRollerRPS(double velocityRPS) {}

  public default void setFeederDutyCycle(double dutyCycle) {}
}
