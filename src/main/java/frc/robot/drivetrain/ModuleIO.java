package frc.robot.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public Rotation2d steerAbsolutePosition = new Rotation2d();
    public Rotation2d steerPosition = new Rotation2d();
    public double steerVelocityRadPerSec = 0.0;
    public double steerAppliedVolts = 0.0;
    public double steerCurrentAmps = 0.0;
    public double steerClosedLoopErrorRad = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};

    public double totalOdometryLoopTime = 0.0;
    public boolean devicesConnected = false;
  }

  public default void updateInputs(ModuleIOInputs inputs) {}

  public default void setDriveRPS(double speedRPS) {}

  public default void setSteerRotations(double angleRotations) {}

  public default void setDriveVoltage(double volts) {}

  public default void setSteerVoltage(double volts) {}

  public default void setDriveBrakeMode(boolean enable) {}

  public default void setSteerBrakeMode(boolean enable) {}
}
