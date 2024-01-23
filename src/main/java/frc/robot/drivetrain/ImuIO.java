package frc.robot.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ImuIO {
  @AutoLog
  public static class ImuIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = new Rotation2d();
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    public double[] odometryYawTimestamps = new double[] {};
    public double yawVelocityRadPerSec = 0.0;
  }

  public default void updateInputs(ImuIOInputs inputs) {}

  public default void updateSim(double dThetaRad) {}
}
