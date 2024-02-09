package frc.robot.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  @AutoLog
  public static class ArmIOInputs {
    public double elbowPositionRad = 0.0;
    public double elbowVelocityRadPerSec = 0.0;
    public double elbowAppliedVolts = 0.0;
    public double[] elbowCurrentAmps = new double[] {};

    public double wristPositionRad = 0.0;
    public double wristVelocityRadPerSec = 0.0;
    public double wristAppliedVolts = 0.0;
    public double[] wristCurrentAmps = new double[] {};
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setElbowRotatations(double rotations, double feedForwardVolts) {}

  public default void setWristRotatations(double rotations, double feedForwardVolts) {}

  public default void setElbowVoltage(double volts) {}

  public default void setWristVoltage(double volts) {}
}
