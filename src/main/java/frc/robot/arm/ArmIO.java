package frc.robot.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  @AutoLog
  public static class ArmIOInputs {
    public double elbowPositionRot = 0.0;
    public double elbowPositionSetpointRot = 0.0;
    public double elbowVelocityRps = 0.0;
    public double elbowAppliedVolts = 0.0;
    public double[] elbowCurrentAmps = new double[] {};

    public double wristPositionRot = 0.0;
    public double wristPositionSetpointRot = 0.0;
    public double wristVelocityRps = 0.0;
    public double wristAppliedVolts = 0.0;
    public double[] wristCurrentAmps = new double[] {};

    public boolean allMotorsConnected = false;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setElbowRotations(double angleRot) {}

  public default void setWristRotations(double angleRot) {}

  public default void setElbowVoltage(double volts) {}

  public default void setWristVoltage(double volts) {}

  public default void setBrakeMode(boolean enableBrakeMode) {}

  public default void configPD(double elbowkP, double elbowkD, double wristkP, double wristkD) {}

  /**
   * Numerator is volts, denominator is in terms of rotations (where applicable)
   */
  public default void configFF(
      double elbowkS,
      double elbowkV,
      double elbowkG,
      double wristkS,
      double wristkV,
      double wristkG) {}

  public default void configLimits(
      double elbowCruiseVelocityRps,
      double elbowAccelerationRpsSq,
      double wristCruiseVelocityRps,
      double wristAccelerationRpsSq) {}
}
