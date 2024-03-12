package frc.robot.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  @AutoLog
  public static class ArmIOInputs {
    public double elbowAbsolutePositionRad = 0.0;
    public boolean elbowAbsoluteEncoderConnected = false;
    public double elbowEncoderRaw = 0.0;
    public double elbowRelativePositionRad = 0.0;
    public double elbowVelocityRadPerSec = 0.0;
    public double elbowAppliedVolts = 0.0;
    public double[] elbowCurrentAmps = new double[] {};

    public double wristAbsolutePositionRad = 0.0;
    public boolean wristAbsoluteEncoderConnected = false;
    public double wristEncoderRaw = 0.0;
    public double wristRelativePositionRad = 0.0;
    public double wristVelocityRadPerSec = 0.0;
    public double wristAppliedVolts = 0.0;
    public double[] wristCurrentAmps = new double[] {};

    public boolean allMotorsConnected = false;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setElbowVoltage(double volts) {}

  public default void setWristVoltage(double volts) {}

  public default void setBrakeMode(boolean enableBrakeMode) {}
}
