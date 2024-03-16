package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.arm.Arm.ArmSetpoints;

public class ArmIOSim implements ArmIO {

  private final SingleJointedArmSim m_elbowSim, m_wristSim;

  private double m_elbowAppliedVolts = 0.0;
  private double m_wristAppliedVolts = 0.0;

  public ArmIOSim() {
    m_elbowSim =
        new SingleJointedArmSim(
            ArmConstants.kElbowGearbox,
            ArmConstants.kElbowReduction,
            ArmConstants.kElbowMoiKgMetersSq,
            ArmConstants.kElbowLengthMeters,
            ArmConstants.kElbowMinAngleRad,
            ArmConstants.kElbowMaxAngleRad,
            true,
            ArmSetpoints.kStowed.elbowAngle);
    m_wristSim =
        new SingleJointedArmSim(
            ArmConstants.kWristGearbox,
            ArmConstants.kWristReduction,
            ArmConstants.kWristMoiKgMetersSq,
            ArmConstants.kWristLengthMeters,
            ArmConstants.kWristMinAngleRad,
            ArmConstants.kWristMaxAngleRad,
            true,
            ArmSetpoints.kStowed.wristAngle);
  }

  public void updateInputs(ArmIOInputs inputs) {

    m_elbowSim.setInputVoltage(m_elbowAppliedVolts);
    m_wristSim.setInputVoltage(m_wristAppliedVolts);
    m_elbowSim.update(0.02);
    m_wristSim.update(0.02);

    inputs.elbowPositionRot = m_elbowSim.getAngleRads();
    inputs.elbowVelocityRps = m_elbowSim.getVelocityRadPerSec();
    inputs.elbowAppliedVolts = m_elbowAppliedVolts;
    inputs.elbowCurrentAmps =
        new double[] {
          ArmConstants.kElbowGearbox.getCurrent(
              m_elbowSim.getVelocityRadPerSec(), m_elbowAppliedVolts)
        };

    inputs.wristPositionRot = m_wristSim.getAngleRads();
    inputs.wristVelocityRps = m_wristSim.getVelocityRadPerSec();
    inputs.wristAppliedVolts = m_wristAppliedVolts;
    inputs.wristCurrentAmps =
        new double[] {
          ArmConstants.kWristGearbox.getCurrent(
              m_wristSim.getVelocityRadPerSec(), m_wristAppliedVolts)
        };

    inputs.allMotorsConnected = true;
  }

  public void setElbowVoltage(double volts) {
    m_elbowAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  public void setWristVoltage(double volts) {
    m_wristAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }
}
