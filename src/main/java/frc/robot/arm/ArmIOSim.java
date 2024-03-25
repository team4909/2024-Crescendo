package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.arm.Arm.ArmSetpoints;

public class ArmIOSim implements ArmIO {

  private final SingleJointedArmSim m_elbowSim, m_wristSim;
  private final PIDController m_elbowFeedback, m_wristFeedback;
  private final ArmFeedforward m_elbowFeedforward, m_wristFeedforward;
  private double m_currentElbowAngleRot, m_currentWristAngleRot;

  private double m_elbowAppliedVolts = 0.0;
  private double m_wristAppliedVolts = 0.0;

  public ArmIOSim() {
    m_elbowSim =
        new SingleJointedArmSim(
            ArmConstants.kElbowGearbox,
            ArmConstants.kElbowReduction,
            ArmConstants.kElbowMoiKgMetersSq,
            ArmConstants.kElbowLengthMeters,
            ArmSetpoints.kStowed.elbowAngle,
            ArmConstants.kElbowMaxAngleRad,
            false,
            ArmSetpoints.kStowed.elbowAngle);
    m_wristSim =
        new SingleJointedArmSim(
            ArmConstants.kWristGearbox,
            ArmConstants.kWristReduction,
            ArmConstants.kWristMoiKgMetersSq,
            ArmConstants.kWristLengthMeters,
            ArmConstants.kWristMinAngleRad,
            ArmConstants.kWristMaxAngleRad,
            false,
            ArmSetpoints.kStowed.wristAngle);
    m_elbowFeedback = new PIDController(25.0, 0.0, 0.0);
    m_wristFeedback = new PIDController(20.0, 0.0, 0.0);
    m_elbowFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);
    m_wristFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);
    m_wristFeedback.enableContinuousInput(-0.5, 0.5);
  }

  public void updateInputs(ArmIOInputs inputs) {

    m_elbowSim.update(0.02);
    m_wristSim.update(0.02);

    inputs.elbowPositionRot = Units.radiansToRotations(m_elbowSim.getAngleRads());
    m_currentElbowAngleRot = inputs.elbowPositionRot;
    inputs.elbowVelocityRps = Units.radiansToRotations(m_elbowSim.getVelocityRadPerSec());
    inputs.elbowAppliedVolts = m_elbowAppliedVolts;
    inputs.elbowStatorCurrentAmps =
        new double[] {
          ArmConstants.kElbowGearbox.getCurrent(
              m_elbowSim.getVelocityRadPerSec(), m_elbowAppliedVolts)
        };

    inputs.wristPositionRot = Units.radiansToRotations(m_wristSim.getAngleRads());
    m_currentWristAngleRot = inputs.wristPositionRot;
    inputs.wristVelocityRps = Units.radiansToRotations(m_wristSim.getVelocityRadPerSec());
    inputs.wristAppliedVolts = m_wristAppliedVolts;
    inputs.wristStatorCurrentAmps =
        new double[] {
          ArmConstants.kWristGearbox.getCurrent(
              m_wristSim.getVelocityRadPerSec(), m_wristAppliedVolts)
        };

    inputs.allMotorsConnected = true;
  }

  @Override
  public void setElbowRotations(double angleRot) {
    m_elbowFeedback.reset();
    m_elbowAppliedVolts =
        m_elbowFeedback.calculate(m_currentElbowAngleRot, angleRot)
            + m_elbowFeedforward.calculate(Units.rotationsToRadians(angleRot), 0.0);
    m_elbowSim.setInputVoltage(m_elbowAppliedVolts);
  }

  @Override
  public void setWristRotations(double angleRot) {
    m_wristFeedback.reset();
    m_wristAppliedVolts =
        m_wristFeedback.calculate(m_currentWristAngleRot, angleRot)
            + m_wristFeedforward.calculate(Units.rotationsToRadians(angleRot), 0.0);

    m_wristSim.setInputVoltage(m_wristAppliedVolts);
  }

  @Override
  public void setElbowVoltage(double volts) {
    m_elbowAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setWristVoltage(double volts) {
    m_wristAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void stop() {
    m_elbowAppliedVolts = 0.0;
    m_wristAppliedVolts = 0.0;
  }
}
