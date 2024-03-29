package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N4;

public class ArmIOSim implements ArmIO {
  private Vector<N4> m_elbowWristSimStates = VecBuilder.fill(0.0, 0.0, 0.0, 0.0);
  private ArmModel m_model = new ArmModel();

  private double m_elbowAppliedVolts = 0.0;
  private double m_wristAppliedVolts = 0.0;

  public ArmIOSim() {}

  public void updateInputs(ArmIOInputs inputs) {

    m_elbowWristSimStates =
        m_model.simulate(
            m_elbowWristSimStates, VecBuilder.fill(m_elbowAppliedVolts, m_wristAppliedVolts), 0.02);
    inputs.elbowAbsolutePositionRad = m_elbowWristSimStates.get(0, 0);
    inputs.elbowAbsoluteEncoderConnected = true;
    inputs.elbowRelativePositionRad = m_elbowWristSimStates.get(0, 0);
    inputs.elbowVelocityRadPerSec = m_elbowWristSimStates.get(2, 0);
    inputs.elbowAppliedVolts = m_elbowAppliedVolts;
    inputs.elbowCurrentAmps =
        new double[] {
          ArmModel.kElbowGearbox.getCurrent(m_elbowWristSimStates.get(2, 0), m_elbowAppliedVolts)
        };

    inputs.wristAbsolutePositionRad = m_elbowWristSimStates.get(1, 0);
    inputs.wristAbsoluteEncoderConnected = true;
    inputs.wristRelativePositionRad = m_elbowWristSimStates.get(1, 0);
    inputs.wristVelocityRadPerSec = m_elbowWristSimStates.get(3, 0);
    inputs.wristAppliedVolts = m_wristAppliedVolts;
    inputs.wristCurrentAmps =
        new double[] {
          ArmModel.kWristGearbox.getCurrent(m_elbowWristSimStates.get(3, 0), m_wristAppliedVolts)
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
