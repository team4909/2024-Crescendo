package frc.robot.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {

  private double m_topRollerAppliedVolts,
      m_bottomRollerAppliedVolts,
      m_centeringRollerAppliedVolts;

  private final DCMotorSim m_topRollerSim = new DCMotorSim(DCMotor.getNEO(1), 1.0, 0.01);
  private final DCMotorSim m_bottomRollerSim = new DCMotorSim(DCMotor.getNEO(1), 1.0, 0.01);
  private final DCMotorSim m_centeringRollersSim = new DCMotorSim(DCMotor.getBag(2), 1.0, 0.01);

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    m_topRollerSim.setInputVoltage(m_topRollerAppliedVolts);
    m_bottomRollerSim.setInputVoltage(m_bottomRollerAppliedVolts);
    m_centeringRollersSim.setInputVoltage(m_centeringRollerAppliedVolts);
    m_topRollerSim.update(0.02);
    m_bottomRollerSim.update(0.02);
    m_centeringRollersSim.update(0.02);

    inputs.topRollerVelocityRadPerSec = m_topRollerSim.getAngularVelocityRadPerSec();
    inputs.topRollerAppliedVolts = m_topRollerAppliedVolts;
    inputs.topRollerCurrentAmps = m_topRollerSim.getCurrentDrawAmps();

    inputs.bottomRollerVelocityRadPerSec = m_bottomRollerSim.getAngularVelocityRadPerSec();
    inputs.bottomRollerAppliedVolts = m_bottomRollerAppliedVolts;
    inputs.bottomRollerCurrentAmps = m_bottomRollerSim.getCurrentDrawAmps();

    inputs.centeringBagMotorsAppliedVolts = m_centeringRollerAppliedVolts;
    inputs.centeringBagMotorsCurrentAmps = m_centeringRollersSim.getCurrentDrawAmps();
  }

  @Override
  public void setTopRollerDutyCycle(double outputDutyCycle) {
    m_topRollerAppliedVolts = MathUtil.clamp(outputDutyCycle * 12.0, -12.0, 12.0);
  }

  @Override
  public void setBottomRollerDutyCycle(double outputDutyCycle) {
    m_bottomRollerAppliedVolts = MathUtil.clamp(outputDutyCycle * 12.0, -12.0, 12.0);
  }

  @Override
  public void setCenteringMotorsDutyCycle(double outputDutyCycle) {
    m_centeringRollerAppliedVolts = MathUtil.clamp(outputDutyCycle * 12.0, -12.0, 12.0);
  }

  @Override
  public void stopRollers() {
    m_topRollerAppliedVolts = 0.0;
    m_bottomRollerAppliedVolts = 0.0;
    m_centeringRollerAppliedVolts = 0.0;
  }
}
