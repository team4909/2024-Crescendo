package frc.robot.feeder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FeederIOSim implements FeederIO {
  private DCMotorSim m_feederSim =
      new DCMotorSim(DCMotor.getFalcon500Foc(1), Feeder.kFeederReduction, 0.01);
  private double m_appliedVoltage = 0.0;

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    m_feederSim.setInputVoltage(m_appliedVoltage * 12.0);
    m_feederSim.update(0.02);

    inputs.feederMotorConnected = true;
    inputs.rollerPositionRot = m_feederSim.getAngularPositionRotations();
    inputs.rollerVelocityRps = Units.radiansToRotations(m_feederSim.getAngularVelocityRadPerSec());
    inputs.rollerCurrentAmps = m_feederSim.getCurrentDrawAmps();
    inputs.rollerAppliedVolts = m_appliedVoltage;
  }

  @Override
  public void setRollerSpeedVolts(double volts) {
    m_appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
  }
}
