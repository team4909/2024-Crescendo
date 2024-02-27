package frc.robot.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {

  private final LinearSystem<N2, N1, N2> m_topRollerPlant =
      LinearSystemId.createDCMotorSystem(Shooter.topRollerkV, Shooter.topRollerkA);
  private final LinearSystem<N2, N1, N2> m_bottomRollerPlant =
      LinearSystemId.createDCMotorSystem(Shooter.bottomRollerkV, Shooter.bottomRollerkA);
  private final DCMotor m_rollerGearbox = DCMotor.getFalcon500Foc(1);
  private final DCMotorSim m_topRollerSim = new DCMotorSim(m_topRollerPlant, m_rollerGearbox, 1.0);
  private final DCMotorSim m_bottomRollerSim =
      new DCMotorSim(m_bottomRollerPlant, m_rollerGearbox, 1.0);

  private double m_topRollerAppliedVolts = 0.0;
  private double m_bottomRollerAppliedVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    m_topRollerSim.setInputVoltage(m_topRollerAppliedVolts);
    m_bottomRollerSim.setInputVoltage(m_bottomRollerAppliedVolts);
    m_topRollerSim.update(0.02);
    m_bottomRollerSim.update(0.02);

    /**
     * The units here are weird. The plant was made with gains that were in volts/rotations. This
     * means what the sim thinks is radians is actually rotations. However, for simulating current
     * draw the DCMotor class expects speed in actual radians/s, so just for this we have to convert
     * rot/s to rad/s, thus we cannot use the sim class's getCurrentDrawAmps() method.
     */
    inputs.topRollerPositionRot = m_topRollerSim.getAngularPositionRad();
    inputs.topRollerVelocityRps = m_topRollerSim.getAngularVelocityRadPerSec();
    inputs.topRollerAppliedVolts = m_topRollerAppliedVolts;
    inputs.topRollerCurrentAmps =
        m_rollerGearbox.getCurrent(
                Units.rotationsToRadians(inputs.topRollerVelocityRps), m_topRollerAppliedVolts)
            * Math.signum(m_topRollerAppliedVolts);
    inputs.bottomRollerPositionRot = m_bottomRollerSim.getAngularPositionRad();
    inputs.bottomRollerVelocityRps = m_bottomRollerSim.getAngularVelocityRadPerSec();
    inputs.bottomRollerAppliedVolts = m_bottomRollerAppliedVolts;
    inputs.bottomRollerCurrentAmps =
        m_rollerGearbox.getCurrent(
                Units.rotationsToRadians(inputs.bottomRollerVelocityRps),
                m_bottomRollerAppliedVolts)
            * Math.signum(m_bottomRollerAppliedVolts);
  }

  @Override
  public void setTopRollerVoltage(double volts) {
    m_topRollerAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setBottomRollerVoltage(double volts) {
    m_bottomRollerAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }
}
