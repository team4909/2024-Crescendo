package frc.robot.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX m_topRoller, m_bottomRoller;
  private final VelocityVoltage m_topRollerControl, m_bottomRollerControl;
  private final StatusSignal<Double> m_topRollerVelocitySignal,
      m_topRollerAccelerationSignal,
      m_topRollerAppliedVoltageSignal,
      m_topRollerCurrentSignal,
      m_bottomRollerVelocitySignal,
      m_bottomRollerAccelerationSignal,
      m_bottomRollerAppliedVoltageSignal,
      m_bottomRollerCurrentSignal;

  public ShooterIOTalonFX() {
    m_topRoller = new TalonFX(20, Constants.kOtherCanBus);
    m_bottomRoller = new TalonFX(21, Constants.kOtherCanBus);

    final Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0.1;
    m_topRoller.getConfigurator().apply(slot0Configs);
    m_bottomRoller.getConfigurator().apply(slot0Configs);

    m_topRollerVelocitySignal = m_topRoller.getVelocity();
    m_topRollerAccelerationSignal = m_topRoller.getAcceleration();
    m_topRollerAppliedVoltageSignal = m_topRoller.getMotorVoltage();
    m_topRollerCurrentSignal = m_topRoller.getStatorCurrent();
    m_bottomRollerVelocitySignal = m_bottomRoller.getVelocity();
    m_bottomRollerAccelerationSignal = m_bottomRoller.getAcceleration();
    m_bottomRollerAppliedVoltageSignal = m_bottomRoller.getMotorVoltage();
    m_bottomRollerCurrentSignal = m_bottomRoller.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        m_topRollerVelocitySignal,
        m_topRollerAccelerationSignal,
        m_topRollerAppliedVoltageSignal,
        m_topRollerCurrentSignal,
        m_bottomRollerVelocitySignal,
        m_bottomRollerAccelerationSignal,
        m_bottomRollerAppliedVoltageSignal,
        m_bottomRollerCurrentSignal);

    ParentDevice.optimizeBusUtilizationForAll(m_topRoller, m_bottomRoller);

    m_topRollerControl = new VelocityVoltage(0, 0, true, 0, 0, true, false, false);
    m_bottomRollerControl = new VelocityVoltage(0, 0, true, 0, 0, true, false, false);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.rollerMotorsConnected =
        BaseStatusSignal.refreshAll(
                m_topRollerVelocitySignal,
                m_topRollerAccelerationSignal,
                m_topRollerAppliedVoltageSignal,
                m_topRollerCurrentSignal,
                m_bottomRollerVelocitySignal,
                m_bottomRollerAccelerationSignal,
                m_bottomRollerAppliedVoltageSignal,
                m_bottomRollerCurrentSignal)
            .equals(StatusCode.OK);

    inputs.topRollerVelocityRps = m_topRollerVelocitySignal.getValue();
    inputs.topRollerAccelerationRpsSq = m_topRollerAccelerationSignal.getValue();
    inputs.topRollerAppliedVolts = m_topRollerAppliedVoltageSignal.getValue();
    inputs.topRollerCurrentAmps = m_topRollerCurrentSignal.getValue();
    inputs.bottomRollerVelocityRps = m_bottomRollerVelocitySignal.getValue();
    inputs.bottomRollerAccelerationRpsSq = m_bottomRollerAccelerationSignal.getValue();
    inputs.bottomRollerAppliedVolts = m_bottomRollerAppliedVoltageSignal.getValue();
    inputs.bottomRollerCurrentAmps = m_bottomRollerCurrentSignal.getValue();
  }

  @Override
  public void setRollersRPS(double velocityRPS) {
    m_topRoller.setControl(m_topRollerControl.withVelocity(velocityRPS));
    m_bottomRoller.setControl(m_bottomRollerControl.withVelocity(velocityRPS));
  }
}
