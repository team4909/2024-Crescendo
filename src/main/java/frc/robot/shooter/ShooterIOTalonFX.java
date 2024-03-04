package frc.robot.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX m_topRoller, m_bottomRoller;
  private final VoltageOut m_topRollerControl, m_bottomRollerControl;
  private final StatusSignal<Double> m_topRollerPositionSignal,
      m_topRollerVelocitySignal,
      m_topRollerAppliedVoltageSignal,
      m_topRollerCurrentSignal,
      m_bottomRollerPositionSignal,
      m_bottomRollerVelocitySignal,
      m_bottomRollerAppliedVoltageSignal,
      m_bottomRollerCurrentSignal;

  public ShooterIOTalonFX() {
    m_topRoller = new TalonFX(20, Constants.kSuperstructureCanBus);
    m_bottomRoller = new TalonFX(21, Constants.kSuperstructureCanBus);

    final TalonFXConfiguration topRollerMotorConfig = new TalonFXConfiguration();
    m_topRoller.getConfigurator().apply(topRollerMotorConfig);
    final TalonFXConfiguration bottomRollerMotorConfig = new TalonFXConfiguration();
    m_bottomRoller.getConfigurator().apply(bottomRollerMotorConfig);

    m_topRollerPositionSignal = m_topRoller.getPosition();
    m_topRollerVelocitySignal = m_topRoller.getVelocity();
    m_topRollerAppliedVoltageSignal = m_topRoller.getMotorVoltage();
    m_topRollerCurrentSignal = m_topRoller.getStatorCurrent();
    m_bottomRollerPositionSignal = m_bottomRoller.getPosition();
    m_bottomRollerVelocitySignal = m_bottomRoller.getVelocity();
    m_bottomRollerAppliedVoltageSignal = m_bottomRoller.getMotorVoltage();
    m_bottomRollerCurrentSignal = m_bottomRoller.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        m_topRollerVelocitySignal,
        m_topRollerAppliedVoltageSignal,
        m_topRollerCurrentSignal,
        m_bottomRollerVelocitySignal,
        m_bottomRollerAppliedVoltageSignal,
        m_bottomRollerCurrentSignal);

    ParentDevice.optimizeBusUtilizationForAll(m_topRoller, m_bottomRoller);

    m_topRollerControl = new VoltageOut(0, true, false, false, false).withUpdateFreqHz(0.0);
    m_bottomRollerControl = new VoltageOut(0, true, false, false, false).withUpdateFreqHz(0.0);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.rollerMotorsConnected =
        BaseStatusSignal.refreshAll(
                m_topRollerPositionSignal,
                m_topRollerVelocitySignal,
                m_topRollerAppliedVoltageSignal,
                m_topRollerCurrentSignal,
                m_bottomRollerPositionSignal,
                m_bottomRollerVelocitySignal,
                m_bottomRollerAppliedVoltageSignal,
                m_bottomRollerCurrentSignal)
            .equals(StatusCode.OK);

    inputs.topRollerPositionRot = m_topRollerPositionSignal.getValue();
    inputs.topRollerVelocityRps = m_topRollerVelocitySignal.getValue();
    inputs.topRollerAppliedVolts = m_topRollerAppliedVoltageSignal.getValue();
    inputs.topRollerCurrentAmps = m_topRollerCurrentSignal.getValue();
    inputs.bottomRollerPositionRot = m_bottomRollerPositionSignal.getValue();
    inputs.bottomRollerVelocityRps = m_bottomRollerVelocitySignal.getValue();
    inputs.bottomRollerAppliedVolts = m_bottomRollerAppliedVoltageSignal.getValue();
    inputs.bottomRollerCurrentAmps = m_bottomRollerCurrentSignal.getValue();
  }

  @Override
  public void setTopRollerVoltage(double volts) {
    m_topRoller.setControl(m_topRollerControl.withOutput(volts));
  }

  @Override
  public void setBottomRollerVoltage(double volts) {
    m_bottomRoller.setControl(m_bottomRollerControl.withOutput(volts));
  }

  public void stopRollers() {
    m_topRoller.setVoltage(0.0);
    m_bottomRoller.setVoltage(0.0);
  }

  public void setRollerDutyCycle(double volts) {
    m_topRoller.setControl(new DutyCycleOut(volts));
    m_bottomRoller.setControl(new DutyCycleOut(volts));
  }
}
