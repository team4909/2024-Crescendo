package frc.robot.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class FeederIOTalonFX implements FeederIO {
  private final double kFeederReduction = 1.0;
  private final TalonFX m_feederMotor;
  private final DigitalInput m_topNoteSensor, m_bottomNoteSensor;

  private final DutyCycleOut m_feederControl;

  private final StatusSignal<Double> m_feederVelocitySignal,
      m_feederAppliedVoltageSignal,
      m_feederCurrentSignal;

  public FeederIOTalonFX() {
    m_feederMotor = new TalonFX(19, Constants.kOtherCanBus);
    m_topNoteSensor = new DigitalInput(0);
    m_bottomNoteSensor = new DigitalInput(1);

    final CurrentLimitsConfigs currentLimitsConfig = new CurrentLimitsConfigs();
    currentLimitsConfig.SupplyCurrentLimit = 30.0;
    currentLimitsConfig.SupplyCurrentLimitEnable = true;
    m_feederMotor.getConfigurator().apply(currentLimitsConfig);

    m_feederVelocitySignal = m_feederMotor.getVelocity();
    m_feederAppliedVoltageSignal = m_feederMotor.getMotorVoltage();
    m_feederCurrentSignal = m_feederMotor.getStatorCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, m_feederVelocitySignal, m_feederAppliedVoltageSignal, m_feederCurrentSignal);
    m_feederMotor.optimizeBusUtilization();

    m_feederControl = new DutyCycleOut(0, true, false, false, false);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.feederMotorConnected =
        BaseStatusSignal.refreshAll(
                m_feederVelocitySignal, m_feederAppliedVoltageSignal, m_feederCurrentSignal)
            .equals(StatusCode.OK);

    inputs.feederVelocityRps = m_feederVelocitySignal.getValue() / kFeederReduction;
    inputs.feederAppliedVolts = m_feederAppliedVoltageSignal.getValue();
    inputs.feederCurrentAmps = m_feederCurrentSignal.getValue();
    inputs.topNoteSensorTripped = m_topNoteSensor.get();
    inputs.bottomNoteSensorTripped = m_bottomNoteSensor.get();
  }

  @Override
  public void setFeederDutyCycle(double volts) {
    m_feederMotor.setControl(m_feederControl.withOutput(volts));
  }
}
