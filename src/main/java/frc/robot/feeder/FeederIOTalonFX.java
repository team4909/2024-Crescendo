package frc.robot.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class FeederIOTalonFX implements FeederIO {
  private final TalonFX m_feederMotor;
  private final DigitalInput m_topNoteSensor;

  private final StatusSignal<Double> m_rollerPositionSignal,
      m_rollerVelocitySignal,
      m_rollerAppliedVoltageSignal,
      m_rollerCurrentSignal;

  public FeederIOTalonFX() {
    m_feederMotor = new TalonFX(19, Constants.kSuperstructureCanBus);
    m_topNoteSensor = new DigitalInput(0);

    final TalonFXConfiguration feederMotorConfig = new TalonFXConfiguration();
    m_feederMotor.getConfigurator().apply(feederMotorConfig);
    feederMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    feederMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    feederMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    feederMotorConfig.Feedback.SensorToMechanismRatio = Feeder.kFeederReduction;
    m_feederMotor.getConfigurator().apply(feederMotorConfig);

    m_rollerPositionSignal = m_feederMotor.getPosition();
    m_rollerVelocitySignal = m_feederMotor.getVelocity();
    m_rollerAppliedVoltageSignal = m_feederMotor.getMotorVoltage();
    m_rollerCurrentSignal = m_feederMotor.getStatorCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, m_rollerVelocitySignal, m_rollerAppliedVoltageSignal, m_rollerCurrentSignal);
    m_feederMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.feederMotorConnected =
        BaseStatusSignal.refreshAll(
                m_rollerPositionSignal,
                m_rollerVelocitySignal,
                m_rollerAppliedVoltageSignal,
                m_rollerCurrentSignal)
            .isOK();

    inputs.rollerPositionRot = m_rollerPositionSignal.getValue();
    inputs.rollerVelocityRps = m_rollerVelocitySignal.getValue();
    inputs.rollerAppliedVolts = m_rollerAppliedVoltageSignal.getValue();
    inputs.rollerCurrentAmps = m_rollerCurrentSignal.getValue();
    inputs.topNoteSensorTripped = m_topNoteSensor.get();
  }

  @Override
  public void setRollerSpeedVolts(double volts) {
    m_feederMotor.setVoltage(volts);
  }

  public void setBrakeMode(boolean enableBrakeMode) {
    final NeutralModeValue neutralModeValue =
        enableBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    m_feederMotor.setNeutralMode(neutralModeValue);
  }

  @Override
  public void stopRoller() {
    m_feederMotor.setVoltage(0.0);
  }
}
