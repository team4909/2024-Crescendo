package frc.robot.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import java.time.Duration;

public class FeederIOTalonFX implements FeederIO {
  private final boolean kUseHighFrequencySensorPolling = true;
  private final TalonFX m_feederMotor;
  private final DigitalInput m_noteSensor;
  private final DigitalGlitchFilter m_glitchFilter;
  private final SensorThread m_sensorThread;

  private final StatusSignal<Double> m_rollerPositionSignal,
      m_rollerVelocitySignal,
      m_rollerAppliedVoltageSignal,
      m_rollerCurrentSignal;

  public FeederIOTalonFX() {
    m_feederMotor = new TalonFX(19, Constants.kSuperstructureCanBus);
    m_noteSensor = new DigitalInput(0);
    m_glitchFilter = new DigitalGlitchFilter();
    m_glitchFilter.setPeriodNanoSeconds(Duration.ofMillis(1).toNanos());
    m_glitchFilter.add(m_noteSensor);
    m_sensorThread = new SensorThread(m_noteSensor);

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
    m_rollerCurrentSignal = m_feederMotor.getSupplyCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        m_rollerPositionSignal,
        m_rollerVelocitySignal,
        m_rollerAppliedVoltageSignal,
        m_rollerCurrentSignal);
    m_feederMotor.optimizeBusUtilization();

    if (kUseHighFrequencySensorPolling) m_sensorThread.start();
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
    inputs.topNoteSensorTripped =
        kUseHighFrequencySensorPolling
            ? m_sensorThread.getSensorTripped.getAsBoolean()
            : m_noteSensor.get();
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
