package frc.robot.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class ClimberIOTalonFX implements ClimberIO {

  private final TalonFX m_leftWinchMotor, m_rightWinchMotor;
  private final StatusSignal<Double> m_leftWinchPositionSignal,
      m_leftWinchVelocitySignal,
      m_leftWinchAppliedVoltageSignal,
      m_leftWinchCurrentSignal,
      m_rightWinchPositionSignal,
      m_rightWinchVelocitySignal,
      m_rightWinchAppliedVoltageSignal,
      m_rightWinchCurrentSignal;

  public ClimberIOTalonFX() {

    m_leftWinchMotor = new TalonFX(25, Constants.kDrivetrainCanBus);
    m_rightWinchMotor = new TalonFX(26, Constants.kDrivetrainCanBus);

    final FeedbackConfigs feedbackConfigs =
        new FeedbackConfigs().withSensorToMechanismRatio(Climber.kWinchReduction);
    final MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);

    m_leftWinchMotor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withFeedback(feedbackConfigs)
                .withMotorOutput(motorOutputConfigs));
    m_rightWinchMotor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withFeedback(feedbackConfigs)
                .withMotorOutput(motorOutputConfigs));

    m_leftWinchPositionSignal = m_leftWinchMotor.getPosition();
    m_leftWinchVelocitySignal = m_leftWinchMotor.getVelocity();
    m_leftWinchAppliedVoltageSignal = m_leftWinchMotor.getMotorVoltage();
    m_leftWinchCurrentSignal = m_leftWinchMotor.getSupplyCurrent();

    m_rightWinchPositionSignal = m_rightWinchMotor.getPosition();
    m_rightWinchVelocitySignal = m_rightWinchMotor.getVelocity();
    m_rightWinchAppliedVoltageSignal = m_rightWinchMotor.getMotorVoltage();
    m_rightWinchCurrentSignal = m_rightWinchMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        m_leftWinchPositionSignal,
        m_leftWinchVelocitySignal,
        m_leftWinchAppliedVoltageSignal,
        m_leftWinchCurrentSignal,
        m_rightWinchPositionSignal,
        m_rightWinchVelocitySignal,
        m_rightWinchAppliedVoltageSignal,
        m_rightWinchCurrentSignal);

    ParentDevice.optimizeBusUtilizationForAll(m_leftWinchMotor, m_rightWinchMotor);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.allMotorsConnected =
        BaseStatusSignal.refreshAll(
                m_leftWinchPositionSignal,
                m_leftWinchVelocitySignal,
                m_leftWinchAppliedVoltageSignal,
                m_leftWinchCurrentSignal,
                m_rightWinchPositionSignal,
                m_rightWinchVelocitySignal,
                m_rightWinchAppliedVoltageSignal,
                m_rightWinchCurrentSignal)
            .isOK();

    inputs.leftWinchPositionRot = m_leftWinchPositionSignal.getValue();
    inputs.leftWinchVelocityRpm = m_leftWinchVelocitySignal.getValue();
    inputs.leftWinchAppliedVolts = m_leftWinchAppliedVoltageSignal.getValue();
    inputs.leftWinchCurrentAmps = m_leftWinchCurrentSignal.getValue();

    inputs.rightWinchPositionRot = m_rightWinchPositionSignal.getValue();
    inputs.rightWinchVelocityRpm = m_rightWinchVelocitySignal.getValue();
    inputs.rightWinchAppliedVolts = m_rightWinchAppliedVoltageSignal.getValue();
    inputs.rightWinchCurrentAmps = m_rightWinchCurrentSignal.getValue();
  }

  @Override
  public void setLeftVoltage(double volts) {
    m_leftWinchMotor.setControl(new VoltageOut(volts).withUpdateFreqHz(0.0));
  }

  @Override
  public void setRightVoltage(double volts) {
    m_rightWinchMotor.setVoltage(volts);
  }
}
