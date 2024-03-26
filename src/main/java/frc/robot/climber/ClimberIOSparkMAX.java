package frc.robot.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class ClimberIOSparkMAX implements ClimberIO {

  private final CANSparkMax m_leftWinchMotor, m_rightWinchMotor;
  private final RelativeEncoder m_leftWinchEncoder, m_rightWinchEncoder;

  public ClimberIOSparkMAX() {

    m_leftWinchMotor = new CANSparkMax(9, CANSparkMax.MotorType.kBrushless);
    m_rightWinchMotor = new CANSparkMax(10, CANSparkMax.MotorType.kBrushless);

    m_leftWinchMotor.restoreFactoryDefaults();
    m_rightWinchMotor.restoreFactoryDefaults();

    m_leftWinchEncoder = m_leftWinchMotor.getEncoder();
    m_rightWinchEncoder = m_rightWinchMotor.getEncoder();

    m_leftWinchMotor.setIdleMode(IdleMode.kBrake);
    m_rightWinchMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.leftWinchPositionRot = m_leftWinchEncoder.getPosition() / Climber.kWinchReduction;
    inputs.leftWinchVelocityRpm = m_leftWinchEncoder.getVelocity() / Climber.kWinchReduction;
    inputs.leftWinchAppliedVolts =
        m_leftWinchMotor.getAppliedOutput() * m_leftWinchMotor.getBusVoltage();
    inputs.leftWinchCurrentAmps = m_leftWinchMotor.getOutputCurrent();

    inputs.rightWinchPositionRot = m_rightWinchEncoder.getPosition() / Climber.kWinchReduction;
    inputs.rightWinchVelocityRpm = m_rightWinchEncoder.getVelocity() / Climber.kWinchReduction;
    inputs.rightWinchAppliedVolts =
        m_rightWinchMotor.getAppliedOutput() * m_rightWinchMotor.getBusVoltage();
    inputs.rightWinchCurrentAmps = m_rightWinchMotor.getOutputCurrent();
  }

  @Override
  public void setLeftVoltage(double outputDutyCycle) {
    m_leftWinchMotor.set(outputDutyCycle);
  }

  @Override
  public void setRightVoltage(double outputDutyCycle) {
    m_rightWinchMotor.set(outputDutyCycle);
  }
}
