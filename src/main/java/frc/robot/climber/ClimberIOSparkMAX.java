package frc.robot.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

public class ClimberIOSparkMAX implements ClimberIO {

  private final double kWinchReduction = 1.0;
  private final CANSparkMax m_leftWinchMotor, m_rightWinchMotor;
  private final RelativeEncoder m_leftWinchEncoder, m_rightWinchEncoder;

  public ClimberIOSparkMAX() {

    m_leftWinchMotor = new CANSparkMax(9, CANSparkMax.MotorType.kBrushless);
    m_rightWinchMotor = new CANSparkMax(10, CANSparkMax.MotorType.kBrushless);

    m_leftWinchEncoder = m_leftWinchMotor.getEncoder();
    m_rightWinchEncoder = m_rightWinchMotor.getEncoder();

    m_leftWinchMotor.setIdleMode(IdleMode.kBrake);
    m_rightWinchMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.leftWinchPositionRad =
        Units.rotationsToRadians(m_leftWinchEncoder.getPosition() / kWinchReduction);
    inputs.leftWinchVelocityRadPerSec =
        Units.rotationsToRadians(m_leftWinchEncoder.getVelocity() / kWinchReduction);
    inputs.leftWinchAppliedVolts =
        m_leftWinchMotor.getAppliedOutput() * m_leftWinchMotor.getBusVoltage();
    inputs.leftWinchCurrentAmps = m_leftWinchMotor.getOutputCurrent();

    inputs.rightWinchPositionRad =
        Units.rotationsToRadians(m_rightWinchEncoder.getPosition() / kWinchReduction);
    inputs.rightWinchVelocityRadPerSec =
        Units.rotationsToRadians(m_rightWinchEncoder.getVelocity() / kWinchReduction);
    inputs.rightWinchAppliedVolts =
        m_rightWinchMotor.getAppliedOutput() * m_rightWinchMotor.getBusVoltage();
    inputs.rightWinchCurrentAmps = m_rightWinchMotor.getOutputCurrent();
  }

  @Override
  public void setLeftDutyCycle(double outputDutyCycle) {
    m_leftWinchMotor.set(outputDutyCycle);
  }

  @Override
  public void setRightDutyCycle(double outputDutyCycle) {
    m_rightWinchMotor.set(outputDutyCycle);
  }
}
