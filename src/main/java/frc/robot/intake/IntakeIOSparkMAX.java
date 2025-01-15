package frc.robot.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

public class IntakeIOSparkMAX implements IntakeIO {

  private final CANSparkMax m_topRollerMotor, m_bottomRollerMotor, m_centeringMotors;
  private final RelativeEncoder m_topRollerEncoder, m_bottomRollerEncoder;

  public IntakeIOSparkMAX() {
    m_topRollerMotor = new CANSparkMax(5, CANSparkMax.MotorType.kBrushless);
    m_bottomRollerMotor = new CANSparkMax(6, CANSparkMax.MotorType.kBrushless);
    m_centeringMotors = new CANSparkMax(8, CANSparkMax.MotorType.kBrushed);

    m_topRollerEncoder = m_topRollerMotor.getEncoder();
    m_bottomRollerEncoder = m_bottomRollerMotor.getEncoder();

    m_topRollerMotor.restoreFactoryDefaults();
    m_bottomRollerMotor.restoreFactoryDefaults();

    m_topRollerMotor.setSmartCurrentLimit(40);
    m_bottomRollerMotor.setSmartCurrentLimit(40);
    m_centeringMotors.setSmartCurrentLimit(40);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.topRollerVelocityRpm =
        Units.rotationsPerMinuteToRadiansPerSecond(m_topRollerEncoder.getVelocity());
    inputs.topRollerAppliedVolts =
        m_topRollerMotor.getAppliedOutput() * m_topRollerMotor.getBusVoltage();
    inputs.topRollerCurrentAmps = m_topRollerMotor.getOutputCurrent();

    inputs.bottomRollerVelocityRpm =
        Units.rotationsPerMinuteToRadiansPerSecond(m_bottomRollerEncoder.getVelocity());
    inputs.bottomRollerAppliedVolts =
        m_bottomRollerMotor.getAppliedOutput() * m_bottomRollerMotor.getBusVoltage();
    inputs.bottomRollerCurrentAmps = m_bottomRollerMotor.getOutputCurrent();

    inputs.centeringBagMotorsAppliedVolts =
        m_centeringMotors.getAppliedOutput() * m_centeringMotors.getBusVoltage();
    inputs.bottomRollerCurrentAmps = m_centeringMotors.getOutputCurrent();

    inputs.rollerMotorsConnected =
        m_topRollerMotor.getLastError() == REVLibError.kOk
            && m_bottomRollerMotor.getLastError() == REVLibError.kOk;
    inputs.centeringMotorConnected = m_centeringMotors.getLastError() == REVLibError.kOk;
  }

  @Override
  public void setFrontRollersVoltage(double volts) {
    m_topRollerMotor.setVoltage(volts);
  }

  @Override
  public void setBackRollersVoltage(double volts) {
    m_bottomRollerMotor.setVoltage(volts);
  }

  @Override
  public void setCenteringMotorsVoltage(double volts) {
    m_centeringMotors.setVoltage(volts);
  }

  @Override
  public void stopRollers() {
    m_topRollerMotor.stopMotor();
    m_bottomRollerMotor.stopMotor();
    m_centeringMotors.stopMotor();
  }
}
