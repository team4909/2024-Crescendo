package frc.robot.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

public class IntakeIOSparkMAX implements IntakeIO {

  private final CANSparkMax topRollerMotor, bottomRollerMotor, centeringMotors;
  private final RelativeEncoder m_topRollerEncoder, m_bottomRollerEncoder;

  public IntakeIOSparkMAX() {
    topRollerMotor = new CANSparkMax(5, CANSparkMax.MotorType.kBrushless);
    bottomRollerMotor = new CANSparkMax(6, CANSparkMax.MotorType.kBrushless);
    centeringMotors = new CANSparkMax(8, CANSparkMax.MotorType.kBrushed);

    m_topRollerEncoder = topRollerMotor.getEncoder();
    m_bottomRollerEncoder = bottomRollerMotor.getEncoder();

    topRollerMotor.restoreFactoryDefaults();
    bottomRollerMotor.restoreFactoryDefaults();

    topRollerMotor.setSmartCurrentLimit(40);
    bottomRollerMotor.setSmartCurrentLimit(40);
    centeringMotors.setSmartCurrentLimit(40);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.topRollerVelocityRpm =
        Units.rotationsPerMinuteToRadiansPerSecond(m_topRollerEncoder.getVelocity());
    inputs.topRollerAppliedVolts =
        topRollerMotor.getAppliedOutput() * topRollerMotor.getBusVoltage();
    inputs.topRollerCurrentAmps = topRollerMotor.getOutputCurrent();

    inputs.bottomRollerVelocityRpm =
        Units.rotationsPerMinuteToRadiansPerSecond(m_bottomRollerEncoder.getVelocity());
    inputs.bottomRollerAppliedVolts =
        bottomRollerMotor.getAppliedOutput() * bottomRollerMotor.getBusVoltage();
    inputs.bottomRollerCurrentAmps = bottomRollerMotor.getOutputCurrent();

    inputs.centeringBagMotorsAppliedVolts =
        centeringMotors.getAppliedOutput() * centeringMotors.getBusVoltage();
    inputs.bottomRollerCurrentAmps = centeringMotors.getOutputCurrent();
  }

  @Override
  public void setFrontRollersVoltage(double volts) {
    topRollerMotor.setVoltage(volts);
  }

  @Override
  public void setBackRollersVoltage(double volts) {
    bottomRollerMotor.setVoltage(volts);
  }

  @Override
  public void setCenteringMotorsVoltage(double volts) {
    centeringMotors.setVoltage(volts);
  }

  @Override
  public void stopRollers() {
    topRollerMotor.stopMotor();
    bottomRollerMotor.stopMotor();
    centeringMotors.stopMotor();
  }
}
