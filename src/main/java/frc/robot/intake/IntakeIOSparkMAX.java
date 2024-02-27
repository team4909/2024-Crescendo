package frc.robot.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

public class IntakeIOSparkMAX implements IntakeIO {

  private final double kRollerReduction = 1.0;
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
    centeringMotors.setSmartCurrentLimit(15);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.topRollerVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            m_topRollerEncoder.getVelocity() / kRollerReduction);
    inputs.topRollerAppliedVolts =
        topRollerMotor.getAppliedOutput() * topRollerMotor.getBusVoltage();
    inputs.topRollerCurrentAmps = topRollerMotor.getOutputCurrent();

    inputs.bottomRollerVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            m_bottomRollerEncoder.getVelocity() / kRollerReduction);
    inputs.bottomRollerAppliedVolts =
        bottomRollerMotor.getAppliedOutput() * bottomRollerMotor.getBusVoltage();
    inputs.bottomRollerCurrentAmps = bottomRollerMotor.getOutputCurrent();

    inputs.centeringBagMotorsAppliedVolts =
        centeringMotors.getAppliedOutput() * centeringMotors.getBusVoltage();
    inputs.bottomRollerCurrentAmps = centeringMotors.getOutputCurrent();
  }

  @Override
  public void setTopRollerDutyCycle(double outputDutyCycle) {
    topRollerMotor.set(outputDutyCycle);
  }

  @Override
  public void setBottomRollerDutyCycle(double outputDutyCycle) {
    bottomRollerMotor.set(outputDutyCycle);
  }

  @Override
  public void setCenteringMotorsDutyCycle(double outputDutyCycle) {
    centeringMotors.set(outputDutyCycle);
  }
}
