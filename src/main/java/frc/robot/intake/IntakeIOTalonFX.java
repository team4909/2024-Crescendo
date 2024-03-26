package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX topRollerMotor, bottomRollerMotor;
  private final TalonSRX centeringMotors;

  private final StatusSignal<Double> m_topRollerVelocitySignal,
      m_topRollerAppliedVoltageSignal,
      m_topRollerCurrentSignal,
      m_bottomRollerVelocitySignal,
      m_bottomRollerAppliedVoltageSignal,
      m_bottomRollerCurrentSignal;

  public IntakeIOTalonFX() {
    topRollerMotor = new TalonFX(5);
    bottomRollerMotor = new TalonFX(6);
    centeringMotors = new TalonSRX(8);

    topRollerMotor.getConfigurator().apply(new TalonFXConfiguration());
    bottomRollerMotor.getConfigurator().apply(new TalonFXConfiguration());
    final CurrentLimitsConfigs currentLimits =
        new CurrentLimitsConfigs().withSupplyCurrentLimit(40.0).withSupplyCurrentLimitEnable(true);
    topRollerMotor.getConfigurator().apply(currentLimits);
    bottomRollerMotor.getConfigurator().apply(currentLimits);

    centeringMotors.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true, 40.0, 0.0, 1.0));
    centeringMotors.configVoltageCompSaturation(12.0);
    centeringMotors.enableVoltageCompensation(true);

    m_topRollerVelocitySignal = topRollerMotor.getVelocity();
    m_topRollerAppliedVoltageSignal = topRollerMotor.getMotorVoltage();
    m_topRollerCurrentSignal = topRollerMotor.getSupplyCurrent();

    m_bottomRollerVelocitySignal = bottomRollerMotor.getVelocity();
    m_bottomRollerAppliedVoltageSignal = bottomRollerMotor.getMotorVoltage();
    m_bottomRollerCurrentSignal = bottomRollerMotor.getSupplyCurrent();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.topRollerVelocityRpm = m_topRollerVelocitySignal.getValue() * 60.0;
    inputs.topRollerAppliedVolts = m_topRollerAppliedVoltageSignal.getValue();
    inputs.topRollerCurrentAmps = m_topRollerCurrentSignal.getValue();

    inputs.bottomRollerVelocityRpm = m_bottomRollerVelocitySignal.getValue() * 60.0;
    inputs.bottomRollerAppliedVolts = m_bottomRollerAppliedVoltageSignal.getValue();
    inputs.bottomRollerCurrentAmps = m_bottomRollerCurrentSignal.getValue();

    inputs.centeringBagMotorsAppliedVolts = centeringMotors.getMotorOutputVoltage();
    inputs.centeringBagMotorsCurrentAmps = centeringMotors.getSupplyCurrent();
  }

  @Override
  public void setTopRollerVoltage(double volts) {
    topRollerMotor.setVoltage(volts);
  }

  @Override
  public void setBottomRollerVoltage(double volts) {
    bottomRollerMotor.setVoltage(volts);
  }

  @Override
  public void setCenteringMotorsVoltage(double volts) {
    centeringMotors.set(TalonSRXControlMode.PercentOutput, (volts / 12.0) * 100.0);
  }

  @Override
  public void stopRollers() {
    topRollerMotor.stopMotor();
    bottomRollerMotor.stopMotor();
    centeringMotors.set(TalonSRXControlMode.PercentOutput, 0.0);
  }
}
