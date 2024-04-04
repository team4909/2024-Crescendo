package frc.robot.intake;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX m_frontRollersMotor, m_backRollersMotor;
  private final BaseTalon m_centeringMotors;

  private final StatusSignal<Double> m_topRollerVelocitySignal,
      m_topRollerAppliedVoltageSignal,
      m_topRollerCurrentSignal,
      m_bottomRollerVelocitySignal,
      m_bottomRollerAppliedVoltageSignal,
      m_bottomRollerCurrentSignal;

  public IntakeIOTalonFX() {
    m_frontRollersMotor = new TalonFX(22, "rio");
    m_backRollersMotor = new TalonFX(23, "rio");
    m_centeringMotors = new BaseTalon(24, "Talon SRX", "rio");
    final CurrentLimitsConfigs currentLimitsConfigs =
        new CurrentLimitsConfigs().withSupplyCurrentLimit(40.0).withSupplyCurrentLimitEnable(true);
    final MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);
    final TalonFXConfiguration rollerConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(currentLimitsConfigs)
            .withMotorOutput(motorOutputConfigs);
    m_frontRollersMotor.getConfigurator().apply(rollerConfigs);
    m_backRollersMotor.getConfigurator().apply(rollerConfigs);

    m_centeringMotors.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true, 40.0, 0.0, 1.0), 1000);
    m_centeringMotors.configVoltageCompSaturation(12.0);
    m_centeringMotors.enableVoltageCompensation(true);

    m_topRollerVelocitySignal = m_frontRollersMotor.getVelocity();
    m_topRollerAppliedVoltageSignal = m_frontRollersMotor.getMotorVoltage();
    m_topRollerCurrentSignal = m_frontRollersMotor.getSupplyCurrent();

    m_bottomRollerVelocitySignal = m_backRollersMotor.getVelocity();
    m_bottomRollerAppliedVoltageSignal = m_backRollersMotor.getMotorVoltage();
    m_bottomRollerCurrentSignal = m_backRollersMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        m_topRollerVelocitySignal,
        m_topRollerAppliedVoltageSignal,
        m_topRollerCurrentSignal,
        m_bottomRollerVelocitySignal,
        m_bottomRollerAppliedVoltageSignal,
        m_bottomRollerCurrentSignal);
    ParentDevice.optimizeBusUtilizationForAll(m_frontRollersMotor, m_backRollersMotor);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerMotorsConnected =
        BaseStatusSignal.refreshAll(
                m_topRollerVelocitySignal,
                m_topRollerAppliedVoltageSignal,
                m_topRollerCurrentSignal,
                m_bottomRollerVelocitySignal,
                m_bottomRollerAppliedVoltageSignal,
                m_bottomRollerCurrentSignal)
            .isOK();
    inputs.centeringMotorConnected = m_centeringMotors.getLastError() == ErrorCode.OK;

    inputs.topRollerVelocityRpm = m_topRollerVelocitySignal.getValue() * 60.0;
    inputs.topRollerAppliedVolts = m_topRollerAppliedVoltageSignal.getValue();
    inputs.topRollerCurrentAmps = m_topRollerCurrentSignal.getValue();

    inputs.bottomRollerVelocityRpm = m_bottomRollerVelocitySignal.getValue() * 60.0;
    inputs.bottomRollerAppliedVolts = m_bottomRollerAppliedVoltageSignal.getValue();
    inputs.bottomRollerCurrentAmps = m_bottomRollerCurrentSignal.getValue();

    inputs.centeringBagMotorsAppliedVolts = m_centeringMotors.getMotorOutputVoltage();
    inputs.centeringBagMotorsCurrentAmps = m_centeringMotors.getSupplyCurrent();
  }

  @Override
  public void setFrontRollersVoltage(double volts) {
    m_frontRollersMotor.setVoltage(volts);
  }

  @Override
  public void setBackRollersVoltage(double volts) {
    m_backRollersMotor.setVoltage(volts);
  }

  @Override
  public void setCenteringMotorsVoltage(double volts) {
    m_centeringMotors.set(ControlMode.PercentOutput, volts / 12.0);
  }

  @Override
  public void stopRollers() {
    m_frontRollersMotor.stopMotor();
    m_backRollersMotor.stopMotor();
    m_centeringMotors.set(ControlMode.PercentOutput, 0.0);
  }
}
