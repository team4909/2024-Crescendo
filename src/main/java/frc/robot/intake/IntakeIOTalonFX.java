package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX m_topRollerMotor, m_bottomRollerMotor;
  private final BaseTalon m_centeringMotors;

  private final StatusSignal<Double> m_topRollerVelocitySignal,
      m_topRollerAppliedVoltageSignal,
      m_topRollerCurrentSignal,
      m_bottomRollerVelocitySignal,
      m_bottomRollerAppliedVoltageSignal,
      m_bottomRollerCurrentSignal;

  public IntakeIOTalonFX() {
    m_topRollerMotor = new TalonFX(22, Constants.kSuperstructureCanBus);
    m_bottomRollerMotor = new TalonFX(23, Constants.kSuperstructureCanBus);
    m_centeringMotors = new BaseTalon(24, "Talon SRX", "rio");
    m_topRollerMotor.getConfigurator().apply(new TalonFXConfiguration());
    m_bottomRollerMotor.getConfigurator().apply(new TalonFXConfiguration());
    final CurrentLimitsConfigs currentLimits =
        new CurrentLimitsConfigs().withSupplyCurrentLimit(40.0).withSupplyCurrentLimitEnable(true);
    m_topRollerMotor.getConfigurator().apply(currentLimits);
    m_bottomRollerMotor.getConfigurator().apply(currentLimits);

    m_centeringMotors.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true, 40.0, 0.0, 1.0), 1000);
    m_centeringMotors.configVoltageCompSaturation(12.0);
    m_centeringMotors.enableVoltageCompensation(true);

    m_topRollerVelocitySignal = m_topRollerMotor.getVelocity();
    m_topRollerAppliedVoltageSignal = m_topRollerMotor.getMotorVoltage();
    m_topRollerCurrentSignal = m_topRollerMotor.getSupplyCurrent();

    m_bottomRollerVelocitySignal = m_bottomRollerMotor.getVelocity();
    m_bottomRollerAppliedVoltageSignal = m_bottomRollerMotor.getMotorVoltage();
    m_bottomRollerCurrentSignal = m_bottomRollerMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        m_topRollerVelocitySignal,
        m_topRollerAppliedVoltageSignal,
        m_topRollerCurrentSignal,
        m_bottomRollerVelocitySignal,
        m_bottomRollerAppliedVoltageSignal,
        m_bottomRollerCurrentSignal);
    ParentDevice.optimizeBusUtilizationForAll(m_topRollerMotor, m_bottomRollerMotor);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.allMotorsConnected =
        BaseStatusSignal.refreshAll(
                m_topRollerVelocitySignal,
                m_topRollerAppliedVoltageSignal,
                m_topRollerCurrentSignal,
                m_bottomRollerVelocitySignal,
                m_bottomRollerAppliedVoltageSignal,
                m_bottomRollerCurrentSignal)
            .isOK();

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
  public void setTopRollerVoltage(double volts) {
    m_topRollerMotor.setVoltage(volts);
  }

  @Override
  public void setBottomRollerVoltage(double volts) {
    m_bottomRollerMotor.setVoltage(volts);
  }

  @Override
  public void setCenteringMotorsVoltage(double volts) {
    m_centeringMotors.set(ControlMode.PercentOutput, volts / 12.0);
  }

  @Override
  public void stopRollers() {
    m_topRollerMotor.stopMotor();
    m_bottomRollerMotor.stopMotor();
    m_centeringMotors.set(ControlMode.PercentOutput, 0.0);
  }
}
