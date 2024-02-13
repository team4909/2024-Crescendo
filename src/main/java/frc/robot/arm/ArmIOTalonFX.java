package frc.robot.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class ArmIOTalonFX implements ArmIO {

  private final double kCurrentLimitAmps = 80.0;
  private final TalonFX m_elbowLeftMotor,
      m_elbowRightFollowerMotor,
      m_wristLeftMotor,
      m_wristRightFollowerMotor;
  private final DutyCycleEncoder m_elbowAbsoluteEncoder, m_wristAbsoluteEncoder;
  private final VoltageOut m_elbowControl, m_wristControl;
  private final StatusSignal<Double> m_elbowPositionSignal,
      m_elbowVelocitySignal,
      m_elbowAppliedVoltsSignal,
      m_elbowCurrentSignal,
      m_elbowFollowerCurrentSignal,
      m_wristPositionSignal,
      m_wristVelocitySignal,
      m_wristAppliedVoltsSignal,
      m_wristCurrentSignal,
      m_wristFollowerCurrentSignal;

  public ArmIOTalonFX() {
    m_elbowLeftMotor = new TalonFX(9, Constants.kCanBusName);
    m_elbowRightFollowerMotor = new TalonFX(10, Constants.kCanBusName);
    m_wristLeftMotor = new TalonFX(12, Constants.kCanBusName);
    m_wristRightFollowerMotor = new TalonFX(11, Constants.kCanBusName);

    m_elbowAbsoluteEncoder = new DutyCycleEncoder(0);
    m_wristAbsoluteEncoder = new DutyCycleEncoder(1);

    m_elbowAbsoluteEncoder.setPositionOffset(0.0);
    m_wristAbsoluteEncoder.setPositionOffset(0.0);

    final CurrentLimitsConfigs currentLimitsConfig = new CurrentLimitsConfigs();
    currentLimitsConfig.StatorCurrentLimit = kCurrentLimitAmps;
    currentLimitsConfig.StatorCurrentLimitEnable = true;
    final TorqueCurrentConfigs torqueCurrentConfig = new TorqueCurrentConfigs();
    torqueCurrentConfig.PeakForwardTorqueCurrent = kCurrentLimitAmps;
    torqueCurrentConfig.PeakReverseTorqueCurrent = -kCurrentLimitAmps;

    final TalonFXConfiguration elbowLeftMotorConfig = new TalonFXConfiguration();
    m_elbowLeftMotor.getConfigurator().apply(elbowLeftMotorConfig);
    elbowLeftMotorConfig.CurrentLimits = currentLimitsConfig;
    elbowLeftMotorConfig.TorqueCurrent = torqueCurrentConfig;

    m_elbowLeftMotor.getConfigurator().apply(elbowLeftMotorConfig);

    final TalonFXConfiguration wristLeftMotorConfig = new TalonFXConfiguration();
    m_wristLeftMotor.getConfigurator().apply(wristLeftMotorConfig);
    wristLeftMotorConfig.CurrentLimits = currentLimitsConfig;
    wristLeftMotorConfig.TorqueCurrent = torqueCurrentConfig;

    m_wristLeftMotor.getConfigurator().apply(wristLeftMotorConfig);

    final TalonFXConfiguration elbowRightMotorConfig = new TalonFXConfiguration();
    m_elbowRightFollowerMotor.getConfigurator().apply(elbowRightMotorConfig);
    elbowRightMotorConfig.CurrentLimits = currentLimitsConfig;
    elbowRightMotorConfig.TorqueCurrent = torqueCurrentConfig;
    m_elbowRightFollowerMotor.getConfigurator().apply(elbowRightMotorConfig);
    m_elbowRightFollowerMotor.setControl(new Follower(m_elbowLeftMotor.getDeviceID(), false));

    final TalonFXConfiguration wristRightMotorConfig = new TalonFXConfiguration();
    m_wristRightFollowerMotor.getConfigurator().apply(wristRightMotorConfig);
    wristRightMotorConfig.CurrentLimits = currentLimitsConfig;
    wristRightMotorConfig.TorqueCurrent = torqueCurrentConfig;
    m_wristRightFollowerMotor.getConfigurator().apply(wristRightMotorConfig);
    m_wristRightFollowerMotor.setControl(new Follower(m_wristLeftMotor.getDeviceID(), false));

    m_elbowPositionSignal = m_elbowLeftMotor.getPosition();
    m_elbowVelocitySignal = m_elbowLeftMotor.getVelocity();
    m_elbowAppliedVoltsSignal = m_elbowLeftMotor.getMotorVoltage();
    m_elbowCurrentSignal = m_elbowLeftMotor.getStatorCurrent();
    m_elbowFollowerCurrentSignal = m_elbowRightFollowerMotor.getStatorCurrent();
    m_wristPositionSignal = m_wristLeftMotor.getPosition();
    m_wristVelocitySignal = m_wristLeftMotor.getVelocity();
    m_wristAppliedVoltsSignal = m_wristLeftMotor.getMotorVoltage();
    m_wristCurrentSignal = m_wristLeftMotor.getStatorCurrent();
    m_wristFollowerCurrentSignal = m_wristRightFollowerMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        m_elbowPositionSignal,
        m_elbowVelocitySignal,
        m_elbowAppliedVoltsSignal,
        m_elbowCurrentSignal,
        m_elbowFollowerCurrentSignal,
        m_wristPositionSignal,
        m_wristVelocitySignal,
        m_wristAppliedVoltsSignal,
        m_wristCurrentSignal,
        m_wristFollowerCurrentSignal);

    m_elbowLeftMotor.optimizeBusUtilization();
    m_elbowRightFollowerMotor.optimizeBusUtilization();
    m_wristLeftMotor.optimizeBusUtilization();
    m_wristRightFollowerMotor.optimizeBusUtilization();

    m_elbowLeftMotor.setPosition(m_elbowAbsoluteEncoder.get());
    m_wristLeftMotor.setPosition(m_wristAbsoluteEncoder.get());
    m_elbowControl = new VoltageOut(0.0, true, true, false, false);
    m_wristControl = new VoltageOut(0.0, true, true, false, false);
  }

  public void updateInputs(ArmIOInputs inputs) {

    BaseStatusSignal.refreshAll(
        m_elbowPositionSignal,
        m_elbowVelocitySignal,
        m_elbowAppliedVoltsSignal,
        m_elbowCurrentSignal,
        m_elbowFollowerCurrentSignal,
        m_wristPositionSignal,
        m_wristVelocitySignal,
        m_wristAppliedVoltsSignal,
        m_wristCurrentSignal,
        m_wristFollowerCurrentSignal);

    inputs.elbowAbsolutePositionRad =
        Units.rotationsToRadians(m_elbowAbsoluteEncoder.get() / ArmModel.kElbowChainReduction);
    inputs.elbowAbsoluteEncoderConnected = m_elbowAbsoluteEncoder.isConnected();
    inputs.elbowPositionRad =
        Units.rotationsToRadians(
            BaseStatusSignal.getLatencyCompensatedValue(
                    m_elbowPositionSignal, m_elbowVelocitySignal)
                / ArmModel.kElbowFinalReduction);
    inputs.elbowVelocityRadPerSec =
        Units.rotationsToRadians(m_elbowVelocitySignal.getValue() / ArmModel.kElbowFinalReduction);
    inputs.elbowAppliedVolts = m_elbowAppliedVoltsSignal.getValue();
    inputs.elbowCurrentAmps =
        new double[] {m_elbowCurrentSignal.getValue(), m_elbowFollowerCurrentSignal.getValue()};

    inputs.wristAbsolutePositionRad =
        Units.rotationsToRadians(m_wristAbsoluteEncoder.get() / ArmModel.kWristChainReduction);
    inputs.wristAbsoluteEncoderConnected = m_wristAbsoluteEncoder.isConnected();
    inputs.wristPositionRad =
        Units.rotationsToRadians(
            BaseStatusSignal.getLatencyCompensatedValue(
                    m_wristPositionSignal, m_wristVelocitySignal)
                / ArmModel.kWristFinalReduction);
    inputs.wristVelocityRadPerSec =
        Units.rotationsToRadians(m_wristVelocitySignal.getValue() / ArmModel.kWristFinalReduction);
    inputs.wristAppliedVolts = m_wristAppliedVoltsSignal.getValue();
    inputs.wristCurrentAmps =
        new double[] {m_wristCurrentSignal.getValue(), m_wristFollowerCurrentSignal.getValue()};
  }

  public void setElbowVoltage(double volts) {
    m_elbowLeftMotor.setControl(m_elbowControl.withOutput(volts));
  }

  public void setWristVoltage(double volts) {
    m_wristLeftMotor.setControl(m_wristControl.withOutput(volts));
  }
}
