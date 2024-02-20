package frc.robot.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class ArmIOTalonFX implements ArmIO {

  // Offsets to the horizontal
  private final double kElbowRelativeEncoderOffsetRad = 0.0;
  private final double kWristRelativeEncoderOffsetRad = 0.0;
  private final boolean kInvertWristAbsoluteEncoder = false;
  private final boolean kInvertElbowAbsoluteEncoder = false;
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

  private final Rotation2d elbowAbsoluteEncoderOffset = new Rotation2d(2.671);
  private final Rotation2d wristAbsoluteEncoderOffset = new Rotation2d(-0.285);

  public ArmIOTalonFX() {
    m_elbowLeftMotor = new TalonFX(15, Constants.kOtherCanBus);
    m_elbowRightFollowerMotor = new TalonFX(17, Constants.kOtherCanBus);
    m_wristLeftMotor = new TalonFX(16, Constants.kOtherCanBus);
    m_wristRightFollowerMotor = new TalonFX(18, Constants.kOtherCanBus);

    m_elbowAbsoluteEncoder = new DutyCycleEncoder(8);
    m_wristAbsoluteEncoder = new DutyCycleEncoder(9);
    m_elbowAbsoluteEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
    m_wristAbsoluteEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);

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
    elbowLeftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_elbowLeftMotor.getConfigurator().apply(elbowLeftMotorConfig);

    final TalonFXConfiguration wristLeftMotorConfig = new TalonFXConfiguration();
    m_wristLeftMotor.getConfigurator().apply(wristLeftMotorConfig);
    wristLeftMotorConfig.CurrentLimits = currentLimitsConfig;
    wristLeftMotorConfig.TorqueCurrent = torqueCurrentConfig;
    wristLeftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_wristLeftMotor.getConfigurator().apply(wristLeftMotorConfig);

    final TalonFXConfiguration elbowRightMotorConfig = new TalonFXConfiguration();
    m_elbowRightFollowerMotor.getConfigurator().apply(elbowRightMotorConfig);
    elbowRightMotorConfig.CurrentLimits = currentLimitsConfig;
    elbowRightMotorConfig.TorqueCurrent = torqueCurrentConfig;
    m_elbowRightFollowerMotor.getConfigurator().apply(elbowRightMotorConfig);
    m_elbowRightFollowerMotor.setControl(new Follower(m_elbowLeftMotor.getDeviceID(), true));

    final TalonFXConfiguration wristRightMotorConfig = new TalonFXConfiguration();
    m_wristRightFollowerMotor.getConfigurator().apply(wristRightMotorConfig);
    wristRightMotorConfig.CurrentLimits = currentLimitsConfig;
    wristRightMotorConfig.TorqueCurrent = torqueCurrentConfig;
    m_wristRightFollowerMotor.getConfigurator().apply(wristRightMotorConfig);
    m_wristRightFollowerMotor.setControl(new Follower(m_wristLeftMotor.getDeviceID(), true));

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

    ParentDevice.optimizeBusUtilizationForAll(
        m_elbowLeftMotor, m_elbowRightFollowerMotor, m_wristLeftMotor, m_wristRightFollowerMotor);

    m_elbowControl = new VoltageOut(0.0, true, true, false, false);
    m_wristControl = new VoltageOut(0.0, true, true, false, false);
  }

  public void updateInputs(ArmIOInputs inputs) {
    inputs.allMotorsConnected =
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
                m_wristFollowerCurrentSignal)
            .equals(StatusCode.OK);

    inputs.elbowAbsolutePositionRad =
        MathUtil.angleModulus(
            Units.rotationsToRadians(m_elbowAbsoluteEncoder.get() / ArmModel.kElbowChainReduction)
                    * (kInvertElbowAbsoluteEncoder ? -1 : 1)
                - elbowAbsoluteEncoderOffset.getRadians());
    inputs.elbowAbsoluteEncoderConnected = m_elbowAbsoluteEncoder.isConnected();
    inputs.elbowRelativePositionRad =
        Units.rotationsToRadians(m_elbowPositionSignal.getValue() / ArmModel.kElbowFinalReduction);
    inputs.elbowVelocityRadPerSec =
        Units.rotationsToRadians(m_elbowVelocitySignal.getValue() / ArmModel.kElbowFinalReduction);
    inputs.elbowAppliedVolts = m_elbowAppliedVoltsSignal.getValue();
    inputs.elbowCurrentAmps =
        new double[] {m_elbowCurrentSignal.getValue(), m_elbowFollowerCurrentSignal.getValue()};

    inputs.wristAbsolutePositionRad =
        MathUtil.angleModulus(
            Units.rotationsToRadians(m_wristAbsoluteEncoder.get() / ArmModel.kWristChainReduction)
                    * (kInvertWristAbsoluteEncoder ? -1 : 1)
                - wristAbsoluteEncoderOffset.getRadians());

    inputs.wristAbsoluteEncoderConnected = m_wristAbsoluteEncoder.isConnected();
    inputs.wristRelativePositionRad =
        Units.rotationsToRadians(m_wristPositionSignal.getValue() / ArmModel.kWristFinalReduction);
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
