package frc.robot.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class ArmIOTalonFX implements ArmIO {

  // Offsets to the horizontal
  private final double kElbowRelativeEncoderOffsetRad = 0.558; // .558;
  private final double kWristRelativeEncoderOffsetRad = 2.5; // 2.5;
  private final boolean kInvertWristAbsoluteEncoder = false;
  private final boolean kInvertElbowAbsoluteEncoder = false;
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

  private final Rotation2d elbowAbsoluteEncoderOffset = new Rotation2d(2.471);
  private final Rotation2d wristAbsoluteEncoderOffset = new Rotation2d(0.646);
  //   private final Rotation2d elbowAbsoluteEncoderOffset;
  //   private final Rotation2d wristAbsoluteEncoderOffset;

  public ArmIOTalonFX() {
    m_elbowLeftMotor = new TalonFX(15, Constants.kSuperstructureCanBus);
    m_elbowRightFollowerMotor = new TalonFX(17, Constants.kSuperstructureCanBus);
    m_wristLeftMotor = new TalonFX(16, Constants.kSuperstructureCanBus);
    m_wristRightFollowerMotor = new TalonFX(18, Constants.kSuperstructureCanBus);

    m_elbowAbsoluteEncoder = new DutyCycleEncoder(8);
    m_wristAbsoluteEncoder = new DutyCycleEncoder(9);
    m_elbowAbsoluteEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
    m_wristAbsoluteEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);

    m_elbowLeftMotor.setPosition(0.0);
    m_wristLeftMotor.setPosition(0.0);
    m_elbowRightFollowerMotor.setPosition(0.0);
    m_wristRightFollowerMotor.setPosition(0.0);

    final CurrentLimitsConfigs currentLimitsConfig = new CurrentLimitsConfigs();
    currentLimitsConfig.SupplyCurrentLimit = 80.0;
    currentLimitsConfig.SupplyCurrentLimitEnable = true;
    final TalonFXConfiguration elbowLeftMotorConfig = new TalonFXConfiguration();
    m_elbowLeftMotor.getConfigurator().apply(elbowLeftMotorConfig);
    elbowLeftMotorConfig.CurrentLimits = currentLimitsConfig;
    elbowLeftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elbowLeftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elbowLeftMotorConfig.Voltage.PeakReverseVoltage = -4.0;
    m_elbowLeftMotor.getConfigurator().apply(elbowLeftMotorConfig);

    final TalonFXConfiguration wristLeftMotorConfig = new TalonFXConfiguration();
    m_wristLeftMotor.getConfigurator().apply(wristLeftMotorConfig);
    wristLeftMotorConfig.CurrentLimits = currentLimitsConfig;
    wristLeftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wristLeftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_wristLeftMotor.getConfigurator().apply(wristLeftMotorConfig);

    final TalonFXConfiguration elbowRightMotorConfig = new TalonFXConfiguration();
    m_elbowRightFollowerMotor.getConfigurator().apply(elbowRightMotorConfig);
    elbowRightMotorConfig.CurrentLimits = currentLimitsConfig;
    elbowRightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_elbowRightFollowerMotor.getConfigurator().apply(elbowRightMotorConfig);
    m_elbowRightFollowerMotor.setControl(new Follower(m_elbowLeftMotor.getDeviceID(), true));

    final TalonFXConfiguration wristRightMotorConfig = new TalonFXConfiguration();
    m_wristRightFollowerMotor.getConfigurator().apply(wristRightMotorConfig);
    wristRightMotorConfig.CurrentLimits = currentLimitsConfig;
    wristRightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
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

    m_elbowControl = new VoltageOut(0.0, true, false, false, false).withUpdateFreqHz(0.0);
    m_wristControl = new VoltageOut(0.0, true, false, false, false).withUpdateFreqHz(0.0);
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

    inputs.elbowEncoderRaw = m_elbowPositionSignal.getValue();
    inputs.wristEncoderRaw = m_wristPositionSignal.getValue();
    inputs.elbowAbsolutePositionRad =
        MathUtil.angleModulus(
            Units.rotationsToRadians(m_elbowAbsoluteEncoder.get() / ArmModel.kElbowChainReduction)
                    * (kInvertElbowAbsoluteEncoder ? -1 : 1)
                - elbowAbsoluteEncoderOffset.getRadians());
    inputs.elbowAbsoluteEncoderConnected = m_elbowAbsoluteEncoder.isConnected();
    inputs.elbowRelativePositionRad =
        MathUtil.angleModulus(
            Units.rotationsToRadians(
                    m_elbowPositionSignal.getValue() / ArmModel.kElbowFinalReduction)
                - kElbowRelativeEncoderOffsetRad);
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
        MathUtil.angleModulus(
            Units.rotationsToRadians(
                    m_wristPositionSignal.getValue() / ArmModel.kWristFinalReduction)
                + kWristRelativeEncoderOffsetRad);
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

  public void setBrakeMode(boolean enableBrakeMode) {
    final NeutralModeValue neutralModeValue =
        enableBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    m_elbowLeftMotor.setNeutralMode(neutralModeValue);
    m_wristLeftMotor.setNeutralMode(neutralModeValue);
    m_elbowRightFollowerMotor.setNeutralMode(neutralModeValue);
    m_wristRightFollowerMotor.setNeutralMode(neutralModeValue);
  }
}
