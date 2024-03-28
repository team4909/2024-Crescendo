package frc.robot.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ArmIOTalonFX implements ArmIO {

  // Offsets to the horizontal
  private final double kElbowRelativeEncoderOffsetRad = 0.558;
  private final double kWristRelativeEncoderOffsetRad = 2.5;
  private final TalonFX m_elbowLeftMotor,
      m_elbowRightFollowerMotor,
      m_wristLeftMotor,
      m_wristRightFollowerMotor;
  private final MotionMagicVoltage m_elbowControl, m_wristControl;
  private final StatusSignal<Double> m_elbowPositionSignal,
      m_elbowPositionSetpointSignal,
      m_elbowVelocitySignal,
      m_elbowAppliedVoltsSignal,
      m_elbowTorqueCurrentSignal,
      m_elbowStatorCurrentSignal,
      m_elbowFollowerStatorCurrentSignal,
      m_wristPositionSignal,
      m_wristPositionSetpointSignal,
      m_wristVelocitySignal,
      m_wristAppliedVoltsSignal,
      m_wristTorqueCurrentSignal,
      m_wristStatorCurrentSignal,
      m_wristFollowerStatorCurrentSignal;

  public ArmIOTalonFX() {
    m_elbowLeftMotor = new TalonFX(15, Constants.kSuperstructureCanBus);
    m_elbowRightFollowerMotor = new TalonFX(17, Constants.kSuperstructureCanBus);
    m_wristLeftMotor = new TalonFX(16, Constants.kSuperstructureCanBus);
    m_wristRightFollowerMotor = new TalonFX(18, Constants.kSuperstructureCanBus);

    m_elbowLeftMotor.setPosition(-Units.radiansToRotations(kElbowRelativeEncoderOffsetRad), 1.0);
    m_wristLeftMotor.setPosition(Units.radiansToRotations(kWristRelativeEncoderOffsetRad), 1.0);

    final CurrentLimitsConfigs currentLimitsConfig = new CurrentLimitsConfigs();
    currentLimitsConfig.StatorCurrentLimit = 60.0;
    currentLimitsConfig.StatorCurrentLimitEnable = true;
    final TalonFXConfiguration elbowLeftMotorConfig = new TalonFXConfiguration();
    elbowLeftMotorConfig.CurrentLimits = currentLimitsConfig;
    elbowLeftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elbowLeftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elbowLeftMotorConfig.Feedback.SensorToMechanismRatio = ArmConstants.kElbowReduction;
    elbowLeftMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    elbowLeftMotorConfig.Slot0.kP = 62.122;
    elbowLeftMotorConfig.Slot0.kD = 10.118;
    elbowLeftMotorConfig.Slot0.kS = 0.74887;
    elbowLeftMotorConfig.Slot0.kV = 13.863;
    elbowLeftMotorConfig.Slot0.kA = 1.2605;
    elbowLeftMotorConfig.Slot0.kG = 0.37766;
    elbowLeftMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 0.5;
    elbowLeftMotorConfig.MotionMagic.MotionMagicAcceleration = 2.0;
    elbowLeftMotorConfig.Voltage.PeakForwardVoltage = 6.0;
    elbowLeftMotorConfig.Voltage.PeakReverseVoltage = -6.0;
    m_elbowLeftMotor.getConfigurator().apply(elbowLeftMotorConfig, 1.0);

    final TalonFXConfiguration wristLeftMotorConfig = new TalonFXConfiguration();
    wristLeftMotorConfig.CurrentLimits = currentLimitsConfig;
    wristLeftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wristLeftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristLeftMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
    wristLeftMotorConfig.Feedback.SensorToMechanismRatio = ArmConstants.kWristReduction;
    wristLeftMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    wristLeftMotorConfig.Slot0.kP = 22.742;
    wristLeftMotorConfig.Slot0.kD = 5.5071;
    wristLeftMotorConfig.Slot0.kS = 0.57211;
    wristLeftMotorConfig.Slot0.kV = 2.5239;
    wristLeftMotorConfig.Slot0.kA = 0.10939;
    wristLeftMotorConfig.Slot0.kG = 0.45457;
    wristLeftMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 0.5;
    wristLeftMotorConfig.MotionMagic.MotionMagicAcceleration = 2.0;
    wristLeftMotorConfig.Voltage.PeakForwardVoltage = 3.0;
    wristLeftMotorConfig.Voltage.PeakReverseVoltage = -3.0;
    m_wristLeftMotor.getConfigurator().apply(wristLeftMotorConfig, 1.0);

    final TalonFXConfiguration elbowRightMotorConfig = new TalonFXConfiguration();
    elbowRightMotorConfig.CurrentLimits = currentLimitsConfig;
    elbowRightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_elbowRightFollowerMotor.getConfigurator().apply(elbowRightMotorConfig, 1.0);
    m_elbowRightFollowerMotor.setControl(new Follower(m_elbowLeftMotor.getDeviceID(), true));

    final TalonFXConfiguration wristRightMotorConfig = new TalonFXConfiguration();
    wristRightMotorConfig.CurrentLimits = currentLimitsConfig;
    wristRightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_wristRightFollowerMotor.getConfigurator().apply(wristRightMotorConfig, 1.0);
    m_wristRightFollowerMotor.setControl(new Follower(m_wristLeftMotor.getDeviceID(), true));

    m_elbowPositionSignal = m_elbowLeftMotor.getPosition();
    m_elbowPositionSetpointSignal = m_elbowLeftMotor.getClosedLoopReference();
    m_elbowVelocitySignal = m_elbowLeftMotor.getVelocity();
    m_elbowAppliedVoltsSignal = m_elbowLeftMotor.getMotorVoltage();
    m_elbowTorqueCurrentSignal = m_elbowLeftMotor.getTorqueCurrent();
    m_elbowStatorCurrentSignal = m_elbowLeftMotor.getStatorCurrent();
    m_elbowFollowerStatorCurrentSignal = m_elbowRightFollowerMotor.getStatorCurrent();
    m_wristPositionSignal = m_wristLeftMotor.getPosition();
    m_wristPositionSetpointSignal = m_wristLeftMotor.getClosedLoopReference();
    m_wristVelocitySignal = m_wristLeftMotor.getVelocity();
    m_wristAppliedVoltsSignal = m_wristLeftMotor.getMotorVoltage();
    m_wristTorqueCurrentSignal = m_wristLeftMotor.getTorqueCurrent();
    m_wristStatorCurrentSignal = m_wristLeftMotor.getStatorCurrent();
    m_wristFollowerStatorCurrentSignal = m_wristRightFollowerMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        m_elbowPositionSignal,
        m_elbowPositionSetpointSignal,
        m_elbowVelocitySignal,
        m_elbowAppliedVoltsSignal,
        m_elbowTorqueCurrentSignal,
        m_elbowStatorCurrentSignal,
        m_elbowFollowerStatorCurrentSignal,
        m_wristPositionSignal,
        m_wristPositionSetpointSignal,
        m_wristVelocitySignal,
        m_wristAppliedVoltsSignal,
        m_wristTorqueCurrentSignal,
        m_wristStatorCurrentSignal,
        m_wristFollowerStatorCurrentSignal);

    ParentDevice.optimizeBusUtilizationForAll(
        m_elbowLeftMotor, m_elbowRightFollowerMotor, m_wristLeftMotor, m_wristRightFollowerMotor);

    m_elbowControl =
        new MotionMagicVoltage(0.0, true, 0.0, 0, false, false, false).withUpdateFreqHz(0.0);
    m_wristControl =
        new MotionMagicVoltage(0.0, true, 0.0, 0, false, false, false).withUpdateFreqHz(0.0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.allMotorsConnected =
        BaseStatusSignal.refreshAll(
                m_elbowPositionSignal,
                m_elbowPositionSetpointSignal,
                m_elbowVelocitySignal,
                m_elbowAppliedVoltsSignal,
                m_elbowTorqueCurrentSignal,
                m_elbowStatorCurrentSignal,
                m_elbowFollowerStatorCurrentSignal,
                m_wristPositionSignal,
                m_wristPositionSetpointSignal,
                m_wristVelocitySignal,
                m_wristAppliedVoltsSignal,
                m_wristTorqueCurrentSignal,
                m_wristStatorCurrentSignal,
                m_wristFollowerStatorCurrentSignal)
            .isOK();

    inputs.elbowPositionRot = m_elbowPositionSignal.getValue();
    inputs.elbowPositionSetpointRot = m_elbowPositionSetpointSignal.getValue();
    inputs.elbowVelocityRps = m_elbowVelocitySignal.getValue();
    inputs.elbowAppliedVolts = m_elbowAppliedVoltsSignal.getValue();
    inputs.elbowTorqueCurrentAmps = m_elbowTorqueCurrentSignal.getValue();
    inputs.elbowStatorCurrentAmps =
        new double[] {
          m_elbowStatorCurrentSignal.getValue(), m_elbowFollowerStatorCurrentSignal.getValue()
        };

    inputs.wristPositionRot = m_wristPositionSignal.getValue();
    inputs.wristPositionSetpointRot = m_wristPositionSetpointSignal.getValue();
    inputs.wristVelocityRps = m_wristVelocitySignal.getValue();
    inputs.wristAppliedVolts = m_wristAppliedVoltsSignal.getValue();
    inputs.wristTorqueCurrentAmps = m_wristTorqueCurrentSignal.getValue();
    inputs.wristStatorCurrentAmps =
        new double[] {
          m_wristStatorCurrentSignal.getValue(), m_wristFollowerStatorCurrentSignal.getValue()
        };
  }

  @Override
  public void setElbowRotations(double angleRot) {
    m_elbowLeftMotor.setControl(m_elbowControl.withPosition(angleRot));
  }

  @Override
  public void setWristRotations(double angleRot) {
    m_wristLeftMotor.setControl(m_wristControl.withPosition(angleRot));
  }

  @Override
  public void setElbowVoltage(double volts) {
    m_elbowLeftMotor.setVoltage(volts);
  }

  @Override
  public void setWristVoltage(double volts) {
    m_wristLeftMotor.setVoltage(volts);
  }

  @Override
  public void setElbowCurrent(double amps) {
    m_elbowLeftMotor.setControl(new TorqueCurrentFOC(amps));
  }

  @Override
  public void setWristCurrent(double amps) {
    m_wristLeftMotor.setControl(new TorqueCurrentFOC(amps));
  }

  @Override
  public void stop() {
    m_elbowLeftMotor.setVoltage(0.0);
    m_wristLeftMotor.setVoltage(0.0);
  }

  @Override
  public void setBrakeMode(boolean enableBrakeMode) {
    final NeutralModeValue neutralModeValue =
        enableBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    m_elbowLeftMotor.setNeutralMode(neutralModeValue);
    m_wristLeftMotor.setNeutralMode(neutralModeValue);
    m_elbowRightFollowerMotor.setNeutralMode(neutralModeValue);
    m_wristRightFollowerMotor.setNeutralMode(neutralModeValue);
  }
}
