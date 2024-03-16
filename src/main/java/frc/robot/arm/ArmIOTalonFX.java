package frc.robot.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.arm.Arm.ArmSetpoints;

public class ArmIOTalonFX implements ArmIO {

  // Offsets to the horizontal
  private final double kElbowRelativeEncoderOffsetRad = 0.558;
  private final double kWristRelativeEncoderOffsetRad = 2.5;
  private final TalonFX m_elbowLeftMotor,
      m_elbowRightFollowerMotor,
      m_wristLeftMotor,
      m_wristRightFollowerMotor;
  private final MotionMagicTorqueCurrentFOC m_elbowControl, m_wristControl;
  private final StatusSignal<Double> m_elbowPositionSignal,
      m_elbowPositionSetpointSignal,
      m_elbowVelocitySignal,
      m_elbowAppliedVoltsSignal,
      m_elbowCurrentSignal,
      m_elbowFollowerCurrentSignal,
      m_wristPositionSignal,
      m_wristPositionSetpointSignal,
      m_wristVelocitySignal,
      m_wristAppliedVoltsSignal,
      m_wristCurrentSignal,
      m_wristFollowerCurrentSignal;

  private final SingleJointedArmSim m_elbowSim =
      new SingleJointedArmSim(
          ArmConstants.kElbowGearbox,
          ArmConstants.kElbowReduction,
          ArmConstants.kElbowMoiKgMetersSq,
          ArmConstants.kElbowLengthMeters,
          ArmConstants.kElbowMinAngleRad,
          ArmConstants.kElbowMaxAngleRad,
          false,
          ArmSetpoints.kStowed.elbowAngle);
  private final SingleJointedArmSim m_wristSim =
      new SingleJointedArmSim(
          ArmConstants.kWristGearbox,
          ArmConstants.kWristReduction,
          ArmConstants.kWristMoiKgMetersSq,
          ArmConstants.kWristLengthMeters,
          ArmConstants.kWristMinAngleRad,
          ArmConstants.kWristMaxAngleRad,
          false,
          ArmSetpoints.kStowed.wristAngle);

  public ArmIOTalonFX() {
    m_elbowLeftMotor = new TalonFX(15, Constants.kSuperstructureCanBus);
    m_elbowRightFollowerMotor = new TalonFX(17, Constants.kSuperstructureCanBus);
    m_wristLeftMotor = new TalonFX(16, Constants.kSuperstructureCanBus);
    m_wristRightFollowerMotor = new TalonFX(18, Constants.kSuperstructureCanBus);

    // if (!Constants.kIsSim) {
    m_elbowLeftMotor.setPosition(-Units.radiansToRotations(kElbowRelativeEncoderOffsetRad));
    m_wristLeftMotor.setPosition(Units.radiansToRotations(kWristRelativeEncoderOffsetRad));
    // }

    final CurrentLimitsConfigs currentLimitsConfig = new CurrentLimitsConfigs();
    currentLimitsConfig.SupplyCurrentLimit = 80.0;
    currentLimitsConfig.SupplyCurrentLimitEnable = true;
    final TalonFXConfiguration elbowLeftMotorConfig = new TalonFXConfiguration();
    m_elbowLeftMotor.getConfigurator().apply(elbowLeftMotorConfig);
    elbowLeftMotorConfig.CurrentLimits = currentLimitsConfig;
    elbowLeftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elbowLeftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elbowLeftMotorConfig.Feedback.SensorToMechanismRatio = ArmConstants.kElbowReduction;
    elbowLeftMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    elbowLeftMotorConfig.Slot0.kP = 20.0;
    elbowLeftMotorConfig.Slot0.kD = 0.0;
    elbowLeftMotorConfig.Slot0.kS = 0.0;
    elbowLeftMotorConfig.Slot0.kV = 0.0;
    elbowLeftMotorConfig.Slot0.kG = 0.0;
    m_elbowLeftMotor.getConfigurator().apply(elbowLeftMotorConfig);

    final TalonFXConfiguration wristLeftMotorConfig = new TalonFXConfiguration();
    m_wristLeftMotor.getConfigurator().apply(wristLeftMotorConfig);
    wristLeftMotorConfig.CurrentLimits = currentLimitsConfig;
    wristLeftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wristLeftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristLeftMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
    wristLeftMotorConfig.Feedback.SensorToMechanismRatio = ArmConstants.kWristReduction;
    wristLeftMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    wristLeftMotorConfig.Slot0.kP = 20.0;
    wristLeftMotorConfig.Slot0.kD = 0.0;
    wristLeftMotorConfig.Slot0.kS = 0.0;
    wristLeftMotorConfig.Slot0.kV = 0.0;
    wristLeftMotorConfig.Slot0.kG = 0.0;
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
    m_elbowPositionSetpointSignal = m_elbowLeftMotor.getClosedLoopReference();
    m_elbowVelocitySignal = m_elbowLeftMotor.getVelocity();
    m_elbowAppliedVoltsSignal = m_elbowLeftMotor.getMotorVoltage();
    m_elbowCurrentSignal = m_elbowLeftMotor.getStatorCurrent();
    m_elbowFollowerCurrentSignal = m_elbowRightFollowerMotor.getStatorCurrent();
    m_wristPositionSignal = m_wristLeftMotor.getPosition();
    m_wristPositionSetpointSignal = m_wristLeftMotor.getClosedLoopReference();
    m_wristVelocitySignal = m_wristLeftMotor.getVelocity();
    m_wristAppliedVoltsSignal = m_wristLeftMotor.getMotorVoltage();
    m_wristCurrentSignal = m_wristLeftMotor.getStatorCurrent();
    m_wristFollowerCurrentSignal = m_wristRightFollowerMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        m_elbowPositionSignal,
        m_elbowPositionSetpointSignal,
        m_elbowVelocitySignal,
        m_elbowAppliedVoltsSignal,
        m_elbowCurrentSignal,
        m_elbowFollowerCurrentSignal,
        m_wristPositionSignal,
        m_wristPositionSetpointSignal,
        m_wristVelocitySignal,
        m_wristAppliedVoltsSignal,
        m_wristCurrentSignal,
        m_wristFollowerCurrentSignal);

    ParentDevice.optimizeBusUtilizationForAll(
        m_elbowLeftMotor, m_elbowRightFollowerMotor, m_wristLeftMotor, m_wristRightFollowerMotor);

    m_elbowControl = new MotionMagicTorqueCurrentFOC(0.0, 0.0, 0, false, false, false);
    m_wristControl = new MotionMagicTorqueCurrentFOC(0.0, 0.0, 0, false, false, false);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    if (Constants.kIsSim) updateSim();
    inputs.allMotorsConnected =
        BaseStatusSignal.refreshAll(
                m_elbowPositionSignal,
                m_elbowPositionSetpointSignal,
                m_elbowVelocitySignal,
                m_elbowAppliedVoltsSignal,
                m_elbowCurrentSignal,
                m_elbowFollowerCurrentSignal,
                m_wristPositionSignal,
                m_wristPositionSetpointSignal,
                m_wristVelocitySignal,
                m_wristAppliedVoltsSignal,
                m_wristCurrentSignal,
                m_wristFollowerCurrentSignal)
            .equals(StatusCode.OK);

    inputs.elbowPositionRot = m_elbowPositionSignal.getValue();
    inputs.elbowPositionSetpointRot = m_elbowPositionSetpointSignal.getValue();
    inputs.elbowVelocityRps = m_elbowVelocitySignal.getValue();
    inputs.elbowAppliedVolts = m_elbowAppliedVoltsSignal.getValue();
    inputs.elbowCurrentAmps =
        new double[] {m_elbowCurrentSignal.getValue(), m_elbowFollowerCurrentSignal.getValue()};

    inputs.wristPositionRot = m_wristPositionSignal.getValue();
    inputs.wristPositionSetpointRot = m_wristPositionSetpointSignal.getValue();
    inputs.wristVelocityRps = m_wristVelocitySignal.getValue();
    inputs.wristAppliedVolts = m_wristAppliedVoltsSignal.getValue();
    inputs.wristCurrentAmps =
        new double[] {m_wristCurrentSignal.getValue(), m_wristFollowerCurrentSignal.getValue()};
  }

  public void updateSim() {
    final TalonFXSimState elbowSimState = m_elbowLeftMotor.getSimState();
    final TalonFXSimState wristSimState = m_wristLeftMotor.getSimState();

    elbowSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    wristSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_elbowSim.setInput(elbowSimState.getMotorVoltage());
    m_wristSim.setInput(wristSimState.getMotorVoltage());

    m_elbowSim.update(0.02);
    m_wristSim.update(0.02);

    final double elbowVelocityRps = Units.radiansToRotations(m_elbowSim.getVelocityRadPerSec());
    elbowSimState.setRotorVelocity(elbowVelocityRps);
    elbowSimState.addRotorPosition(elbowVelocityRps * 0.02);
    // System.out.println(Units.radiansToRotations(m_elbowSim.getAngleRads()));
    final double wristVelocityRps = Units.radiansToRotations(m_wristSim.getVelocityRadPerSec());
    wristSimState.setRotorVelocity(wristVelocityRps);
    wristSimState.addRotorPosition(wristVelocityRps * 0.02);
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
  public void setBrakeMode(boolean enableBrakeMode) {
    final NeutralModeValue neutralModeValue =
        enableBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    m_elbowLeftMotor.setNeutralMode(neutralModeValue);
    m_wristLeftMotor.setNeutralMode(neutralModeValue);
    m_elbowRightFollowerMotor.setNeutralMode(neutralModeValue);
    m_wristRightFollowerMotor.setNeutralMode(neutralModeValue);
  }

  @Override
  public void configLimits(
      double elbowCruiseVelocityRps,
      double elbowAccelerationRpsSq,
      double wristCruiseVelocityRps,
      double wristAccelerationRpsSq) {
    final TalonFXConfiguration currentElbowConfig = new TalonFXConfiguration();
    m_elbowLeftMotor.getConfigurator().refresh(currentElbowConfig);
    currentElbowConfig.MotionMagic.MotionMagicCruiseVelocity = elbowCruiseVelocityRps;
    currentElbowConfig.MotionMagic.MotionMagicAcceleration = elbowAccelerationRpsSq;
    m_elbowLeftMotor.getConfigurator().apply(currentElbowConfig);
    final TalonFXConfiguration currentWristConfig = new TalonFXConfiguration();
    m_wristLeftMotor.getConfigurator().refresh(currentWristConfig);
    currentWristConfig.MotionMagic.MotionMagicCruiseVelocity = wristCruiseVelocityRps;
    currentWristConfig.MotionMagic.MotionMagicAcceleration = wristAccelerationRpsSq;
    m_wristLeftMotor.getConfigurator().apply(currentWristConfig);
  }
}
