package frc.robot.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;

public class ModuleIOTalonFX implements ModuleIO {

  private final double kDrivekP = 0.031509;
  private final double kDrivekS = 0.1737;
  private final double kDrivekV = 0.13279;
  private final double kSteerkP = 100.0;
  private final double kSteerkD = 0.2;
  private final double kSlipCurrent = 40.0;

  private final TalonFX m_driveMotor, m_steerMotor;
  private final CANcoder m_azimuthEncoder;
  private final double m_absoluteEncoderMagnetOffset;

  private final StatusSignal<Double> m_drivePositionSignal,
      m_driveVelocitySignal,
      m_driveAppliedVoltsSignal,
      m_driveCurrentSignal,
      m_steerAbsolutePositionSignal,
      m_steerPositionSignal,
      m_steerVelocitySignal,
      m_steerAppliedVoltsSignal,
      m_steerCurrentSignal;
  private final Queue<Double> m_drivePositionQueue, m_steerPositionQueue, m_timestampQueue;
  private final MotionMagicExpoVoltage m_steerControl;
  private final VelocityVoltage m_driveControl;
  private final DCMotorSim m_driveSim =
      new DCMotorSim(DCMotor.getKrakenX60Foc(1), Module.kDriveRatio, 0.025);
  private final DCMotorSim m_steerSim =
      new DCMotorSim(DCMotor.getFalcon500Foc(1), Module.kSteerRatio, 0.004);
  private final int m_index;

  public ModuleIOTalonFX(int index) {
    m_index = index;
    switch (index) {
      case 0: // FL
        m_driveMotor = new TalonFX(4, Constants.kDrivetrainCanBus);
        m_steerMotor = new TalonFX(3, Constants.kDrivetrainCanBus);
        m_azimuthEncoder = new CANcoder(12, Constants.kDrivetrainCanBus);
        m_absoluteEncoderMagnetOffset = 0.37353515625;
        break;
      case 1: // FR
        m_driveMotor = new TalonFX(6, Constants.kDrivetrainCanBus);
        m_steerMotor = new TalonFX(5, Constants.kDrivetrainCanBus);
        m_azimuthEncoder = new CANcoder(13, Constants.kDrivetrainCanBus);
        m_absoluteEncoderMagnetOffset = -0.090576171875;
        break;
      case 2: // BL
        m_driveMotor = new TalonFX(2, Constants.kDrivetrainCanBus);
        m_steerMotor = new TalonFX(1, Constants.kDrivetrainCanBus);
        m_azimuthEncoder = new CANcoder(11, Constants.kDrivetrainCanBus);
        m_absoluteEncoderMagnetOffset = 0.275634765625;
        break;
      case 3: // BR
        m_driveMotor = new TalonFX(7, Constants.kDrivetrainCanBus);
        m_steerMotor = new TalonFX(8, Constants.kDrivetrainCanBus);
        m_azimuthEncoder = new CANcoder(14, Constants.kDrivetrainCanBus);
        m_absoluteEncoderMagnetOffset = 0.314208984375;
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    final CANcoderConfiguration azimuthEncoderConfig = new CANcoderConfiguration();
    m_azimuthEncoder.getConfigurator().apply(azimuthEncoderConfig); // Factory Default
    azimuthEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    azimuthEncoderConfig.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    azimuthEncoderConfig.MagnetSensor.MagnetOffset = m_absoluteEncoderMagnetOffset;
    m_azimuthEncoder.getConfigurator().apply(azimuthEncoderConfig);

    final TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
    m_driveMotor.getConfigurator().apply(driveMotorConfig); // Factory Default
    driveMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    driveMotorConfig.Slot0.kP = kDrivekP;
    driveMotorConfig.Slot0.kS = kDrivekS;
    driveMotorConfig.Slot0.kV = kDrivekV;
    driveMotorConfig.CurrentLimits.StatorCurrentLimit = kSlipCurrent;
    driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_driveMotor.getConfigurator().apply(driveMotorConfig);
    m_driveMotor.setPosition(0.0);

    final TalonFXConfiguration steerMotorConfig = new TalonFXConfiguration();
    m_steerMotor.getConfigurator().apply(steerMotorConfig); // Factory Default
    steerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    steerMotorConfig.Feedback.FeedbackRemoteSensorID = m_azimuthEncoder.getDeviceID();
    steerMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    steerMotorConfig.Feedback.RotorToSensorRatio = Module.kSteerRatio;
    steerMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
    steerMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
    steerMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / Module.kSteerRatio;
    steerMotorConfig.MotionMagic.MotionMagicAcceleration =
        steerMotorConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    steerMotorConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * Module.kSteerRatio;
    steerMotorConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    steerMotorConfig.Slot0.kP = kSteerkP;
    steerMotorConfig.Slot0.kD = kSteerkD;
    m_steerMotor.getConfigurator().apply(steerMotorConfig);

    m_drivePositionSignal = m_driveMotor.getPosition();
    m_driveVelocitySignal = m_driveMotor.getVelocity();
    m_driveAppliedVoltsSignal = m_driveMotor.getMotorVoltage();
    m_driveCurrentSignal = m_driveMotor.getSupplyCurrent();
    m_steerPositionSignal = m_steerMotor.getPosition();
    m_steerVelocitySignal = m_steerMotor.getVelocity();
    m_steerAppliedVoltsSignal = m_steerMotor.getMotorVoltage();
    m_steerCurrentSignal = m_steerMotor.getSupplyCurrent();
    m_steerAbsolutePositionSignal = m_azimuthEncoder.getAbsolutePosition();

    m_drivePositionQueue =
        PhoenixOdometryThread.getInstance()
            .registerSignal(m_driveMotor, m_driveMotor.getPosition());
    m_steerPositionQueue =
        PhoenixOdometryThread.getInstance()
            .registerSignal(m_steerMotor, m_steerMotor.getPosition());
    m_timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    BaseStatusSignal.setUpdateFrequencyForAll(
        PhoenixOdometryThread.kOdometryFrequencyHz, m_drivePositionSignal, m_steerPositionSignal);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        m_driveVelocitySignal,
        m_driveAppliedVoltsSignal,
        m_driveCurrentSignal,
        m_steerAbsolutePositionSignal,
        m_steerVelocitySignal,
        m_steerAppliedVoltsSignal,
        m_steerCurrentSignal);
    ParentDevice.optimizeBusUtilizationForAll(m_driveMotor, m_steerMotor, m_azimuthEncoder);

    m_steerControl =
        new MotionMagicExpoVoltage(0.0, true, 0.0, 0, true, false, false).withUpdateFreqHz(0.0);
    m_driveControl =
        new VelocityVoltage(0.0, 0.0, true, 0.0, 0, true, false, false).withUpdateFreqHz(0.0);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    if (Constants.kIsSim) {
      updateSim();
    }
    BaseStatusSignal.refreshAll(
        m_drivePositionSignal,
        m_driveVelocitySignal,
        m_driveAppliedVoltsSignal,
        m_driveCurrentSignal,
        m_steerAbsolutePositionSignal,
        m_steerPositionSignal,
        m_steerVelocitySignal,
        m_steerAppliedVoltsSignal,
        m_steerCurrentSignal);

    inputs.drivePositionRad =
        Units.rotationsToRadians(m_drivePositionSignal.getValueAsDouble()) / Module.kDriveRatio;
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(m_driveVelocitySignal.getValueAsDouble()) / Module.kDriveRatio;
    inputs.driveAppliedVolts = m_driveAppliedVoltsSignal.getValueAsDouble();
    inputs.driveCurrentAmps = m_driveCurrentSignal.getValueAsDouble();

    inputs.steerAbsolutePosition =
        Rotation2d.fromRotations(m_steerAbsolutePositionSignal.getValueAsDouble());

    /**
     * NOTE We do not need to divide by the steer ratio since we're fusing a cancoder with the ratio
     * already accounted for.
     */
    inputs.steerPosition = Rotation2d.fromRotations(m_steerPositionSignal.getValueAsDouble());
    inputs.steerVelocityRadPerSec =
        Units.rotationsToRadians(m_steerVelocitySignal.getValueAsDouble());
    inputs.steerAppliedVolts = m_steerAppliedVoltsSignal.getValueAsDouble();
    inputs.steerCurrentAmps = m_steerCurrentSignal.getValueAsDouble();

    inputs.odometryDrivePositionsRad =
        m_drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
    inputs.odometryTurnPositions =
        m_steerPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    inputs.odometryTimestamps =
        m_timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    m_drivePositionQueue.clear();
    m_steerPositionQueue.clear();
    m_timestampQueue.clear();
  }

  public void updateSim() {
    final TalonFXSimState driveSimState = m_driveMotor.getSimState();
    final TalonFXSimState steerSimState = m_steerMotor.getSimState();
    final CANcoderSimState azimuthSimState = m_azimuthEncoder.getSimState();

    steerSimState.Orientation = ChassisReference.Clockwise_Positive;
    driveSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    steerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    azimuthSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_driveSim.setInput(driveSimState.getMotorVoltage());
    m_steerSim.setInput(steerSimState.getMotorVoltage());

    m_driveSim.update(0.02);
    m_steerSim.update(0.02);

    Logger.recordOutput(
        "Drivetrain/Sim/Module " + m_index + "/Drive Motor Output V",
        driveSimState.getMotorVoltage());
    final double driveRotorVelocityRPS =
        Units.radiansToRotations(m_driveSim.getAngularVelocityRadPerSec()) * Module.kDriveRatio;
    driveSimState.addRotorPosition(driveRotorVelocityRPS * 0.02);
    driveSimState.setRotorVelocity(driveRotorVelocityRPS);
    Logger.recordOutput(
        "Drivetrain/Sim/Module " + m_index + "/Steer Motor Output V",
        steerSimState.getMotorVoltage());
    /*
     * Note that since we are simulating a fused cancoder we cannot simulate the
     * internal rotor's state, just the cancoder
     */
    final double steerVelocityRPS =
        Units.radiansToRotations(m_steerSim.getAngularVelocityRadPerSec());
    azimuthSimState.addPosition(steerVelocityRPS * 0.02);
    azimuthSimState.setVelocity(steerVelocityRPS);
  }

  @Override
  public void setSteerRotations(double angleRotations) {
    m_steerMotor.setControl(m_steerControl.withPosition(angleRotations));
  }

  @Override
  public void setDriveRPS(double speedRPS) {
    m_driveMotor.setControl(m_driveControl.withVelocity(speedRPS));
  }

  @Override
  public void setSteerVoltage(double volts) {
    m_steerMotor.setVoltage(volts);
  }

  @Override
  public void setDriveVoltage(double volts) {
    m_driveMotor.setVoltage(volts);
  }
}
