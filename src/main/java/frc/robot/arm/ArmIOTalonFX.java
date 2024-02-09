package frc.robot.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class ArmIOTalonFX implements ArmIO {

  private final double kCurrentLimitAmps = 80.0;
  private final TalonFX m_elbowLeftMotor,
      m_elbowRightFollowerMotor,
      m_wristLeftMotor,
      m_wristRightFollowerMotor;
  private final MotionMagicExpoVoltage m_elbowControl, m_wristControl;
  private final ArmModel m_armSimModel =
      new ArmModel(ArmConfig.kElbowConfig, ArmConfig.kWristConfig);
  private Vector<N4> elbowWristSimStates = VecBuilder.fill(Math.PI / 2, Math.PI / 2, 0.0, 0.0);

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
    elbowLeftMotorConfig.Slot0.kP = 5;
    elbowLeftMotorConfig.Slot0.kD = 0.05;
    elbowLeftMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / ArmConfig.kElbowReduction;
    elbowLeftMotorConfig.MotionMagic.MotionMagicAcceleration =
        elbowLeftMotorConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    elbowLeftMotorConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * ArmConfig.kElbowReduction;
    elbowLeftMotorConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    m_elbowLeftMotor.getConfigurator().apply(elbowLeftMotorConfig);

    final TalonFXConfiguration wristLeftMotorConfig = new TalonFXConfiguration();
    m_wristLeftMotor.getConfigurator().apply(wristLeftMotorConfig);
    wristLeftMotorConfig.CurrentLimits = currentLimitsConfig;
    wristLeftMotorConfig.TorqueCurrent = torqueCurrentConfig;
    wristLeftMotorConfig.Slot0.kP = 5;
    wristLeftMotorConfig.Slot0.kD = 0.05;
    wristLeftMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / ArmConfig.kWristReduction;
    wristLeftMotorConfig.MotionMagic.MotionMagicAcceleration =
        wristLeftMotorConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    wristLeftMotorConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * ArmConfig.kWristReduction;
    wristLeftMotorConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
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

    m_elbowControl = new MotionMagicExpoVoltage(0.0, false, 0, 0, true, false, false);
    m_wristControl = new MotionMagicExpoVoltage(0.0, false, 0, 0, true, false, false);
  }

  public void updateInputs(ArmIOInputs inputs) {

    if (Constants.kIsSim) {
      updateSim();
    }
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

    inputs.elbowPositionRad =
        Units.rotationsToRadians(
            BaseStatusSignal.getLatencyCompensatedValue(
                    m_elbowPositionSignal, m_elbowVelocitySignal)
                / ArmConfig.kElbowReduction);
    inputs.elbowVelocityRadPerSec =
        Units.rotationsToRadians(m_elbowVelocitySignal.getValue() / ArmConfig.kElbowReduction);
    inputs.elbowAppliedVolts = m_elbowAppliedVoltsSignal.getValue();
    inputs.elbowCurrentAmps =
        new double[] {m_elbowCurrentSignal.getValue(), m_elbowFollowerCurrentSignal.getValue()};

    inputs.wristPositionRad =
        Units.rotationsToRadians(
            BaseStatusSignal.getLatencyCompensatedValue(
                    m_wristPositionSignal, m_wristVelocitySignal)
                / ArmConfig.kWristReduction);
    inputs.wristVelocityRadPerSec =
        Units.rotationsToRadians(m_wristVelocitySignal.getValue() / ArmConfig.kWristReduction);
    inputs.wristAppliedVolts = m_wristAppliedVoltsSignal.getValue();
    inputs.wristCurrentAmps =
        new double[] {m_wristCurrentSignal.getValue(), m_wristFollowerCurrentSignal.getValue()};
  }

  public void updateSim() {
    final TalonFXSimState elbowSimState = m_elbowLeftMotor.getSimState();
    final TalonFXSimState wristSimState = m_wristLeftMotor.getSimState();
    elbowSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    wristSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    elbowWristSimStates =
        m_armSimModel.simulate(
            elbowWristSimStates,
            VecBuilder.fill(elbowSimState.getMotorVoltage(), wristSimState.getMotorVoltage()),
            0.02);
    elbowSimState.setRawRotorPosition(
        Units.radiansToRotations(elbowWristSimStates.get(0, 0)) * ArmConfig.kElbowReduction);
    elbowSimState.setRotorVelocity(
        Units.radiansToRotations(elbowWristSimStates.get(2, 0)) * ArmConfig.kElbowReduction);
    wristSimState.setRawRotorPosition(
        Units.radiansToRotations(elbowWristSimStates.get(1, 0)) * ArmConfig.kWristReduction);
    wristSimState.setRotorVelocity(
        Units.radiansToRotations(elbowWristSimStates.get(3, 0)) * ArmConfig.kWristReduction);
  }

  public void setElbowRotatations(double rotations, double feedForwardVolts) {
    m_elbowLeftMotor.setControl(
        m_elbowControl.withPosition(rotations).withFeedForward(feedForwardVolts));
  }

  public void setWristRotatations(double rotations, double feedForwardVolts) {
    m_wristLeftMotor.setControl(
        m_wristControl.withPosition(rotations).withFeedForward(feedForwardVolts));
  }

  public void setElbowVoltage(double volts) {
    m_elbowLeftMotor.setVoltage(volts);
  }

  public void setWristVoltage(double volts) {
    m_wristLeftMotor.setVoltage(volts);
  }
}
