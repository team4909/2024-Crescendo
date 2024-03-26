package frc.robot.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX m_topRoller, m_bottomRoller;
  private final MotionMagicVelocityVoltage m_topRollerControl, m_bottomRollerControl;
  private final StatusSignal<Double> m_topRollerPositionSignal,
      m_topRollerVelocitySignal,
      m_topRollerAppliedVoltageSignal,
      m_topRollerCurrentSignal,
      m_bottomRollerPositionSignal,
      m_bottomRollerVelocitySignal,
      m_bottomRollerAppliedVoltageSignal,
      m_bottomRollerCurrentSignal;

  public ShooterIOTalonFX() {
    m_topRoller = new TalonFX(20, Constants.kSuperstructureCanBus);
    m_bottomRoller = new TalonFX(21, Constants.kSuperstructureCanBus);

    final MotionMagicConfigs motionMagicConfigs =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(Shooter.kFarShotVelocityRpm * (1 / 0.25));
    final TalonFXConfiguration topRollerMotorConfig = new TalonFXConfiguration();
    topRollerMotorConfig.Slot0.kS = Shooter.topRollerkS;
    topRollerMotorConfig.Slot0.kV = Shooter.topRollerkV;
    topRollerMotorConfig.Slot0.kA = Shooter.topRollerkA;
    topRollerMotorConfig.Slot0.kP = Shooter.topRollerkP;
    topRollerMotorConfig.MotionMagic = motionMagicConfigs;
    m_topRoller.getConfigurator().apply(topRollerMotorConfig);
    final TalonFXConfiguration bottomRollerMotorConfig = new TalonFXConfiguration();
    bottomRollerMotorConfig.Slot0.kS = Shooter.bottomRollerkS;
    bottomRollerMotorConfig.Slot0.kV = Shooter.bottomRollerkV;
    bottomRollerMotorConfig.Slot0.kA = Shooter.bottomRollerkA;
    bottomRollerMotorConfig.Slot0.kP = Shooter.bottomRollerkP;
    bottomRollerMotorConfig.MotionMagic = motionMagicConfigs;
    m_bottomRoller.getConfigurator().apply(bottomRollerMotorConfig);

    m_topRollerPositionSignal = m_topRoller.getPosition();
    m_topRollerVelocitySignal = m_topRoller.getVelocity();
    m_topRollerAppliedVoltageSignal = m_topRoller.getMotorVoltage();
    m_topRollerCurrentSignal = m_topRoller.getStatorCurrent();
    m_bottomRollerPositionSignal = m_bottomRoller.getPosition();
    m_bottomRollerVelocitySignal = m_bottomRoller.getVelocity();
    m_bottomRollerAppliedVoltageSignal = m_bottomRoller.getMotorVoltage();
    m_bottomRollerCurrentSignal = m_bottomRoller.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        m_topRollerVelocitySignal,
        m_topRollerAppliedVoltageSignal,
        m_topRollerCurrentSignal,
        m_bottomRollerVelocitySignal,
        m_bottomRollerAppliedVoltageSignal,
        m_bottomRollerCurrentSignal);

    ParentDevice.optimizeBusUtilizationForAll(m_topRoller, m_bottomRoller);

    m_topRollerControl = new MotionMagicVelocityVoltage(0, 0, true, 0, 0, false, false, false);
    m_bottomRollerControl = new MotionMagicVelocityVoltage(0, 0, true, 0, 0, false, false, false);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.rollerMotorsConnected =
        BaseStatusSignal.refreshAll(
                m_topRollerPositionSignal,
                m_topRollerVelocitySignal,
                m_topRollerAppliedVoltageSignal,
                m_topRollerCurrentSignal,
                m_bottomRollerPositionSignal,
                m_bottomRollerVelocitySignal,
                m_bottomRollerAppliedVoltageSignal,
                m_bottomRollerCurrentSignal)
            .equals(StatusCode.OK);

    inputs.topRollerPositionRot = m_topRollerPositionSignal.getValue();
    inputs.topRollerVelocityRps = m_topRollerVelocitySignal.getValue();
    inputs.topRollerAppliedVolts = m_topRollerAppliedVoltageSignal.getValue();
    inputs.topRollerCurrentAmps = m_topRollerCurrentSignal.getValue();
    inputs.bottomRollerPositionRot = m_bottomRollerPositionSignal.getValue();
    inputs.bottomRollerVelocityRps = m_bottomRollerVelocitySignal.getValue();
    inputs.bottomRollerAppliedVolts = m_bottomRollerAppliedVoltageSignal.getValue();
    inputs.bottomRollerCurrentAmps = m_bottomRollerCurrentSignal.getValue();
  }

  @Override
  public void setTopRollerVoltage(double volts) {
    m_topRoller.setVoltage(volts);
  }

  @Override
  public void setBottomRollerVoltage(double volts) {
    m_bottomRoller.setVoltage(volts);
  }

  @Override
  public void setTopRollerVelocity(double velocityRps) {
    m_topRoller.setControl(m_topRollerControl.withVelocity(velocityRps));
  }

  @Override
  public void setBottomRollerVelocity(double velocityRps) {
    m_bottomRoller.setControl(m_bottomRollerControl.withVelocity(velocityRps));
  }

  public void stopRollers() {
    m_topRoller.setVoltage(0.0);
    m_bottomRoller.setVoltage(0.0);
  }
}
