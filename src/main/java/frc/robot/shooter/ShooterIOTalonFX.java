package frc.robot.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;

public class ShooterIOTalonFX implements ShooterIO {

  private final double kFeederReduction = 5.0;
  private final TalonFX m_topRoller, m_bottomRoller, m_feeder;
  private final DigitalInput m_noteSensor;

  private final VelocityVoltage m_topRollerControl, m_bottomRollerControl;
  private final DutyCycleOut m_feederControl;

  private final StatusSignal<Double> m_topRollerVelocitySignal,
      m_topRollerAccelerationSignal,
      m_topRollerAppliedVoltageSignal,
      m_topRollerCurrentSignal,
      m_bottomRollerVelocitySignal,
      m_bottomRollerAccelerationSignal,
      m_bottomRollerAppliedVoltageSignal,
      m_bottomRollerCurrentSignal,
      m_feederVelocitySignal,
      m_feederAccelerationSignal,
      m_feederAppliedVoltageSignal,
      m_feederCurrentSignal;

  public ShooterIOTalonFX() {
    m_topRoller = new TalonFX(17, "CANivore2");
    m_bottomRoller = new TalonFX(18, "CANivore2");
    m_feeder = new TalonFX(19, "CANivore2");
    m_noteSensor = new DigitalInput(3);

    final Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = 1.0;
    m_topRoller.getConfigurator().apply(slot0Configs);
    m_bottomRoller.getConfigurator().apply(slot0Configs);

    m_topRollerVelocitySignal = m_topRoller.getVelocity();
    m_topRollerAccelerationSignal = m_topRoller.getAcceleration();
    m_topRollerAppliedVoltageSignal = m_topRoller.getMotorVoltage();
    m_topRollerCurrentSignal = m_topRoller.getStatorCurrent();
    m_bottomRollerVelocitySignal = m_bottomRoller.getVelocity();
    m_bottomRollerAccelerationSignal = m_bottomRoller.getAcceleration();
    m_bottomRollerAppliedVoltageSignal = m_bottomRoller.getMotorVoltage();
    m_bottomRollerCurrentSignal = m_bottomRoller.getStatorCurrent();
    m_feederVelocitySignal = m_feeder.getVelocity();
    m_feederAccelerationSignal = m_feeder.getAcceleration();
    m_feederAppliedVoltageSignal = m_feeder.getMotorVoltage();
    m_feederCurrentSignal = m_feeder.getStatorCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        m_topRollerVelocitySignal,
        m_topRollerAccelerationSignal,
        m_topRollerAppliedVoltageSignal,
        m_topRollerCurrentSignal,
        m_bottomRollerVelocitySignal,
        m_bottomRollerAccelerationSignal,
        m_bottomRollerAppliedVoltageSignal,
        m_bottomRollerCurrentSignal,
        m_feederVelocitySignal,
        m_feederAccelerationSignal,
        m_feederAppliedVoltageSignal,
        m_feederCurrentSignal);
    ParentDevice.optimizeBusUtilizationForAll(m_topRoller, m_bottomRoller, m_feeder);

    m_feederControl = new DutyCycleOut(0, false, true, false, false);
    m_topRollerControl = new VelocityVoltage(0, 0, false, 0, 0, true, false, false);
    m_bottomRollerControl = new VelocityVoltage(0, 0, false, 0, 0, true, false, false);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    BaseStatusSignal.refreshAll(
        m_topRollerVelocitySignal,
        m_topRollerAccelerationSignal,
        m_topRollerAppliedVoltageSignal,
        m_topRollerCurrentSignal,
        m_bottomRollerVelocitySignal,
        m_bottomRollerAccelerationSignal,
        m_bottomRollerAppliedVoltageSignal,
        m_bottomRollerCurrentSignal,
        m_feederVelocitySignal,
        m_feederAccelerationSignal,
        m_feederAppliedVoltageSignal,
        m_feederCurrentSignal);

    inputs.topRollerVelocityRps = m_topRollerVelocitySignal.getValue();
    inputs.topRollerAccelerationRpsSq = m_topRollerAccelerationSignal.getValue();
    inputs.topRollerAppliedVolts = m_topRollerAppliedVoltageSignal.getValue();
    inputs.topRollerCurrentAmps = m_topRollerCurrentSignal.getValue();
    inputs.bottomRollerVelocityRps = m_bottomRollerVelocitySignal.getValue();
    inputs.bottomRollerAccelerationRpsSq = m_bottomRollerAccelerationSignal.getValue();
    inputs.bottomRollerAppliedVolts = m_bottomRollerAppliedVoltageSignal.getValue();
    inputs.bottomRollerCurrentAmps = m_bottomRollerCurrentSignal.getValue();
    inputs.feederVelocityRps = m_feederVelocitySignal.getValue() / kFeederReduction;
    inputs.feederAccelerationRpsSq = m_feederAccelerationSignal.getValue() / kFeederReduction;
    inputs.feederAppliedVolts = m_feederAppliedVoltageSignal.getValue();
    inputs.feederCurrentAmps = m_feederCurrentSignal.getValue();
    inputs.noteSensorTripped = !m_noteSensor.get();
  }

  @Override
  public void setRollerRPS(double velocityRPS) {
    m_topRoller.setControl(m_topRollerControl.withVelocity(velocityRPS));
    m_bottomRoller.setControl(m_bottomRollerControl.withVelocity(velocityRPS));
  }

  @Override
  public void setFeederDutyCycle(double volts) {
    m_feeder.setControl(m_feederControl.withOutput(volts));
  }
}
