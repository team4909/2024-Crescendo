package frc.robot.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final double kFarShotVelocityRpm = 5000.0;
  private final double kSpitVelocityRpm = -500.0;
  private final double kIdleVelocityRpm = 60.0;

  private final ShooterIO m_io;
  private final ShooterIOInputsAutoLogged m_inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    m_io = io;
    setDefaultCommand(idle());
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("ShooterInputs", m_inputs);
  }

  private void setRollersSetpointRpm(double velocityRpm) {
    m_io.setRollersRPS(velocityRpm * 60.0);
  }

  private boolean getShooterRollersAtSetpoint() {
    return MathUtil.isNear(kFarShotVelocityRpm, m_inputs.topRollerVelocityRps / 60.0, 5.0)
        && MathUtil.isNear(kFarShotVelocityRpm, m_inputs.bottomRollerVelocityRps / 60.0, 5.0);
  }

  public Trigger readyToShoot() {
    return new Trigger(() -> getShooterRollersAtSetpoint()).debounce(0.5);
  }

  public Command idle() {
    return this.run(
        () -> {
          setRollersSetpointRpm(kIdleVelocityRpm);
        });
  }

  public Command runShooter() {
    return this.run(
        () -> {
          setRollersSetpointRpm(kFarShotVelocityRpm);
        });
  }

  public Command spit() {
    return this.run(
        () -> {
          m_io.setRollersRPS(-kSpitVelocityRpm);
        });
  }

  public Command disableShooter() {
    return this.run(
        () -> {
          m_io.setRollersRPS(0.0);
        });
  }

  public Command catchNote() {
    return this.run(
        () -> {
          setRollersSetpointRpm(-kFarShotVelocityRpm);
        });
  }

  public Command shootWithFeederDelay() {
    return this.run(() -> setRollersSetpointRpm(kFarShotVelocityRpm))
        .until(() -> getShooterRollersAtSetpoint())
        .andThen(runShooter());
  }
}
