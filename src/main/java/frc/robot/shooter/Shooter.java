package frc.robot.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private final double kFarShotVelocityRpm = 5000.0;
  private final double kSpitVelocityRpm = -500.0;
  private final double kIdleVelocityRpm = 60.0;

  private final ShooterIO m_io;
  private final ShooterIOInputsAutoLogged m_inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    m_io = io;
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
  }

  private void setRollersSetpointRpm(double velocityRpm) {
    m_io.setRollerRPS(velocityRpm * 60.0);
  }

  private boolean getShooterRollersAtSetpoint() {
    return MathUtil.isNear(kFarShotVelocityRpm, m_inputs.topRollerVelocityRps / 60.0, 5.0)
        && MathUtil.isNear(kFarShotVelocityRpm, m_inputs.bottomRollerVelocityRps / 60.0, 5.0);
  }

  public Command idle() {
    return this.run(
        () -> {
          setRollersSetpointRpm(kIdleVelocityRpm);
          m_io.setFeederDutyCycle(0.0);
        });
  }

  public Command runShooter() {
    return this.run(
        () -> {
          setRollersSetpointRpm(kFarShotVelocityRpm);
          m_io.setFeederDutyCycle(-1.0);
        });
  }

  public Command runFeeder() {
    return this.run(() -> m_io.setFeederDutyCycle(-1.0)).until(() -> m_inputs.noteSensorTripped);
  }

  public Command spit() {
    return this.run(
        () -> {
          m_io.setRollerRPS(-kSpitVelocityRpm);
          m_io.setFeederDutyCycle(-1.0);
        });
  }

  public Command disableShooter() {
    return this.run(
        () -> {
          m_io.setRollerRPS(0.0);
          m_io.setFeederDutyCycle(0.0);
        });
  }

  public Command catchNote() {
    return new InstantCommand(
        () -> {
          m_io.setFeederDutyCycle(-0.25);
          setRollersSetpointRpm(-kFarShotVelocityRpm);
        },
        this);
  }

  public Command pullBack() {
    return this.run(() -> m_io.setFeederDutyCycle(0.3)).withTimeout(0.15);
  }

  public Command shootWithFeederDelay() {
    return this.run(() -> setRollersSetpointRpm(kFarShotVelocityRpm))
        .until(() -> getShooterRollersAtSetpoint())
        .andThen(runShooter());
  }
}
