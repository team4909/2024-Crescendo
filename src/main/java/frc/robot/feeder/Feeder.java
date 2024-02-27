package frc.robot.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {

  public final Trigger hasNote;

  private final FeederIO m_io;
  private final FeederIOInputsAutoLogged m_inputs = new FeederIOInputsAutoLogged();

  public Feeder(FeederIO io) {
    m_io = io;
    hasNote = new Trigger(() -> m_inputs.topNoteSensorTripped);
    setDefaultCommand(idle());
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("FeederInputs", m_inputs);
    Logger.recordOutput(
        "Feeder/Command", getCurrentCommand() == null ? "" : getCurrentCommand().getName());
  }

  public Command idle() {
    return this.run(() -> m_io.setFeederDutyCycle(0.0));
  }

  public Command feed() {
    return this.run(() -> m_io.setFeederDutyCycle(-0.5));
  }

  public Command spit() {
    return this.run(() -> m_io.setFeederDutyCycle(1.0));
  }

  public Command holdCatch() {
    return this.run(() -> m_io.setFeederDutyCycle(-0.25));
  }

  public Command pullBack() {
    return this.run(() -> m_io.setFeederDutyCycle(0.1));
  }
}
