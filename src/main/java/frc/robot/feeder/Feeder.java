package frc.robot.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.NoteVisualizer;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {

  public static final double kFeederReduction = 12.0;
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
  }

  public Command idle() {
    return this.run(() -> m_io.setRollerSpeedVolts(0.0)).withName("Idle (Feeder)");
  }

  public Command stop() {
    return this.runOnce(m_io::stopRoller);
  }

  public Command shoot() {
    return Commands.parallel(
            this.run(() -> m_io.setRollerSpeedVolts(-5.0)),
            Commands.waitUntil(hasNote.negate()).andThen(NoteVisualizer.shoot()))
        .withName("Shoot (Feeder)");
  }

  public Command feed() {
    return this.run(() -> m_io.setRollerSpeedVolts(-6.0)).withName("Feed (Feeder)");
  }

  public Command spit() {
    return this.run(() -> m_io.setRollerSpeedVolts(12.0)).withName("Spit (Feeder)");
  }

  public Command enterCoast() {
    return this.startEnd(() -> m_io.setBrakeMode(false), () -> m_io.setBrakeMode(true));
  }

  public Command pullBack() {
    return this.run(() -> m_io.setRollerSpeedVolts(3.0));
  }
}
