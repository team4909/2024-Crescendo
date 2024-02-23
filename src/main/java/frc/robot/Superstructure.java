package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.feeder.Feeder;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;

/** Class of factories for commands that combine multiple subsystems. */
public class Superstructure {
  public static Command sensorIntake(Feeder feeder, Intake intake) {
    return Commands.deadline(feeder.feed().until(feeder.hasNote), intake.intake());
  }

  public static Command sensorCatch(Shooter shooter, Feeder feeder, Intake intake) {
    return Commands.deadline(shooter.catchNote().until(feeder.hasNote), intake.intake());
  }
}
