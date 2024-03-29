package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.arm.Arm;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.feeder.Feeder;
import frc.robot.intake.Intake;
import frc.robot.lights.Lights;
import frc.robot.shooter.Shooter;

/** Class of factories for commands that combine multiple subsystems. */
public class Superstructure {
  public static Command sensorIntake(Feeder feeder, Intake intake) {
    // In sim just wait 5 seconds since we don't have a sensor
    final Command feedUntilHasNote =
        Commands.either(
            feeder.feed().until(intake.hasIntookPieceSim),
            feeder.feed().until(feeder.hasNote),
            () -> Constants.kIsSim);
    return Commands.deadline(feedUntilHasNote, intake.intake())
        .andThen(feeder.idle().withTimeout(0.01))
        .withName("Sensor Intake");
  }

  public static Command sensorCatch(Shooter shooter, Feeder feeder, Intake intake, Arm arm) {
    return Commands.parallel(
            arm.goToSetpoint(-0.558, 2.264 - Units.degreesToRadians(5.0), 0, 0),
            feeder.enterCoast(),
            shooter.catchNote())
        .andThen(feeder.pullBack().until(feeder.hasNote))
        .withName("Sensor Catch");
  }

  public static Command ampShot(Arm arm, Shooter shooter) {
    return Commands.parallel(
            arm.goToSetpoint(1.49 + 0.0873, -2.307, 0.0, 0.0), shooter.runShooter())
        .finallyDo(() -> arm.goToSetpoint(-0.548, 2.485, 0.15, 0.0));
  }

  public static Command spit(Shooter shooter, Feeder feeder, Intake intake) {
    return Commands.parallel(shooter.spit(), feeder.spit(), intake.spit()).withName("Spit");
  }

  public static Command aimAtGoal(Drivetrain drivetrain, Shooter shooter, Lights lights) {
    return Commands.parallel(
            Commands.startEnd(
                () ->
                    drivetrain.setHeadingGoal(
                        () -> PoseEstimation.getInstance().getAimingParameters().driveHeading()),
                drivetrain::clearHeadingGoal),
            shooter.runShooter(),
            lights.showReadyToShootStatus(drivetrain.atHeadingGoal.and(shooter.readyToShoot)))
        .finallyDo(() -> lights.getCurrentCommand().cancel())
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }
}
