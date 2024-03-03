package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmSetpoints;
import frc.robot.feeder.Feeder;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;

/** Class of factories for commands that combine multiple subsystems. */
public class Superstructure {
  public static Command sensorIntake(Feeder feeder, Intake intake) {
    // In sim just wait 5 seconds since we don't have a sensor
    final Command feedUntilHasNote =
        Commands.either(
            feeder.feed().withTimeout(5.0),
            feeder.feed().until(feeder.hasNote),
            () -> Constants.kIsSim);
    return Commands.deadline(feedUntilHasNote, intake.intake())
        .andThen(feeder.idle().withTimeout(0.01))
        .withName("Sensor Intake");
  }

  public static Command sensorCatch(Shooter shooter, Feeder feeder, Intake intake, Arm arm) {
    return Commands.parallel(
            arm.aimWrist(Arm.kCatchWristAngleRad), feeder.enterCoast(), shooter.catchNote())
        .andThen(feeder.pullBack().until(feeder.hasNote))
        .withName("Sensor Catch");
  }

  public static Command ampShot(Arm arm, Shooter shooter) {
    return Commands.parallel(arm.goToSetpoint(ArmSetpoints.kAmp), shooter.runShooter())
        .finallyDo(() -> arm.goToSetpoint(ArmSetpoints.kStowed));
  }

  public static Command spit(Shooter shooter, Feeder feeder, Intake intake) {
    return Commands.parallel(shooter.spit(), feeder.spit(), intake.spit()).withName("Spit");
  }
}
