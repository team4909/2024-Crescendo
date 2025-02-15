package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmSetpoints;
import frc.robot.climber.Climber;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.feeder.Feeder;
import frc.robot.intake.Intake;
import frc.robot.lights.Lights;
import frc.robot.shooter.Shooter;

/** Class of factories for commands that combine multiple subsystems. */
public class Superstructure {
  public static Command sensorIntake(Feeder feeder, Intake intake) {
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
            arm.aimWrist(2.264 - Units.degreesToRadians(5.0)),
            feeder.enterCoast(),
            shooter.catchNote())
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

  public static Command aimAtGoal(Drivetrain drivetrain, Shooter shooter, Arm arm, Lights lights) {
    return Commands.parallel(
            Commands.startEnd(
                () ->
                    drivetrain.setHeadingGoal(
                        () -> PoseEstimation.getInstance().getAimingParameters().driveHeading()),
                drivetrain::clearHeadingGoal),
            shooter.runShooter(),
            // arm.aim(
            //     () -> PoseEstimation.getInstance().getAimingParameters().aimingJointIndex(),
            //     () -> PoseEstimation.getInstance().getAimingParameters().armAngle()),
            lights.showReadyToShootStatus(
                drivetrain.atHeadingGoal.and(shooter.readyToShoot).and(arm.atGoal)))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public static Command feedShotHigh(
      Drivetrain drivetrain, Shooter shooter, Arm arm, Lights lights) {
    return Commands.parallel(
            Commands.startEnd(
                () -> drivetrain.setHeadingGoal(PoseEstimation.getInstance()::getFeedHeading),
                drivetrain::clearHeadingGoal),
            shooter.feederShot(),
            arm.goToSetpoint(ArmSetpoints.kFeedHigh),
            lights.showReadyToShootStatus(
                drivetrain.atHeadingGoal.and(arm.atGoal).and(shooter.readyToShoot)))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public static Command feedShotLow(
      Drivetrain drivetrain, Shooter shooter, Arm arm, Lights lights) {
    return Commands.parallel(
            Commands.startEnd(
                () -> drivetrain.setHeadingGoal(PoseEstimation.getInstance()::getFeedHeading),
                drivetrain::clearHeadingGoal),
            shooter.runShooter(),
            arm.goToSetpoint(ArmSetpoints.kFeedLow),
            lights.showReadyToShootStatus(drivetrain.atHeadingGoal.and(arm.atGoal)))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public static Command autoTrap(Arm arm, Climber climber, Shooter shooter, Lights lights) {
    return Commands.sequence(
            new ScheduleCommand(arm.goToSetpoint(ArmSetpoints.kTrap)),
            climber.goToTrapLimit(),
            new ScheduleCommand(lights.startBlink(Color.kGreen)),
            new ScheduleCommand(shooter.trapShot()))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public static Command shortRumble(CommandXboxController controller) {
    return Commands.sequence(
            Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.5)),
            Commands.waitSeconds(0.3),
            Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0)))
        .withName("Short Rumble");
  }
}
