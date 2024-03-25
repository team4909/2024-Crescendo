package frc.robot;

import com.choreo.lib.Choreo;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.arm.Arm;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.feeder.Feeder;
import frc.robot.intake.Intake;
import frc.robot.lights.Lights;
import frc.robot.shooter.Shooter;

public class Autos {

  private final double kAimAndShootTimeoutSeconds = 3.0;

  private final Drivetrain m_drivetrain;
  private final Shooter m_shooter;
  private final Feeder m_feeder;
  private final Intake m_intake;
  private final Arm m_arm;
  private final Lights m_lights;

  public Autos(
      Drivetrain drivetrain,
      Shooter shooter,
      Feeder feeder,
      Intake intake,
      Arm arm,
      Lights lights) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_feeder = feeder;
    m_intake = intake;
    m_arm = arm;
    m_lights = lights;
  }

  public Command centerlineTwoPiece() {
    return Commands.sequence(
            resetPose("Centerline Auto"),
            shoot(false),
            intake().deadlineWith(getPathFollowingCommand("Centerline Auto.1")),
            getPathFollowingCommand("Centerline Auto.2"),
            shoot(false),
            intake().deadlineWith(getPathFollowingCommand("Centerline Auto.3")),
            getPathFollowingCommand("Centerline Auto.4"),
            shoot(false))
        .withName("Centerline 3 Piece");
  }

  public Command sixPiece() {
    return Commands.sequence(
            resetPose("6Piece"),
            aimAndShoot(),
            intake().deadlineWith(getPathFollowingCommand("6Piece.1")),
            aimAndShoot(),
            intake().deadlineWith(getPathFollowingCommand("6Piece.2")),
            aimAndShoot(),
            intake().deadlineWith(getPathFollowingCommand("6Piece.3")),
            aimAndShoot(),
            intake().deadlineWith(getPathFollowingCommand("6Piece.4")),
            getPathFollowingCommand("6Piece.5"),
            aimAndShoot(),
            intake().deadlineWith(getPathFollowingCommand("6Piece.6")),
            getPathFollowingCommand("6Piece.7"),
            aimAndShoot())
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("Six Piece");
  }

  private Command getPathFollowingCommand(String trajectoryName) {
    return AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(trajectoryName));
  }

  private Command resetPose(String trajectoryName) {
    Pose2d startingPose = Choreo.getTrajectory(trajectoryName).getInitialPose();
    return Commands.runOnce(
        () ->
            m_drivetrain.resetPose.accept(
                m_drivetrain.onRedAllianceSupplier.getAsBoolean()
                    ? GeometryUtil.flipFieldPose(startingPose)
                    : startingPose));
  }

  private Command intake() {
    return Superstructure.sensorIntake(m_feeder, m_intake)
        .andThen(m_intake.stop(), m_feeder.stop());
  }

  private Command aim() {
    return Superstructure.aimAtGoal(m_drivetrain, m_shooter, m_arm, m_lights)
        .alongWith(m_drivetrain.blankDrive());
  }

  private Command aimAndShoot() {
    final var state =
        new Object() {
          private final Timer timeoutTimer = new Timer();
          private final Trigger hasTimedOut =
              new Trigger(() -> timeoutTimer.hasElapsed(kAimAndShootTimeoutSeconds));
          private boolean hasShot = false;
        };
    return Commands.race(
            aim().until(() -> state.hasShot),
            Commands.sequence(feedShooter(), Commands.runOnce(() -> state.hasShot = true))
                .onlyIf(
                    m_drivetrain
                        .atHeadingGoal
                        .and(m_shooter.readyToShoot)
                        .and(m_arm.atGoal)
                        .or(state.hasTimedOut))
                .repeatedly())
        .beforeStarting(() -> state.timeoutTimer.start())
        .andThen(Commands.print("WARNING: Aim and shoot timed out").onlyIf(state.hasTimedOut))
        .finallyDo(
            () -> {
              state.timeoutTimer.stop();
              state.timeoutTimer.reset();
              state.hasShot = false;
            });
  }

  private Command feedShooter() {
    return m_feeder
        .shoot()
        .alongWith(m_intake.intake())
        .withTimeout(0.5)
        .andThen(m_intake.stop(), m_feeder.stop());
  }

  private Command shoot(boolean disableShooterAfter) {
    return Commands.race(
            m_shooter.runShooter(),
            Commands.waitUntil(m_shooter.readyToShoot).andThen(feedShooter()))
        .andThen(m_shooter.stop().unless(() -> !disableShooterAfter));
  }
}
