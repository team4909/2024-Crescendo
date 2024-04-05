package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import java.util.Set;
import java.util.function.BooleanSupplier;

public class Autos {

  private final double kAimAndShootTimeoutSeconds = 3.0;

  private final Drivetrain m_drivetrain;
  private final Shooter m_shooter;
  private final Feeder m_feeder;
  private final Intake m_intake;
  private final Arm m_arm;
  private final Lights m_lights;

  private final PIDController m_translationController = new PIDController(3.0, 0.0, 0.0);
  private final PIDController m_rotationController = new PIDController(3.0, 0.0, 0.0);
  private boolean shootingWhileMoving = false;

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

  public Command subShot() {
    return Commands.parallel(
            m_arm.aimWrist(Arm.kSubwooferWristAngleRad),
            shoot(true).beforeStarting(Commands.waitSeconds(2.0)))
        .withName("Sub Shot");
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

  public Command threePieceSourceSide() {
    return Commands.sequence(
            resetPose("3PieceSourceSide"),
            aimAndShoot(),
            intake().raceWith(getPathFollowingCommand("3PieceSourceSide.1")),
            getPathFollowingCommand("3PieceSourceSide.2"),
            aimAndShoot(),
            intake().raceWith(getPathFollowingCommand("3PieceSourceSide.3")),
            getPathFollowingCommand("3PieceSourceSide.4"),
            aimAndShoot())
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("Three Piece Source Side");
  }

  public Command centerlineDisrupt() {
    return Commands.sequence(
            resetPose("CenterlineDisrupt"),
            toggleShootingWhileMoving(),
            Commands.deadline(
                getPathFollowingCommand(
                    "CenterlineDisrupt.1",
                    shootWhileMovingControlFunction(() -> shootingWhileMoving)),
                aimAndShoot().andThen(toggleShootingWhileMoving())))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("Centerline Disrupt");
  }

  private Command toggleShootingWhileMoving() {
    return Commands.runOnce(() -> shootingWhileMoving = !shootingWhileMoving);
  }

  private Command getPathFollowingCommand(
      String trajectoryName, ChoreoControlFunction controlFunction) {
    return Choreo.choreoSwerveCommand(
            Choreo.getTrajectory(trajectoryName),
            PoseEstimation.getInstance()::getPose,
            controlFunction,
            m_drivetrain::runVelocity,
            Constants.onRedAllianceSupplier,
            m_drivetrain)
        .andThen(Commands.waitSeconds(3.0));
  }

  private Command getPathFollowingCommand(String trajectoryName) {
    return getPathFollowingCommand(
        trajectoryName,
        Choreo.choreoSwerveController(
            m_translationController, m_translationController, m_rotationController));
  }

  private ChoreoControlFunction shootWhileMovingControlFunction(BooleanSupplier shootingOnTheMove) {
    return (pose, referenceState) -> {
      double xFF = referenceState.velocityX;
      double yFF = referenceState.velocityY;
      double rotationFF = referenceState.angularVelocity;

      double xFeedback = m_translationController.calculate(pose.getX(), referenceState.x);
      double yFeedback = m_translationController.calculate(pose.getY(), referenceState.y);
      double rotationFeedback =
          m_rotationController.calculate(pose.getRotation().getRadians(), referenceState.heading);
      double omegaRadiansPerSecond =
          shootingOnTheMove.getAsBoolean() ? 0.0 : rotationFF + rotationFeedback;
      Rotation2d robotRotation =
          shootingOnTheMove.getAsBoolean()
              ? PoseEstimation.getInstance().getAimingParameters().driveHeading()
              : pose.getRotation();
      return ChassisSpeeds.fromFieldRelativeSpeeds(
          xFF + xFeedback, yFF + yFeedback, omegaRadiansPerSecond, robotRotation);
    };
  }

  private Command resetPose(String trajectoryName) {
    ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
    return Commands.runOnce(
        () ->
            m_drivetrain.resetPose.accept(
                Constants.onRedAllianceSupplier.getAsBoolean()
                    ? trajectory.getFlippedInitialPose()
                    : trajectory.getInitialPose()));
  }

  private Command intake() {
    return Superstructure.sensorIntake(m_feeder, m_intake)
        .andThen(m_intake.stop(), m_feeder.stop());
  }

  private Command aim() {
    return Superstructure.aimAtGoal(m_drivetrain, m_shooter, m_arm, m_lights)
        .alongWith(
            Commands.defer(
                () -> {
                  if (shootingWhileMoving) {
                    return Commands.startEnd(
                        () ->
                            m_drivetrain.setHeadingGoal(
                                () ->
                                    PoseEstimation.getInstance()
                                        .getAimingParameters()
                                        .driveHeading()),
                        m_drivetrain::clearHeadingGoal);
                  } else {
                    return m_drivetrain.blankDrive();
                  }
                },
                Set.of()))
        .withName("Aim");
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
                    m_drivetrain.atHeadingGoal.and(m_shooter.readyToShoot).or(state.hasTimedOut))
                .repeatedly())
        .beforeStarting(() -> state.timeoutTimer.start())
        .andThen(Commands.print("WARNING: Aim and shoot timed out").onlyIf(state.hasTimedOut))
        .andThen(m_arm.stop())
        .finallyDo(
            () -> {
              state.timeoutTimer.stop();
              state.timeoutTimer.reset();
              state.hasShot = false;
            })
        .withName("Aim and Shoot");
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
