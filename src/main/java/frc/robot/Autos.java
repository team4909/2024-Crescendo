package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmSetpoints;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.feeder.Feeder;
import frc.robot.intake.Intake;
import frc.robot.lights.Lights;
import frc.robot.shooter.Shooter;
import frc.robot.vision.GamePieceDetection;
import org.littletonrobotics.junction.Logger;

public class Autos {

  private final double kAimAndShootTimeoutSeconds = 3.0;

  private final Drivetrain m_drivetrain;
  private final Shooter m_shooter;
  private final Feeder m_feeder;
  private final Intake m_intake;
  private final Arm m_arm;
  private final Lights m_lights;
  private final GamePieceDetection m_gamePieceDetection;

  private final PIDController m_translationController = new PIDController(5.0, 0.0, 0.0);
  private final PIDController m_rotationController = new PIDController(5.0, 0.0, 0.0);
  private final PIDController m_gamePieceCorrectionController = new PIDController(0.04, 0.0, 0.0);

  public Autos(
      Drivetrain drivetrain,
      Shooter shooter,
      Feeder feeder,
      Intake intake,
      Arm arm,
      Lights lights,
      GamePieceDetection gamePieceDetection) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_feeder = feeder;
    m_intake = intake;
    m_arm = arm;
    m_lights = lights;
    m_gamePieceDetection = gamePieceDetection;
  }

  public Command subShot() {
    return shoot(false)
        .deadlineWith(m_arm.aimWrist(Arm.kSubwooferWristAngleRad))
        .withName("Sub Shot");
  }

  public Command sixPiece() {
    return Commands.sequence(
            resetPose("6Piece"),
            aimAndShoot(),
            intake().deadlineWith(getPathFollowingCommand("6Piece.1", false)),
            aimAndShoot(),
            intake().deadlineWith(getPathFollowingCommand("6Piece.2", false)),
            aimAndShoot(),
            intake().deadlineWith(getPathFollowingCommand("6Piece.3", false)),
            aimAndShoot(),
            intake().deadlineWith(getPathFollowingCommand("6Piece.4", false)),
            getPathFollowingCommand("6Piece.5", false),
            aimAndShoot(),
            intake().deadlineWith(getPathFollowingCommand("6Piece.6", false)),
            getPathFollowingCommand("6Piece.7", false),
            aimAndShoot())
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("Six Piece");
  }

  public Command threePieceSourceSide() {
    return Commands.sequence(
            resetPose("3PieceSourceSide"),
            subShot(),
            intake()
                .raceWith(
                    getPathFollowingCommand("3PieceSourceSide.1", true),
                    m_arm.goToSetpoint(ArmSetpoints.kStowed)),
            getPathFollowingCommand("3PieceSourceSide.2", false),
            stopIntake(),
            shoot(false),
            intake().raceWith(getPathFollowingCommand("3PieceSourceSide.3", true)),
            getPathFollowingCommand("3PieceSourceSide.4", false),
            stopIntake(),
            shoot(false))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("Three Piece Source Side");
  }

  public Command threePieceSourceSideLower() {
    return Commands.sequence(
            resetPose("3PieceSourceSideLower"),
            subShot(),
            intake()
                .raceWith(
                    getPathFollowingCommand("3PieceSourceSideLower.1", true),
                    m_arm.goToSetpoint(ArmSetpoints.kStowed)),
            getPathFollowingCommand("3PieceSourceSideLower.2", false),
            stopIntake(),
            shoot(false),
            intake().raceWith(getPathFollowingCommand("3PieceSourceSideLower.3", true)),
            getPathFollowingCommand("3PieceSourceSideLower.4", false),
            stopIntake(),
            shoot(false))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("Three Piece Source Side Lower");
  }

  public Command threePieceAmpSide() {
    return Commands.sequence(
            resetPose("3PieceAmpSide"),
            subShot(),
            intake()
                .raceWith(
                    getPathFollowingCommand("3PieceAmpSide.1", true),
                    m_arm.goToSetpoint(ArmSetpoints.kStowed)),
            getPathFollowingCommand("3PieceAmpSide.2", false),
            shoot(false),
            intake().raceWith(getPathFollowingCommand("3PieceAmpSide.3", true)),
            getPathFollowingCommand("3PieceAmpSide.4", false),
            shoot(false))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("Three Piece Amp Side");
  }

  private Command getPathFollowingCommand(
      String trajectoryName, ChoreoControlFunction controlFunction) {
    final ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
    return Choreo.choreoSwerveCommand(
            trajectory,
            PoseEstimation.getInstance()::getPose,
            controlFunction,
            m_drivetrain::runVelocity,
            Constants.onRedAllianceSupplier,
            m_drivetrain)
        .andThen(Commands.waitSeconds(0.5))
        .beforeStarting(() -> Logger.recordOutput("Drivetrain/Trajectory", trajectory.getPoses()));
  }

  private Command getPathFollowingCommand(String trajectoryName, boolean useGamePieceCorrection) {
    return getPathFollowingCommand(
        trajectoryName,
        choreoSwerveController(
            m_translationController,
            m_translationController,
            m_rotationController,
            useGamePieceCorrection));
  }

  private ChoreoControlFunction choreoSwerveController(
      PIDController xController,
      PIDController yController,
      PIDController rotationController,
      boolean useGamePieceCorrection) {
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    return (Pose2d pose, ChoreoTrajectoryState referenceState) -> {
      Logger.recordOutput(
          "Drivetrain/TrajectorySetpoint",
          new Pose2d(referenceState.x, referenceState.y, new Rotation2d(referenceState.heading)));
      final double xFF = referenceState.velocityX;
      final double yFF = referenceState.velocityY;
      final double rotationFF = referenceState.angularVelocity;

      final double xFeedback = xController.calculate(pose.getX(), referenceState.x);
      final double yFeedback, yCorrection;
      if (m_gamePieceDetection.hasValidTarget.getAsBoolean() && useGamePieceCorrection) {
        yCorrection =
            m_gamePieceCorrectionController.calculate(
                m_gamePieceDetection.horizontalErrorDeg.getAsDouble(), 0.0);
        yFeedback = 0.0;
      } else {
        yCorrection = 0.0;
        yFeedback = yController.calculate(pose.getY(), referenceState.y);
      }
      final double rotationFeedback =
          rotationController.calculate(pose.getRotation().getRadians(), referenceState.heading);
      return generateChassisSpeeds(
          xFF + xFeedback,
          yFF + yFeedback,
          rotationFF + rotationFeedback,
          pose.getRotation(),
          yCorrection);
    };
  }

  private ChassisSpeeds generateChassisSpeeds(
      double vxMetersPerSecond,
      double vyMetersPerSecond,
      double omegaRadiansPerSecond,
      Rotation2d robotAngle,
      double vyRobotRelativeCorrectionMetersPerSecond) {
    final Translation2d fieldRelativeAdjustedSpeeds =
        new Translation2d(vxMetersPerSecond, vyMetersPerSecond).rotateBy(robotAngle.unaryMinus());
    return new ChassisSpeeds(
        fieldRelativeAdjustedSpeeds.getX(),
        fieldRelativeAdjustedSpeeds.getY() + vyRobotRelativeCorrectionMetersPerSecond,
        omegaRadiansPerSecond);
  }

  private Command resetPose(String trajectoryName) {
    final ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
    return Commands.runOnce(
        () ->
            m_drivetrain.resetPose.accept(
                Constants.onRedAllianceSupplier.getAsBoolean()
                    ? trajectory.getFlippedInitialPose()
                    : trajectory.getInitialPose()));
  }

  private Command intake() {
    return Superstructure.sensorIntake(m_feeder, m_intake).andThen(stopIntake());
  }

  private Command stopIntake() {
    return Commands.sequence(m_intake.stop(), m_feeder.stop());
  }

  private Command aim() {
    return Superstructure.aimAtGoal(m_drivetrain, m_shooter, m_arm, m_lights)
        .alongWith(m_drivetrain.blankDrive())
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
