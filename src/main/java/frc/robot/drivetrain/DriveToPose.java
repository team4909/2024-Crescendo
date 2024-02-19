package frc.robot.drivetrain;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToPose extends Command {

  private final Drivetrain m_drivetrain;
  private final Pose2d m_goalPose;
  private final ProfiledPIDController m_translationController, m_thetaController;
  private Translation2d m_lastSetpointTranslation;

  public DriveToPose(Pose2d pose, Drivetrain drivetrain) {
    m_goalPose =
        drivetrain.onRedAllianceSupplier.getAsBoolean() ? GeometryUtil.flipFieldPose(pose) : pose;
    m_translationController =
        new ProfiledPIDController(2.0, 0.0, 0.0, new TrapezoidProfile.Constraints(3.5, 2.2));
    m_thetaController =
        new ProfiledPIDController(
            5.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2 * Math.PI, 4 * Math.PI));
    this.m_drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    m_translationController.setTolerance(0.01);
    m_thetaController.setTolerance(Units.degreesToRadians(1.0));
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    Twist2d fieldVelocity = m_drivetrain.getFieldVelocity();
    m_translationController.reset(
        m_drivetrain.getPose().getTranslation().getDistance(m_goalPose.getTranslation()),
        Math.min(
            0.0,
            -new Translation2d(fieldVelocity.dx, fieldVelocity.dy)
                .rotateBy(
                    m_goalPose
                        .getTranslation()
                        .minus(m_drivetrain.getPose().getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    m_thetaController.reset(
        m_drivetrain.getPose().getRotation().getRadians(), m_drivetrain.getYawVelocity());
    m_lastSetpointTranslation = m_drivetrain.getPose().getTranslation();
  }

  @Override
  public void execute() {
    Pose2d currentPose = m_drivetrain.getPose();
    double currentDistance = currentPose.getTranslation().getDistance(m_goalPose.getTranslation());
    double ffScaler = MathUtil.clamp((currentDistance - 0.2) / (0.8 - 0.2), 0.0, 1.0);
    m_translationController.reset(ffScaler);

    m_translationController.reset(
        m_lastSetpointTranslation.getDistance(m_goalPose.getTranslation()),
        m_translationController.getSetpoint().velocity);
    double driveVelocityScalar =
        m_translationController.getSetpoint().velocity * ffScaler
            + m_translationController.calculate(currentDistance, 0.0);
    if (currentDistance < m_translationController.getPositionTolerance()) driveVelocityScalar = 0.0;
    m_lastSetpointTranslation =
        new Pose2d(
                m_goalPose.getTranslation(),
                currentPose.getTranslation().minus(m_goalPose.getTranslation()).getAngle())
            .transformBy(
                new Transform2d(
                    new Translation2d(m_translationController.getSetpoint().position, 0.0),
                    new Rotation2d()))
            .getTranslation();

    double thetaVelocity =
        m_thetaController.getSetpoint().velocity * ffScaler
            + m_thetaController.calculate(
                currentPose.getRotation().getRadians(), m_goalPose.getRotation().getRadians());
    double thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(m_goalPose.getRotation()).getRadians());
    if (thetaErrorAbs < m_thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    Translation2d driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(m_goalPose.getTranslation()).getAngle())
            .transformBy(
                new Transform2d(new Translation2d(driveVelocityScalar, 0.0), new Rotation2d()))
            .getTranslation();
    m_drivetrain.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));
  }
}
