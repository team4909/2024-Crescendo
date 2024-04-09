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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.PoseEstimation;
import frc.robot.lights.Lights;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {

  private final Drivetrain m_drivetrain;
  private final Lights m_lights;
  private final ProfiledPIDController m_translationController, m_thetaController;
  private Translation2d m_lastSetpointTranslation;
  private Pose2d m_pose;
  private Pose2d m_goalPose;

  public DriveToPose(Pose2d pose, Drivetrain drivetrain, Lights lights) {
    m_pose = pose;
    m_translationController =
        new ProfiledPIDController(2.0, 0.0, 0.0, new TrapezoidProfile.Constraints(3.5, 2.2));
    m_thetaController =
        new ProfiledPIDController(
            5.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2 * Math.PI, 4 * Math.PI));
    this.m_drivetrain = drivetrain;
    this.m_lights = lights;
    addRequirements(drivetrain, lights);
  }

  @Override
  public void initialize() {
    m_lights.setBlink(Color.kWhite);
    m_goalPose =
        Constants.onRedAllianceSupplier.getAsBoolean()
            ? GeometryUtil.flipFieldPose(m_pose)
            : m_pose;
    Pose2d initialPose = PoseEstimation.getInstance().getPose();
    m_translationController.setTolerance(0.01);
    m_thetaController.setTolerance(Units.degreesToRadians(1.0));
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    Twist2d fieldVelocity = PoseEstimation.getInstance().getFieldVelocity();
    m_translationController.reset(
        initialPose.getTranslation().getDistance(m_goalPose.getTranslation()),
        Math.min(
            0.0,
            -new Translation2d(fieldVelocity.dx, fieldVelocity.dy)
                .rotateBy(
                    m_goalPose
                        .getTranslation()
                        .minus(initialPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    m_thetaController.reset(
        initialPose.getRotation().getRadians(),
        PoseEstimation.getInstance().getFieldVelocity().dtheta);
    m_lastSetpointTranslation = initialPose.getTranslation();
  }

  @Override
  public void execute() {
    Pose2d currentPose = PoseEstimation.getInstance().getPose();
    double distanceToGoalPose =
        currentPose.getTranslation().getDistance(m_goalPose.getTranslation());
    double ffScaler = MathUtil.clamp((distanceToGoalPose - 0.2) / (0.8 - 0.2), 0.0, 1.0);
    m_translationController.reset(
        m_lastSetpointTranslation.getDistance(m_goalPose.getTranslation()),
        m_translationController.getSetpoint().velocity);
    double driveVelocityScalar =
        m_translationController.getSetpoint().velocity * ffScaler
            + m_translationController.calculate(distanceToGoalPose, 0.0);
    if (distanceToGoalPose < m_translationController.getPositionTolerance())
      driveVelocityScalar = 0.0;
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
    double thetaErrorAbsolute =
        Math.abs(currentPose.getRotation().minus(m_goalPose.getRotation()).getRadians());
    if (thetaErrorAbsolute < m_thetaController.getPositionTolerance()) thetaVelocity = 0.0;

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

    Logger.recordOutput("Drivetrain/DriveToPose/DistanceToGoalPose", distanceToGoalPose);
    Logger.recordOutput(
        "Drivetrain/DriveToPose/Setpoint",
        new Pose2d(
            m_lastSetpointTranslation, new Rotation2d(m_thetaController.getSetpoint().position)));
    Logger.recordOutput("Drivetrain/DriveToPose/Goal", m_goalPose);
  }
}
