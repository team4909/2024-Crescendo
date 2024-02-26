package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.vision.Vision.VisionUpdate;
import org.littletonrobotics.junction.AutoLogOutput;

public class PoseEstimation {

  private static PoseEstimation m_instance;
  private final SwerveDrivePoseEstimator m_poseEstimator;
  private Twist2d m_robotVelocity = new Twist2d();
  private Twist2d m_fieldVelocity = new Twist2d();

  public PoseEstimation() {
    m_poseEstimator =
        new SwerveDrivePoseEstimator(
            Drivetrain.swerveKinematics,
            new Rotation2d(),
            new SwerveModulePosition[] {
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition()
            },
            new Pose2d());
  }

  public void addOdometryMeasurement(
      double timestamp, Rotation2d yawPosition, SwerveModulePosition[] modulePositions) {
    m_poseEstimator.updateWithTime(timestamp, yawPosition, modulePositions);
  }

  public void setVelocity(Twist2d robotVelocity) {
    m_robotVelocity = robotVelocity;
  }

  public void addVisionMeasurement(VisionUpdate visionUpdate) {
    m_poseEstimator.addVisionMeasurement(
        visionUpdate.pose(), visionUpdate.timestampSeconds(), visionUpdate.standardDeviations());
  }

  @AutoLogOutput(key = "PoseEstimation/Estimated Pose")
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  @AutoLogOutput(key = "PoseEstimation/FieldVelocity")
  public Twist2d getFieldVelocity() {
    final Translation2d linearFieldVelocity =
        new Translation2d(m_robotVelocity.dx, m_robotVelocity.dy).rotateBy(getPose().getRotation());
    return new Twist2d(
        linearFieldVelocity.getX(), linearFieldVelocity.getY(), m_robotVelocity.dtheta);
  }

  public Pose2d getPredictedPose(double translationLookaheadS, double rotationLookaheadS) {
    return getPose()
        .exp(
            new Twist2d(
                m_robotVelocity.dx * translationLookaheadS,
                m_robotVelocity.dy * translationLookaheadS,
                m_robotVelocity.dtheta * rotationLookaheadS));
  }

  public void resetPose(
      Rotation2d currentYawPosition,
      SwerveModulePosition[] currentModulePositions,
      Pose2d newPose) {
    m_poseEstimator.resetPosition(currentYawPosition, currentModulePositions, newPose);
  }

  public static PoseEstimation getInstance() {
    if (m_instance == null) m_instance = new PoseEstimation();
    return m_instance;
  }
}
