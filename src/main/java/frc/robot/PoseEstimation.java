package frc.robot;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.arm.Arm.Joint;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.vision.Vision.VisionUpdate;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

// why is this in our code? Shou;dn't this be in a library we use?
public class PoseEstimation {

  private static PoseEstimation m_instance;
  private final SwerveDrivePoseEstimator m_poseEstimator;
  private final BooleanSupplier lookaheadDisable = () -> false;
  private final double kLookaheadSeconds = 0.35;
  private final double jointSwitchDistanceMeters = Double.POSITIVE_INFINITY;
  private Twist2d m_robotVelocity = new Twist2d();
  private AimingParameters m_lastAimingParameters = null;

  private static final InterpolatingDoubleTreeMap wristAngleMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap elbowAngleMap = new InterpolatingDoubleTreeMap();

  static {
    wristAngleMap.put(1.2, 119.3);
    wristAngleMap.put(2.13, 139.0);
    wristAngleMap.put(2.22, 137.1);
    wristAngleMap.put(2.88, 142.0);
  }

  static {
    elbowAngleMap.put(2.4, -31.4);
    elbowAngleMap.put(4.07, -15.4);
  }

  public record AimingParameters(
      Rotation2d driveHeading,
      Rotation2d armAngle,
      double effectiveDistance,
      double driveFeedVelocity,
      Joint aimingJointIndex) {}

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
    m_lastAimingParameters = null;
    m_poseEstimator.updateWithTime(timestamp, yawPosition, modulePositions);
  }

  public void setVelocity(Twist2d robotVelocity) {
    m_lastAimingParameters = null;
    m_robotVelocity = robotVelocity;
  }

  public void addVisionMeasurement(VisionUpdate visionUpdate) {
    m_lastAimingParameters = null;
    m_poseEstimator.addVisionMeasurement(
        visionUpdate.pose(), visionUpdate.timestampSeconds(), visionUpdate.standardDeviations());
  }

  @AutoLogOutput(key = "PoseEstimation/EstimatedPose")
  public Pose2d getPose() {
    return m_poseEstimator
        .getEstimatedPosition()
        .transformBy(new Transform2d(Constants.poseOffset.toTranslation2d(), new Rotation2d()));
  }

  @AutoLogOutput(key = "PoseEstimation/FieldVelocity")
  public Twist2d getFieldVelocity() {
    final Translation2d linearFieldVelocity =
        new Translation2d(m_robotVelocity.dx, m_robotVelocity.dy).rotateBy(getPose().getRotation());
    return new Twist2d(
        linearFieldVelocity.getX(), linearFieldVelocity.getY(), m_robotVelocity.dtheta);
  }

  @AutoLogOutput(key = "PoseEstimation/PredictedPose")
  public Pose2d getPredictedPose(double translationLookaheadS, double rotationLookaheadS) {
    return getPose()
        .exp(
            new Twist2d(
                m_robotVelocity.dx * translationLookaheadS,
                m_robotVelocity.dy * translationLookaheadS,
                m_robotVelocity.dtheta * rotationLookaheadS));
  }

  public AimingParameters getAimingParameters() {
    if (m_lastAimingParameters != null) return m_lastAimingParameters;

    Translation2d speakerPosition = FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d();
    speakerPosition =
        Constants.onRedAllianceSupplier.getAsBoolean()
            ? GeometryUtil.flipFieldPosition(speakerPosition)
            : speakerPosition;
    final Transform2d fieldToTarget = new Transform2d(speakerPosition, new Rotation2d());
    final Pose2d fieldToPredictedVehicle =
        lookaheadDisable.getAsBoolean() || DriverStation.isAutonomousEnabled()
            ? getPose()
            : getPredictedPose(kLookaheadSeconds, kLookaheadSeconds);
    final Pose2d fieldToPredictedVehicleFixed =
        new Pose2d(fieldToPredictedVehicle.getTranslation(), new Rotation2d());

    final Translation2d predictedVehicleToTargetTranslation =
        poseInverse(fieldToPredictedVehicle).transformBy(fieldToTarget).getTranslation();
    final Translation2d predictedVehicleFixedToTargetTranslation =
        poseInverse(fieldToPredictedVehicleFixed).transformBy(fieldToTarget).getTranslation();

    /**
     * Rotate both angles by 180 degrees so that the back (shooter) points towards the target and
     * not the front (intake)
     */
    final Rotation2d vehicleToGoalDirection =
        predictedVehicleToTargetTranslation.getAngle().rotateBy(new Rotation2d(Math.PI));
    final Rotation2d targetVehicleDirection =
        predictedVehicleFixedToTargetTranslation.getAngle().rotateBy(new Rotation2d(Math.PI));

    double targetDistance = predictedVehicleToTargetTranslation.getNorm();

    double feedVelocity =
        m_robotVelocity.dx * vehicleToGoalDirection.getSin() / targetDistance
            - m_robotVelocity.dy * vehicleToGoalDirection.getCos() / targetDistance;

    Joint jointToAim = targetDistance < jointSwitchDistanceMeters ? Joint.kWrist : Joint.kElbow;
    double armAngle =
        jointToAim == Joint.kWrist
            ? wristAngleMap.get(targetDistance)
            : elbowAngleMap.get(targetDistance);

    m_lastAimingParameters =
        new AimingParameters(
            targetVehicleDirection,
            Rotation2d.fromDegrees(armAngle),
            targetDistance,
            feedVelocity,
            jointToAim);
    return m_lastAimingParameters;
  }

  public Rotation2d getFeedHeading() {
    return FieldConstants.stashPositionSupplier
        .get()
        .minus(getPose().getTranslation())
        .getAngle()
        .rotateBy(new Rotation2d(Math.PI));
  }

  private Pose2d poseInverse(Pose2d pose) {
    Rotation2d rotationInverse = pose.getRotation().unaryMinus();
    return new Pose2d(
        pose.getTranslation().unaryMinus().rotateBy(rotationInverse), rotationInverse);
  }

  public void resetPose(
      Rotation2d currentYawPosition,
      SwerveModulePosition[] currentModulePositions,
      Pose2d newPose) {
    m_poseEstimator.resetPosition(
        currentYawPosition,
        currentModulePositions,
        newPose.transformBy(
            new Transform2d(
                Constants.poseOffset.toTranslation2d().unaryMinus(), new Rotation2d())));
  }

  public static PoseEstimation getInstance() {
    if (m_instance == null) m_instance = new PoseEstimation();
    return m_instance;
  }
}
